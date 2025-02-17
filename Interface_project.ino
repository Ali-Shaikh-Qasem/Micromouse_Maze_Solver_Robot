#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>
#include "BluetoothSerial.h"

// --------------------------
// Bluetooth Configuration
// --------------------------
BluetoothSerial SerialBT;

// --------------------------
// Motor Control Pin Definitions
// --------------------------
#define MOTOR1_IN1 26
#define MOTOR1_IN2 27
#define MOTOR1_ENA 25  // PWM-capable GPIO pin

#define MOTOR2_IN3 14
#define MOTOR2_IN4 12
#define MOTOR2_ENA 13  // PWM-capable GPIO pin

// --------------------------
// Encoder Pin Definitions
// --------------------------
#define ENCODER1_A 32
#define ENCODER1_B 33
#define ENCODER2_A 18
#define ENCODER2_B 19

// IR sensor pins
#define IR_SENSOR_1 23
#define IR_SENSOR_2 4

// I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// VL53L0X object
VL53L0X lidar;

// --------------------------
// Global Variables
// --------------------------
MPU6050 mpu;

float yaw = 0; // Yaw angle
float Kp = 4.3199, Ki = 2.4, Kd = 2.755;
const float derivativeAlpha = 0.1;
float prev_error = 0;
int Base_PWM = 150; 
float cycleError = 0;
volatile int encoder_left_count = 0; 
volatile int encoder_right_count = 0; 
const float wheel_diameter = 4.4;
const int PPR = 210;              
const float wheel_circumference = 3.14159 * wheel_diameter;

// PID State Variables
float integral   = 0.0;
float derivative = 0.0;
unsigned long lastTime = 0;
const float maxIntegral = 1000.0;

float desiredYawAngle = 0;
float gyroBiasZ = 0;

// *** ADDED *** Maze size, robot state, visited array
static const int MAZE_WIDTH  = 8;
static const int MAZE_HEIGHT = 8;



//************* added part by Ali *****************//
// maze info
const int mazeWidth = 8, mazeHeight = 8;

int maze[mazeHeight][mazeWidth] = {
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,0,0,1,1,1},  // Center cells (3,3) and (3,4)
  {1,1,1,0,0,1,1,1},  // Center cells (4,3) and (4,4)
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1}
};

// Define robot orientation
enum Orientation {NORTH, SOUTH, EAST, WEST};
Orientation currentOrientation = NORTH; // to be modified based on the initial orientation of the robot.

// Define robot position
struct Position {
  int row;
  int col;
};
Position currentPosition = {7, 0}; // Start at bottom-left corner, to be modified based on the initial cell of the robot.

// --------------------------
// LEDC (PWM) Configuration
// --------------------------
void configureLEDC(uint8_t channel, uint8_t pin, uint32_t freq, uint8_t resolution) {
  // Configure LEDC timer
  ledc_timer_config_t ledcTimer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = (ledc_timer_bit_t)resolution,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = freq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledcTimer);

  // Configure LEDC channel
  ledc_channel_config_t ledcConfig = {
    .gpio_num       = pin,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = (ledc_channel_t)channel,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledcConfig);
}

// --------------------------
// LiDAR Functions
// --------------------------
void initLiDAR() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!lidar.init()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1);
  }
  lidar.setTimeout(500);
  lidar.startContinuous();
  Serial.println("LiDAR initialized.");
}

int readLiDAR() {
  int distance = lidar.readRangeContinuousMillimeters();
  if (lidar.timeoutOccurred()) {
    Serial.println("LiDAR timeout!");
    return -1;
  }
  return distance;
}

// --------------------------
// Motor & Encoder
// --------------------------
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  int leftDuty  = abs(leftSpeed);
  int rightDuty = abs(rightSpeed);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0, leftDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1, rightDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1);
}

void updateEncoderLeft()  { encoder_left_count++; }
void updateEncoderRight() { encoder_right_count++; }

void initialize_motors() {
  // Forward direction
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  setMotorSpeed(Base_PWM, Base_PWM);
}

void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  setMotorSpeed(0, 0);
}

// --------------------------
// MPU (Gyro) Functions
// --------------------------
void initializeMPU6050() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    return;
  }
  Serial.println("MPU6050 initialized successfully!");
}

float readYaw() {
  static unsigned long lastTime = 0;
  if (lastTime == 0) lastTime = micros();

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gyroZ = (gz / 131.0f) - gyroBiasZ; // Subtract bias

  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;
  if (deltaTime <= 0.0f || deltaTime > 1.0f) {
    deltaTime = 0.01f;
  }

  yaw += gyroZ * deltaTime;
  // wrap yaw in [0..360)
  if (yaw >= 360.0f) yaw -= 360.0f;
  if (yaw < 0.0f)    yaw += 360.0f;

  //SerialBT.printf("Yaw: %.2f, GyroZ: %.2f, DeltaTime: %.6f\n", yaw, gyroZ, deltaTime);
  return yaw;
}

// --------------------------
// Basic PID update
// --------------------------
void updatePID(float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0f;
  if (deltaTime <= 0.0f) deltaTime = 0.01f;

  integral += error * deltaTime;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  float derivativeRaw = (error - prev_error) / deltaTime;
  derivative = derivativeAlpha * derivativeRaw + (1.0f - derivativeAlpha) * derivative;

  prev_error = error;
  lastTime   = currentTime;
  
}

// --------------------------
// moveForward(...) with ramp & correction
// --------------------------

void moveForward(float target_distance) {

  // Reset encoder counts
  encoder_left_count = 0;
  encoder_right_count = 0;

  // --- Local (Distance) PID State Variables ---
  float distanceIntegral = 0;
  float distancePrevError = 0;
  unsigned long lastTimeDist = millis();
  
  // --- PID Gains (tune these values as needed) ---
  // Distance PID gains
  float Kp_dist = Kp;
  float Ki_dist = Ki;
  float Kd_dist = Kd;
  
  float Kp_head = Kp;
  float Ki_head = Ki;
  float Kd_head = Kd;
  
  integral = 0;
  prev_error = 0;
  derivative = 0;
  lastTime = millis();
  
  initialize_motors();
  
  while (true) {
    int avgPulses = (encoder_left_count + encoder_right_count) / 2;
    float distanceTraveled = (avgPulses / (float)PPR) * wheel_circumference;
    
    // Compute distance error (how far remaining)
    float distError = target_distance - distanceTraveled;
    
    if (fabs(distError) < 0.5) {
      break;
    }
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimeDist) / 1000.0f;
    if (dt < 0.01f) dt = 0.01f;
    lastTimeDist = currentTime;

    distanceIntegral += distError * dt;
    float distanceDerivative = (distError - distancePrevError) / dt;
    float forwardCommand = (Kp_dist * distError) + (Ki_dist * distanceIntegral) + (Kd_dist * distanceDerivative);
    distancePrevError = distError;
    
    // Constrain the forward command (this is the desired base speed)
    forwardCommand = constrain(forwardCommand, 0, Base_PWM);
    
    // Calculate a heading error based on the difference between left and right encoders.
    float headingError = (float)encoder_left_count - (float)encoder_right_count;
    updatePID(headingError); 
    float headingCorrection = (Kp_head * headingError) + (Ki_head * integral) + (Kd_head * derivative);
    
    int leftSpeed  = constrain((int)(forwardCommand - headingCorrection), 0, 255);
    int rightSpeed = constrain((int)(forwardCommand + headingCorrection), 0, 255);
    
    setMotorSpeed(leftSpeed, rightSpeed);
    delay(5);
  }
  
  stopMotors();
  updatePosition();
}

// --------------------------
// Turn Right / Turn Left
// --------------------------

// Helper: smallest angular difference in [-180..180]
float angleDiff(float from, float to) {
  float diff = fmodf((to - from + 540.0f), 360.0f) - 180.0f;
  return diff;
}

void turnRight(float targetAngle) {
  float initialYaw = readYaw();
  float rawTarget = initialYaw - targetAngle;
  if (rawTarget < 0)   rawTarget += 360.0f;
  if (rawTarget >= 360.0f) rawTarget -= 360.0f;

  float currentYaw = initialYaw;

  // Reset PID
  integral = 0;
  prev_error = 0;
  derivative = 0;
  lastTime = millis();

  unsigned long startTime = millis();
  SerialBT.printf("Turning Right: InitialYaw=%.2f, TargetYaw=%.2f\n", initialYaw, rawTarget);

  while (true) {
    if (millis() - startTime > 4000) {
      SerialBT.println("Turn Timeout! Stopping Motors.");
      stopMotors();
      break;
    }

    currentYaw = readYaw();
    float error = angleDiff(currentYaw, rawTarget);

    if (fabs(error) < 3.0f) {
      SerialBT.println("Turn Completed.");
      stopMotors();
      break;
    }

    updatePID(error);
    if (integral > 100)  integral = 100;
    if (integral < -100) integral = -100;

    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // negative => turn right, positive => turn left
    int leftDir  = (correction > 0) ? LOW : HIGH;
    int rightDir = (correction > 0) ? HIGH : LOW;
    digitalWrite(MOTOR1_IN1, leftDir);
    digitalWrite(MOTOR1_IN2, !leftDir);
    digitalWrite(MOTOR2_IN3, rightDir);
    digitalWrite(MOTOR2_IN4, !rightDir);

    int turnSpeed = 50 + (int)fabs(correction);
    turnSpeed = constrain(turnSpeed, 60, 140);

    setMotorSpeed(turnSpeed, turnSpeed);
    delay(10);
  }


  // update current orientation ******* new part ********
    if (targetAngle == 90){
      switch (currentOrientation) {
        case NORTH: currentOrientation = EAST; break;
        case EAST:  currentOrientation = SOUTH; break;
        case SOUTH: currentOrientation = WEST; break;
        case WEST:  currentOrientation = NORTH; break;
      }
    }
    else{ // this is when target angle is 180
      switch (currentOrientation) {
        case NORTH: currentOrientation = SOUTH; break;
        case EAST:  currentOrientation = WEST; break;
        case SOUTH: currentOrientation = NORTH; break;
        case WEST:  currentOrientation = EAST; break;
      }
    }
}

void turnLeft(float targetAngle) {
  float initialYaw = readYaw();
  float rawTarget = initialYaw + targetAngle;
  if (rawTarget < 0)      rawTarget += 360.0f; 
  if (rawTarget >= 360.0f) rawTarget -= 360.0f;

  float currentYaw = initialYaw;

  integral = 0;
  prev_error = 0;
  derivative = 0;
  lastTime = millis();

  unsigned long startTime = millis();
  SerialBT.printf("Turning Left: InitialYaw=%.2f, TargetYaw=%.2f\n", initialYaw, rawTarget);

  while (true) {
    if (millis() - startTime > 4000) {
      SerialBT.println("Turn Timeout! Stopping Motors.");
      stopMotors();
      break;
    }

    currentYaw = readYaw();
    float error = angleDiff(currentYaw, rawTarget);

    if (fabs(error) < 3.0f) {
      SerialBT.println("Turn Completed.");
      stopMotors();
      break;
    }

    updatePID(error);
    if (integral > 100)  integral = 100;
    if (integral < -100) integral = -100;

    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int leftDir  = (correction > 0) ? LOW : HIGH;
    int rightDir = (correction > 0) ? HIGH : LOW;
    digitalWrite(MOTOR1_IN1, leftDir);
    digitalWrite(MOTOR1_IN2, !leftDir);
    digitalWrite(MOTOR2_IN3, rightDir);
    digitalWrite(MOTOR2_IN4, !rightDir);

    int turnSpeed = 50 + (int)fabs(correction);
    turnSpeed = constrain(turnSpeed, 60, 140);

    setMotorSpeed(turnSpeed, turnSpeed);
    delay(10);
  }

   // update current orientaion ********** new part **********
    switch (currentOrientation) {
      case NORTH: currentOrientation = WEST; break;
      case WEST:  currentOrientation = SOUTH; break;
      case SOUTH: currentOrientation = EAST; break;
      case EAST:  currentOrientation = NORTH; break;
    }
}

// --------------------------
// Maze / Sensor functions
// --------------------------
int API_mazeWidth()  { return 8; }
int API_mazeHeight() { return 8; }

int API_wallFront() {
  int distance = readLiDAR();
  return (distance > 0 && distance < 130);
}

int API_wallRight() {
  int irRight = digitalRead(IR_SENSOR_1);
  // If your IR hardware returns HIGH for "wall" or "no wall," adapt as needed.
  return (irRight == LOW); // e.g. LOW means no wall or vice versa
}

int API_wallLeft() {
  int irLeft = digitalRead(IR_SENSOR_2);
  return (irLeft == LOW);
}



// ************** new part added by Ali *************************// 
void updatePosition() {
  switch (currentOrientation) {
    case NORTH: currentPosition.row -= 1; break;
    case SOUTH: currentPosition.row += 1; break;
    case EAST:  currentPosition.col += 1; break;
    case WEST:  currentPosition.col -= 1; break;
  }
}

void randomExplorerStep() {
  // Evaluate which directions are open
  bool rightOpen  = !API_wallRight();   // TRUE if NO wall on right
  bool leftOpen   = !API_wallLeft();    // TRUE if NO wall on left
  bool frontOpen  = !API_wallFront();   // TRUE if NO wall in front

  // Generate a random choice: 0 or 1
  int randomChoice = random(2);

  // =========== CASE 1: Both left & right open ===========
  if (leftOpen && rightOpen) {
    if (randomChoice == 0) {
      SerialBT.println("Random => Turn RIGHT + Forward");
      turnRight(90);
    } else {
      SerialBT.println("Random => Turn LEFT + Forward");
      turnLeft(90);
    }

    moveForward(19.6);        // move 20cm to next cell

  // =========== CASE 2: Exactly one side is open ===========
  } else if (leftOpen && !rightOpen) {
    if (randomChoice == 0 && frontOpen) {
      // 50% chance => forward
      SerialBT.println("Left open but picking FORWARD");
      moveForward(19.6);
    } else {
      // else => turn left
      SerialBT.println("Turn LEFT + Forward");
      turnLeft(90);
      moveForward(19.6);
    }

  } else if (rightOpen && !leftOpen) {
    // Right is open, left is blocked
    if (randomChoice == 0 && frontOpen) {
      SerialBT.println("Right open but picking FORWARD");
    } else {
      // else => turn right
      SerialBT.println("Turn RIGHT + Forward");
      turnRight(90);
      moveForward(19.6);
    }

  // =========== CASE 3: No side open, but front is open ===========
  } else if (frontOpen) {
    SerialBT.println("Forward is open => Move forward");
    moveForward(19.6);

  // =========== CASE 4: Dead End => turn around ===========
  } else {
    SerialBT.println("Dead end => Turn around 180");
    turnRight(180);
    moveForward(19.6);  // optional step forward
  }
}

void rightHandRule() {
  delay(2000);
  
  while (true) {
    if (maze[currentPosition.row][currentPosition.col] == 0) {
      stopMotors();
      break;
    }

    bool wallRight  = API_wallRight();
    bool wallFront  = API_wallFront();
    bool wallLeft   = API_wallLeft();

    SerialBT.print("Pos("); SerialBT.print(currentPosition.row); SerialBT.print(", ");
    SerialBT.print(currentPosition.col);
    SerialBT.print(" | WallRight="); SerialBT.print(wallRight);
    SerialBT.print(", WallFront="); SerialBT.print(wallFront);
    SerialBT.print(", WallLeft="); SerialBT.println(wallLeft);
    randomExplorerStep();
    delay(500);
  }
}

// --------------------------
// Setup & Loop
// --------------------------
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_PID_Controller");  // Bluetooth device name
  SerialBT.println("Bluetooth Initialized!");
  delay(3000);

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);

  configureLEDC(0, MOTOR1_ENA, 5000, 8);
  configureLEDC(1, MOTOR2_ENA, 5000, 8);

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoderRight, RISING);

  // IR sensors
  pinMode(IR_SENSOR_1, INPUT_PULLUP);
  pinMode(IR_SENSOR_2, INPUT_PULLUP);

  initializeMPU6050();
  initLiDAR();
  
}
void loop() {
  // Right-hand rule solver
  rightHandRule();

  // Once done, do nothing
  while (1) {
    delay(1000);
  }
}