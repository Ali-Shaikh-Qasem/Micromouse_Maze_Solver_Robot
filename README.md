# 🚀 Maze-Solving Robot

## 🏆 Project Overview
This project is an advanced **maze-solving robot**, developed as part of the **Interfacing Techniques course**. The robot is designed to autonomously navigate complex mazes using a combination of **sensor fusion, precise motion control, and an intelligent navigation algorithm**. 

## 🔧 Features & Technical Highlights

### ✅ **Microcontroller & Sensors**
- **ESP32**: Handles real-time processing, communication, and control.
- **MPU6050**: Provides accurate orientation feedback.
- **VL53L0X LiDAR**: Detects front walls and obstacles.
- **2 x IR Sensors**: Detect side walls for fine adjustments.
- **Wheel Encoders**: Enable precise distance tracking and speed control.

### ✅ **Motion Control System**
- **Dual-PID Forward Motion Control**:
  - One PID controller minimizes **target distance error**.
  - Another PID controller ensures **orientation correction** for straight-line movement.
- **Loop-Feedback PID Rotation Control**:
  - Corrects **angle errors** for smooth and accurate turns.

### ✅ **Maze-Solving Algorithm**
- Implements a **hybrid approach** of **right-hand and left-hand rules**.
- Includes a **loop escape mechanism** to prevent infinite loops and optimize pathfinding.

### ✅ **Dynamic Parameter Optimization & Noise Reduction**
- Uses an **advanced gradient descent algorithm** to fine-tune control parameters in real-time.
- A **moving average filter** acts as a **low-pass filter**, reducing high-frequency oscillations for smoother motion control.

## ⚙️ Hardware Components
| Component       | Description                        |
|----------------|----------------------------------|
| **ESP32**      | Microcontroller for processing & control |
| **MPU6050**    | IMU for orientation feedback  |
| **VL53L0X**    | LiDAR for front obstacle detection |
| **2 x IR Sensors** | Detect side walls for navigation |
| **2 x Motors with Encoders** | Enable precise motion control |
| **L298N**      | Motor driver module |
| **9V Battery** | Power supply for the system |

## 📌 Team Members
This project was developed by:
- **Ali Shaikh Qasem**
- **Abdalrahman Jaber**
- **Khaled Rimawi**
- **Hasan Khaled**

## 🔄 Future Improvements
- Implement **path optimization algorithms** (e.g., flood-fill) for shorter routes.
- Add **wireless communication** for real-time monitoring and debugging.
- Improve **power efficiency** for extended operation.

## 📜 License
This project is open-source under the **MIT License**. Feel free to contribute and improve!



