# Autonomous Drone Stack  
*A step-by-step implementation of the core algorithms needed to build an autonomous drone from the ground up.*

This repository contains my personal end-to-end journey in robotics, state estimation, sensor fusion, and drone control.  
Each module is built **from scratch**, explained clearly, and implemented in a way that can later run on embedded hardware (ESP32) and integrate with ROS 2.

The project is organized into branches, where every branch focuses on one core part of the drone pipeline.

---

## EKF Demo Video

A short demo video showing the EKF prediction & update running live is included here:

➡ ![video](https://github.com/ekrrems/autonomous-drone-stack/blob/ekf_fusion/data/images/Screen_Recording.gif?raw=true)

The video shows:
- OpenVINS poses (red)
- EKF estimated trajectory (blue)

---

## Project Goals

- Implement an **IMU-based Extended Kalman Filter (EKF)** from scratch  
- Fuse real IMU data with visual odometry (OpenVINS)  
- Build a complete **state estimation pipeline**  
- Add **PID control** for brushless motors (next branch)  
- Build a full **autonomous flight stack**  
- Eventually integrate everything into ROS 2 + SLAM + onboard ESP32 systems  
- Document everything transparently for learning & reproducibility

---
## 🧭 EKF Module (This Branch)

This branch (`ekf`) implements:

### **1. IMU Preprocessing**
- Transform IMU readings from **IMU frame (X fwd, Y right, Z down)**  
  into **ENU (East, North, Up)**.
- Bias initialization from static IMU samples.
- Gravity compensation in ENU.

### **2. Nominal State Propagation**
We propagate:

- Position  
- Velocity  
- Orientation (via quaternion exponential map)  
- Accelerometer bias  
- Gyroscope bias  

Using continuous dynamics and first-order discretization.

### **3. Error-State EKF**
We maintain a **15-dimensional error state*
[ δp, δv, δθ, δba, δbg ]

And update it with:

- Measurement Jacobian  
- Innovation  
- Kalman Gain  
- Joseph-form covariance update (numerically stable)

### **4. OpenVINS Position Updates**
We receive pose messages from OpenVINS via TCP and fuse them as a **position-only update** in the EKF.

This simulates how a real drone would fuse:
- IMU (high rate)  
- Visual odometry (lower rate)  

---

## Technology Stack

- **MATLAB** for rapid prototyping  
- **ROS 2** for real robotics communication  
- **OpenVINS** for visual-inertial odometry  
- **TCP networking** to fuse real-time sensor streams  
- **ESP32** (coming in later branches) for embedded IMU + camera + motor control  
- **Future:** PID loops, onboard control, SLAM, planning, navigation

---

## Branch Structure

Every branch implements one self-contained part of the drone:

- `ekf_fusion` → IMU-based error-state EKF 
- `pid` → PID motor control  
- `motor_drivers` → ESC + PWM control  
- `quad_dynamics` → full 6-DoF drone dynamics  
- `sensor_fusion` → EKF + camera + IMU fusion  
- `slam` → monocular / stereo SLAM  
- `ros_integration` → ROS 2 nodes, topics, visualization  
- `autonomous_flight` → full drone autonomy

---
