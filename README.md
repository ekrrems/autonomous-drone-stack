# Autonomous Drone Stack  
*A step-by-step implementation of the core algorithms needed to build an autonomous drone from the ground up.*

This repository contains my personal end-to-end journey in robotics, state estimation, sensor fusion, and drone control.  
Each module is built **from scratch**, explained clearly, and implemented in a way that can later run on embedded hardware (ESP32) and integrate with ROS 2.

The project is organized into branches, where every branch focuses on one core part of the drone pipeline.

---

## Geometric Controller of Quatrotor Drone (SE(3)-based Control)

The goal is to enable a quadrotor to track a desired position and yaw angle, using a control law that computes the required thrust and torque direclty from position,
velocity, and orientation errors. The controller is robust to aggressive maneuvers and maintains full control of both translation and rotation.

A short demo video showing the drone with initial velocity trying to find the desired position and hover on it:

➡ ![video](https://github.com/ekrrems/autonomous-drone-stack/blob/ekf_fusion/data/images/Screen_Recording.gif?raw=true)

The video shows:
- Thrown drone with initial velocity
- Hovering and controlling itself on desired position
---

## Project Goals

- Geometric tracking control on SE(3)
- Thrust and attitude control
- Torque control
- Realistic simulation environment

---

## 🧭 Geometric Controller Module
This branch (`geometric_controller`) implements:

### **1. Thrust and Attitude Control**
- The drone computes the desired thrust direction based on position and velocity errors.
- A desired rotation matrix is build so the drone tilts toward this thrust direction, while aligning tis yaw with a given heading.
- The resulting thrust and desired orientation are used to control the drone in 3D space.

### **2. Torque Control**

- The drone calculate orientation and angular velocity errors by comparing its current orientation with the desired one.
- Using these errors, it computes the necessary torque to rotate itself into the correct pose.
- The torque control includes inertia, feedwoward, and gyroscopic compensation to ensure stability.

### **3. Simulation Environment**
- The system is simualted in MATLAB using a custom drone model and the uavScenario 3D envrionment.
- The drone is initialized at a high altitute and commanded to hover or follow a simple trajectory.
- The controller stabilizes the drone using only internal feedback(No ground truth corrections).
- All simulations are visualized in 3D, and the thrust/torque outputs are monitored over time.

---

## Outcomes

- The drone successfully stabilizes and follows simple target positions.
- Orientation remains controlled during flight, with correct yaw alignment.
- The simulation demostrate that geometric control can handle full SE(3) motion, including rotation and aggressive translations.
- This module forms one of the key basis for **fully autonomous flight** in later branches.

## Technology Stack

- **MATLAB** for rapid prototyping  
- **ROS 2** for real robotics communication  
- **OpenVINS** for visual-inertial odometry  
- **TCP networking** to fuse real-time sensor streams  
- **ESP32** (coming in later branches) for embedded IMU + camera + motor control  
- **Future:** Control loops, onboard control, SLAM, planning, navigation

---
