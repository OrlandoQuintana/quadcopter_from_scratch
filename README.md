# **Quadcopter From Scratch**

## **Overview**

This project implements a fully functional quadcopter system from scratch, using **Rust** for firmware development and **ROS2** for distributed processing and communication. The goal is to create a modular, extensible, and high-performance quadcopter platform that can stabilize itself in flight, process sensor data, and handle user input.

The project is divided into two main packages:
1. **IMU Processing Package**: Handles sensor data acquisition, filtering, and fusion.
2. **Motor Control Package**: Implements control algorithms and motor command generation.

---

## **Technologies Used**

### **Programming Languages**
- **Rust**: Core development language for high-performance, memory-safe firmware and ROS2 nodes.
- **C/C++**: Possible extensions or integrations where high-performance libraries like OpenCV or specific ROS2 features are required.

### **Frameworks and Libraries**
- **ROS2**: Middleware framework for managing communication between nodes.
- **rclrs**: Rust client library for ROS2.
- **embedded-hal**: Rust-based hardware abstraction layer for embedded systems.
- **linux-embedded-hal**: SPI and GPIO support for the Raspberry Pi.
- **icm-20948-driver**: Custom Rust crate for interacting with the ICM-20948 IMU sensor.

### **Hardware**
- **ICM-20948 IMU**: Provides accelerometer and gyroscope data for flight stabilization.
- **Raspberry Pi**: Serves as the main flight controller.
- **Electronic Speed Controllers (ESCs)**: Drive the quadcopter motors.
- **XBox Controller**: Used for user input to control orientation and throttle.

---

## **Project Architecture**

The quadcopter system is modular, with individual ROS2 nodes performing specific tasks. Below is an explanation of the architecture, based on the provided system diagram:

### **1. IMU Processing Package**
#### **Purpose**
- To process raw IMU sensor data and estimate the quadcopter's orientation in real-time.

#### **Components**
1. **`imu_publisher` Node**
   - Reads raw accelerometer and gyroscope data from the ICM-20948 over SPI using the custom Rust driver.
   - Publishes raw IMU data to the `/raw_imu` ROS2 topic.
   
2. **`sensor_fusion` Node**
   - Subscribes to `/raw_imu` to receive raw accelerometer and gyroscope data.
   - Uses an **Extended Kalman Filter (EKF)** to fuse the data and estimate roll, pitch, and yaw angles.
   - Publishes the estimated orientation to the `/estimated_orientation` topic.

---

### **2. Motor Control Package**
#### **Purpose**
- To stabilize the quadcopter in flight by adjusting motor speeds based on desired orientation and throttle inputs.

#### **Components**
1. **`controller_input` Node**
   - Reads user input from an XBox controller connected via USB.
   - Publishes the desired orientation and throttle to the `/desired_orientation` and `/throttle` topics.

2. **`pid_controller` Node**
   - Subscribes to `/estimated_orientation`, `/desired_orientation`, and `/throttle`.
   - Implements a **cascaded PID control system** to calculate the necessary motor adjustments to achieve stable flight.
   - Publishes motor commands to the `/calculated_motor_commands` topic.

3. **`motor_command` Node**
   - Subscribes to `/calculated_motor_commands`.
   - Converts the PID controller output into **PWM signals** to drive the ESCs and control motor speeds.

---

## **Workflow Explanation**

1. **IMU Data Acquisition**:
   - The ICM-20948 IMU provides raw accelerometer and gyroscope data.
   - The `imu_publisher` node reads this data via SPI and publishes it to the `/raw_imu` topic.

2. **Sensor Fusion**:
   - The `sensor_fusion` node fuses accelerometer and gyroscope data using an EKF to compute the quadcopter's orientation (roll, pitch, yaw).
   - The fused data is published to the `/estimated_orientation` topic.

3. **User Input**:
   - The `controller_input` node reads user commands (desired roll, pitch, yaw, and throttle) from an XBox controller.
   - These commands are published to the `/desired_orientation` and `/throttle` topics.

4. **Control System**:
   - The `pid_controller` node calculates the motor adjustments required to align the quadcopter's current orientation with the desired orientation while accounting for throttle.
   - The resulting motor commands are published to the `/calculated_motor_commands` topic.

5. **Motor Commands**:
   - The `motor_command` node converts motor adjustment commands into PWM signals to drive the quadcopter's motors via ESCs.

---

## **Installation and Setup**

### **Prerequisites**
1. **Rust**:
   - Install using [rustup](https://rustup.rs/):
     ```bash
     curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
     ```
2. **ROS2 Humble**:
   - Follow the [installation guide](https://docs.ros.org/en/humble/Installation.html).

3. **Enable SPI** on the Raspberry Pi:
   ```bash
   sudo raspi-config
   ```
   Navigate to **Interfacing Options → SPI** and enable it.

4. **Clone the Repository**:
   ```bash
   git clone https://github.com/your-repo/quadcopter_from_scratch.git
   cd quadcopter_from_scratch
   ```

### **Build and Run**
1. **Build the Rust Nodes**:
   ```bash
   cargo build --release
   ```

2. **Run the Nodes**:
   - Start the ROS2 nodes individually or use a launch file for automation.
   - Example: Start the IMU publisher node:
     ```bash
     cargo run --release --bin imu_publisher
     ```

---

## **Topics**

| **Topic Name**          | **Message Type**          | **Description**                                                   |
|--------------------------|---------------------------|-------------------------------------------------------------------|
| `/raw_imu`              | `sensor_msgs/msg/Imu`     | Raw accelerometer and gyroscope data from the ICM-20948 IMU.      |
| `/estimated_orientation` | `geometry_msgs/msg/Pose` | Fused roll, pitch, and yaw data estimated via sensor fusion.      |
| `/desired_orientation`   | `geometry_msgs/msg/Pose` | User-specified desired orientation (roll, pitch, yaw).            |
| `/throttle`              | `std_msgs/msg/Float64`   | User-specified throttle value.                                    |
| `/calculated_motor_commands` | `std_msgs/msg/Float64MultiArray` | Motor adjustment commands from the PID controller.               |

---

## **Future Enhancements**
1. **Magnetometer Integration**:
   - Incorporate the ICM-20948's magnetometer for more accurate heading estimation.

2. **Computer Vision**:
   - Add a wide angle depth camera and utilize computer vision libraries in C++ or Python for obstacle detection, SLAM, yaw angle correction, and more

3. **Autonomous Features**:
   - Add GPS integration for waypoint navigation and autonomous flight modes.

4. **Flight Simulation**:
   - Use Gazebo or similar simulators to test the quadcopter's behavior in a virtual environment.

---

## **Conclusion**

This project demonstrates a modular and scalable approach to building a quadcopter system entirely from scratch using Rust and ROS2. By leveraging modern technologies and custom hardware drivers, the project creates a foundation for further research and development in robotics and UAVs.

Feel free to contribute or modify the system to add new features and functionality!
