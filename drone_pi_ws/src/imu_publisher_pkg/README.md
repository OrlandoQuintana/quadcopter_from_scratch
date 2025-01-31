# **ROS2 IMU Publisher Node**

## **Overview**
This project implements a **ROS2 node** in Rust that reads accerelomerer and gyroscope data from the **ICM-20948 IMU** and publishes it to a ROS2 topic `/raw_imu`. The node uses a custom Rust driver (`icm-20948-driver`) to interface with the IMU over SPI and follows the `sensor_msgs/msg/Imu` message format for publishing.

The node provides:
- Real-time accelerometer and gyroscope data.
- Thread-safe SPI communication.
- Modular abstractions for IMU initialization, accelerometer, and gyroscope-specific logic.

---

## **Features**
1. **Custom Rust Driver Integration**:
   - Utilizes the `icm-20948-driver` crate, which provides modular abstractions for initializing and accessing data from the IMU sensors.
   - Handles SPI communication via the `embedded-hal` and `linux-embedded-hal` crates.

2. **ROS2 Sensor Messages**:
   - Publishes data in the standard `sensor_msgs/msg/Imu` format, which is widely compatible with robotics applications.

3. **High-Frequency Publishing**:
   - Publishes IMU data to the topic `/raw_imu` at approximately **200 Hz**.

4. **Thread-Safe Design**:
   - Uses `Arc<Mutex<SpiCore>>` to ensure safe concurrent access to the SPI bus.

---

## **Requirements**

### **Software**
- **Rust** 

- **ROS2 Humble** or newer:
  - Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

- **Enable SPI on the Raspberry Pi**:
  1. Open the Raspberry Pi configuration tool:
     ```bash
     sudo raspi-config
     ```
  2. Navigate to **Interfacing Options → SPI** and enable it.

  3. Verify SPI devices are available:
     ```bash
     ls /dev/spidev*
     ```
     Ensure you see `/dev/spidev0.0`.

### **Hardware**
- **ICM-20948 IMU**: A 9-axis inertial measurement unit.
- **Raspberry Pi**: Any model with SPI support (e.g., Raspberry Pi 4, Raspberry Pi Zero W).

### **Wiring**
| **Raspberry Pi Pin** | **ICM-20948 Pin** | **Function**            |
|-----------------------|-------------------|-------------------------|
| GPIO 10 (MOSI)        | SDI              | Master Out, Slave In    |
| GPIO 9 (MISO)         | SDO              | Master In, Slave Out    |
| GPIO 11 (SCLK)        | SCL              | Serial Clock            |
| GPIO 8 (CE0)          | CS               | Chip Select             |
| GND                   | GND              | Ground                  |
| 3.3V                  | VDD              | Power Supply (3.3V)     |

---

## **Crates Used**
1. **`rclrs`**:
   - Provides ROS2 bindings for Rust.
   - Used to create nodes, publishers, and interact with ROS2 topics.

2. **`icm-20948-driver`**:
   - A custom Rust crate for interfacing with the ICM-20948 IMU.
   - Handles IMU initialization and sensor-specific functionality (e.g., reading accelerometer and gyroscope data).

3. **`linux-embedded-hal`**:
   - Implements the `embedded-hal` traits for Linux systems.
   - Provides SPI communication via the Raspberry Pi's `/dev/spidev0.0`.

4. **Standard Sync Utilities**:
   - Uses `Arc` and `Mutex` for thread-safe access to the shared SPI bus.

Add these dependencies to your `Cargo.toml`:
```toml
[dependencies]
rclrs = "0.10"
linux-embedded-hal = "0.6"
icm20948-driver-rust = { git = "https://github.com/OrlandoQuintana/icm20948-driver-rust" }
```

---

## **Node Details**
### **Node Name**
- `imu_publisher`

### **Published Topic**
- **`/raw_imu`**
  - Message type: `sensor_msgs/msg/Imu`.
  - Contains:
    - `linear_acceleration`: Acceleration in m/s² along X, Y, and Z axes.
    - `angular_velocity`: Angular velocity in rad/s along X, Y, and Z axes.

### **Viewing the Published Data**
To see the data being published:
1. Open a terminal and run:
   ```bash
   ros2 topic echo /raw_imu
   ```
2. Example output:
   ```yaml
   header:
     stamp:
       sec: 1635877468
       nanosec: 123456789
     frame_id: ''
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
   angular_velocity: {x: 0.01, y: -0.02, z: 0.05}
   linear_acceleration: {x: 0.98, y: 0.02, z: 9.81}
   ```

---

## **Code Overview**
### **`IMUPublisherNode`**
A custom struct encapsulating:
- A ROS2 node (`imu_publisher`).
- A publisher for the `/raw_imu` topic.
- The IMU components (`IMU`, `Accelerometer`, `Gyroscope`).

### **Initialization**
- The `IMUPublisherNode::new` method initializes the ROS2 node, SPI bus, and IMU components.
- IMU initialization is explicitly handled via `IMUPublisherNode::initialize_imu`.

### **Data Publishing**
- Reads data from the accelerometer and gyroscope using the custom driver.
- Applies a Butterworh low-pass filter to the accelerometer data. Sammpling frquency here must match the real sampling frequency (200 Hz in this case)
- Applies calibrationi offset to gyroscope data. To find the offsets, read the gyroscope data rest for a set period of time. Calculate the averagae reading over the period of time and use that as the offset. There plans to add automatic calibration but it is done manually for now.
- Publishes IMU data in `sensor_msgs/msg/Imu` format via the `/raw_imu` topic.

---

## **Main Components**
### **IMU Initialization**
The IMU is initialized with the following steps:
1. Configure power management to enable the accelerometer and gyroscope.
2. Set sensitivity for:
   - Accelerometer: ±2g.
   - Gyroscope: ±250 dps.
3. Configure low-pass filters for noise reduction.

### **Publishing Logic**
The `publish_data` method:
1. Reads data from the accelerometer (`linear_acceleration`) and gyroscope (`angular_velocity`).
2. Populates the `ImuMsg` with sensor data.
3. Publishes the message to the `/raw_imu` topic.

---

## **Future Enhancements**
1. Add support for the ICM-20948 magnetometer.
2. Implement additional configuration options for IMU sensitivity and filters.
3. Add some method of auto gyroscope calibration.

---

