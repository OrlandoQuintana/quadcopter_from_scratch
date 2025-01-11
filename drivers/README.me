# **ICM-20948 Driver**

## **Overview**
This project provides a complete Rust driver for the **ICM-20948 IMU** connected to a **Raspberry Pi** via the SPI communication protocol. It supports initialization of the IMU and provides abstractions for reading data from the accelerometer and gyroscope.

The driver is modular, portable, and thread-safe, leveraging the **embedded-hal** and **linux-embedded-hal** crates for abstraction and compatibility across platforms.

---

## **Hardware Requirements**
### **Required Components**
- **ICM-20948 IMU**
  - A 9-axis inertial measurement unit featuring:
    - Accelerometer (3-axis).
    - Gyroscope (3-axis).
    - Magnetometer (3-axis, future support planned).
- **Raspberry Pi**
  - Any Linux-capable Raspberry Pi model with SPI support (e.g., Raspberry Pi 4, Raspberry Pi Zero W).

---

## **Wiring**
The ICM-20948 IMU connects to the Raspberry Pi as follows:

| **Raspberry Pi Pin** | **ICM-20948 Pin** | **Function**            |
|-----------------------|-------------------|-------------------------|
| GPIO 10 (MOSI)        | SDI              | Master Out, Slave In    |
| GPIO 9 (MISO)         | SDO              | Master In, Slave Out    |
| GPIO 11 (SCLK)        | SCL              | Serial Clock            |
| GPIO 8 (CE0)          | CS               | Chip Select             |
| GND                   | GND              | Ground                  |
| 3.3V                  | VDD              | Power Supply (3.3V)     |

To enable SPI on the Raspberry Pi:
```bash
sudo raspi-config
```
Navigate to **Interfacing Options → SPI** and enable it.

Verify SPI devices are available:
```bash
ls /dev/spidev*
```

---

## **Crates Used**
### **Key Dependencies**
1. **`embedded-hal`**:
   - A hardware abstraction layer for embedded systems.
   - Defines traits for peripherals like SPI, I²C, GPIO, etc.

2. **`linux-embedded-hal`**:
   - Implements the `embedded-hal` traits for Linux systems, enabling SPI communication through `spidev`.

3. **`std::sync::{Arc, Mutex}`**:
   - Ensures thread-safe shared access to the SPI bus between the accelerometer and gyroscope.

Install the necessary crates in your `Cargo.toml`:
```toml
[dependencies]
embedded-hal = "0.2"
linux-embedded-hal = "0.6"
```

---

## **Driver Architecture**
The driver consists of the following layers:

1. **Hardware Layer**:
   - SPI hardware on the Raspberry Pi communicates with the ICM-20948.

2. **SPI Core Abstraction (`SpiCore`)**:
   - A reusable interface for register-based SPI communication.

3. **IMU Initialization Layer (`IMU`)**:
   - Handles hardware initialization (e.g., power management, filters, sensitivity).

4. **Sensor-Specific Abstraction**:
   - **`Accelerometer`**:
     - Reads and processes linear acceleration data (m/s²).
   - **`Gyroscope`**:
     - Reads and processes angular velocity data (radians/second).

---

## **Code Structure**
```plaintext
src/
├── main.rs              # Example usage
├── spi_core.rs          # SPI core abstraction
├── imu/                 # IMU module
│   ├── mod.rs           # IMU initialization and sensor management
│   ├── accelerometer.rs # Accelerometer logic
│   └── gyroscope.rs     # Gyroscope logic
```

---

## **Example Usage**
Here’s an example `main.rs` that demonstrates how to use the driver:

```rust
use crate::imu::{IMU, Accelerometer, Gyroscope};
use crate::spi_core::SpiCore;
use linux_embedded_hal::Spidev;
use std::sync::{Arc, Mutex};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Step 1: Initialize SPI communication
    let spidev = Spidev::open("/dev/spidev0.0")?;
    let spi_core = Arc::new(Mutex::new(SpiCore::new(spidev)));

    // Step 2: Initialize the IMU
    let mut imu = IMU::new(spi_core.clone());
    imu.initialize()?;

    // Step 3: Create instances of Accelerometer and Gyroscope
    let mut accelerometer = Accelerometer::new(spi_core.clone());
    let mut gyroscope = Gyroscope::new(spi_core.clone());

    // Step 4: Read data from accelerometer
    let accel_data = accelerometer.read()?;
    println!("Acceleration: x = {}, y = {}, z = {}", accel_data[0], accel_data[1], accel_data[2]);

    // Step 5: Read data from gyroscope
    let gyro_data = gyroscope.read()?;
    println!("Angular velocity: x = {}, y = {}, z = {}", gyro_data[0], gyro_data[1], gyro_data[2]);

    Ok(())
}
```

---

## **Features**
- **Portability**: 
  - Built on `embedded-hal`, the driver works with any platform that supports the standard HAL traits.
- **Thread-Safe**:
  - Shared SPI access is protected using `Arc<Mutex<SpiCore>>`.
- **Modular Design**:
  - Separate abstractions for SPI communication, initialization, and individual sensors.
- **Scalable**:
  - Future enhancements could include support for the ICM-20948’s magnetometer and sensor fusion algorithms.

---

## **Future Enhancements**
1. Add support for the ICM-20948 magnetometer.
2. Implement configuration options for sensitivity and filter settings.
3. Introduce advanced features like sensor fusion for calculating roll, pitch, and yaw.

---

## **How to Run**
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/icm-20948-driver.git
   cd icm-20948-driver
   ```
2. Run the example program:
   ```bash
   cargo run --example main
   ```

---
