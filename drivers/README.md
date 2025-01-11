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
embedded-hal = "1.0.0"
linux-embedded-hal = "0.4"
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
use icm_20948_driver::imu::{Accelerometer, Gyroscope, IMU};
use icm_20948_driver::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags};
use linux_embedded_hal::SpidevBus;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Step 1: Create and configure the SPI device
    let mut spidev = Spidev::open("/dev/spidev0.0").expect("Failed to open SPI device");
    spidev
        .configure(
            &SpidevOptions::new()
                .bits_per_word(8)
                .max_speed_hz(1_000_000) // 1 MHz
                .mode(SpiModeFlags::SPI_MODE_0)
                .build(),
        )
        .expect("Failed to configure SPI device");

    // Step 2: Wrap Spidev in a SpidevBus
    let spidev_bus = SpidevBus::from(linux_embedded_hal::SpidevBus(spidev));

    // Step 3: Create an Arc<Mutex<SpiCore>> to share SPI access
    let spi = Arc::new(Mutex::new(SpiCore::new(spidev_bus)));

    // Step 4: Create and initialize the central IMU
    let mut imu = IMU::new(Arc::clone(&spi));
    imu.initialize().expect("Failed to initialize IMU");

    // Step 5: Create accelerometer, gyroscope
    let mut accel = Accelerometer::new(Arc::clone(&spi));
    let mut gyro = Gyroscope::new(Arc::clone(&spi));

    // Step 6: Main loop to read data
    let mut _loop_count = 0; // Loop counter

    loop {
        let loop_start = Instant::now();

        // Accelerometer Logic
        match accel.read() {
            Ok(accel_data) => println!(
                "Accelerometer - X: {:.2}, Y: {:.2}, Z: {:.2}",
                accel_data[0], accel_data[1], accel_data[2]
            ),
            Err(e) => eprintln!("Failed to read accelerometer data: {:?}", e),
        }
    
        // Gyroscope Logic
        match gyro.read() {
            Ok(gyro_data) => println!(
                "Gyroscope - X: {:.2}, Y: {:.2}, Z: {:.2}",
                gyro_data[0], gyro_data[1], gyro_data[2]
            ),
            Err(e) => eprintln!("Failed to read gyroscope data: {:?}", e),
        }
    
        // Maintain 2.5ms loop duration
        let elapsed = loop_start.elapsed();
        if elapsed < Duration::from_micros(2500) {
            thread::sleep(Duration::from_micros(2500) - elapsed);
        }
    
        _loop_count += 1;
    }
                     
}


```

---

## **Features**
- **Portability**: 
  - Built on `embedded-hal`, the driver works with any platform that supports the standard HAL traits including other Linux platforms (Nvidia Jetson series computers) or compatible microcontrollers (STM32, Arduino, etc.)
- **Thread-Safe**:
  - Shared SPI access is protected using `Arc<Mutex<SpiCore>>`. SPI communication is not inherently threadsafe so this driver utilizes Rust's borrow checker to guarantee only 1 abstraction accesses the SPI bus at a time.
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
