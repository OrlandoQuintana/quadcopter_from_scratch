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

