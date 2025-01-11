use crate::spi_core::SpiCore;
use std::sync::{Arc, Mutex};
use embedded_hal::spi::SpiBus;

pub mod accelerometer;
pub mod gyroscope;

pub use accelerometer::Accelerometer;
pub use gyroscope::Gyroscope;

/// IMU Register Addresses
const I2C_MST_CTRL: u8 = 0x03;
const PWR_MGMT_1: u8 = 0x06;
const PWR_MGMT_2: u8 = 0x07;
const GYRO_DLPF_CONFIG: u8 = 0x1A;
const ACCEL_SENSITIVITY_CONFIG: u8 = 0x1C;
const ACCEL_DLPF_CONFIG: u8 = 0x1D;
const GYRO_SENSITIVITY_CONFIG: u8 = 0x1B;

/// Central IMU struct to handle overall initialization
pub struct IMU<SPI>
where
    SPI: SpiBus,
{
    spi_core: Arc<Mutex<SpiCore<SPI>>>,
}

impl<SPI> IMU<SPI>
where
    SPI: SpiBus,
{
    /// Create a new IMU instance
    pub fn new(spi_core: Arc<Mutex<SpiCore<SPI>>>) -> Self {
        Self { spi_core }
    }

    /// Initialize the IMU (accelerometer, gyroscope, etc.)
    pub fn initialize(&mut self) -> Result<(), SPI::Error> {
        // Lock the SPI core
        let mut spi_core = self.spi_core.lock().unwrap();

        spi_core.write_register(I2C_MST_CTRL, 0x30)?; // Enable I2C Master and disable primary I2C

        // Step 1: Power Management
        // Use PLL as clock source
        spi_core.write_register(PWR_MGMT_1, 0x01)?; // Set PWR_MGMT_1
        // Enable accel and gyro
        spi_core.write_register(PWR_MGMT_2, 0x00)?; // Set PWR_MGMT_2

        // Step 2: Configure low-pass filters
        // Set digital filter for gyro
        spi_core.write_register(GYRO_DLPF_CONFIG, 0x03)?;
        // Set digital filter for accelerometer
        spi_core.write_register(ACCEL_DLPF_CONFIG, 0x03)?;

        // Initializes the accelerometer (configure sensitivity, enable axes, etc.)
        spi_core.write_register(ACCEL_SENSITIVITY_CONFIG, 0x00)?; // Set ACCEL_CONFIG to +/- 2g

        // Initializes the gyroscope (configure sensitivity, enable axes, etc.)
        spi_core.write_register(GYRO_SENSITIVITY_CONFIG, 0x00)?; // Set GYRO_CONFIG to +/- 250 dps

        // Step 4: Verify power settings
        let power_status = spi_core.read_register(PWR_MGMT_1)?;
        if power_status != 0x01 {
            println!("Warning: PWR_MGMT_1 is not set correctly (0x{:X})", power_status);
        }

        Ok(())
    }
}
