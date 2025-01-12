use crate::spi_core::SpiCore;
use std::sync::{Arc, Mutex};
use embedded_hal::spi::SpiBus;

pub mod accelerometer;
pub mod gyroscope;

pub use accelerometer::Accelerometer;
pub use gyroscope::Gyroscope;

// General Configuration
const REG_I2C_MST_CTRL: u8 = 0x03;
const REG_PWR_MGMT_1: u8 = 0x06;
const REG_PWR_MGMT_2: u8 = 0x07;

// Gyroscope Configuration
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_GYRO_DLPF_CONFIG: u8 = 0x1A;
const REG_GYRO_SMPLRT_DIV: u8 = 0x19;

// Accelerometer Configuration
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_DLPF_CONFIG: u8 = 0x1D;
const REG_ACCEL_SMPLRT_DIV: u8 = 0x1A; // Shared with gyro
const USER_BANK_SELECT: u8 = 0x7F;     // User Bank Selection Register
const USER_BANK_0: u8 = 0x00;          // User Bank 0
const USER_BANK_2: u8 = 0x20;          // User Bank 2


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
    
        // Enable I2C Master and disable primary I2C
        spi_core.write_register(REG_I2C_MST_CTRL, 0x30)?;
    
        // Step 1: Power Management
        // Use PLL as clock source
        spi_core.write_register(REG_PWR_MGMT_1, 0x01)?; // Set PWR_MGMT_1 to use PLL
        // Enable accelerometer and gyroscope
        spi_core.write_register(REG_PWR_MGMT_2, 0x00)?; // Set PWR_MGMT_2 to enable all sensors
    
        // Step 2: Switch to User Bank 2 for configuration
        spi_core.write_register(USER_BANK_SELECT, USER_BANK_2)?;
    
        // Step 3: Configure low-pass filters and sampling rates
        // Gyroscope configuration
        spi_core.write_register(REG_GYRO_CONFIG, 0x00)?; // +/- 250dps        
        spi_core.write_register(REG_GYRO_DLPF_CONFIG, 0x07)?; // GYRO_FCHOICE = 1, GYRO_DLPFCFG = 7
        spi_core.write_register(REG_GYRO_SMPLRT_DIV, 0x02)?;  // Set gyro sampling rate divider for ~400 Hz
    
        // Accelerometer configuration
        spi_core.write_register(REG_ACCEL_CONFIG, 0x00)?;     // Set ACCEL_FS_SEL to +/- 2g
        spi_core.write_register(REG_ACCEL_DLPF_CONFIG, 0x07)?; // ACCEL_FCHOICE = 1, ACCEL_DLPFCFG = 7
        spi_core.write_register(REG_ACCEL_SMPLRT_DIV, 0x02)?;  // Set accel sampling rate divider for ~400 Hz
    
        // Step 4: Switch back to User Bank 0
        spi_core.write_register(USER_BANK_SELECT, USER_BANK_0)?;
    
        // Step 5: Verify power settings
        let power_status = spi_core.read_register(REG_PWR_MGMT_1)?;
        if power_status != 0x01 {
            println!("Warning: PWR_MGMT_1 is not set correctly (0x{:X})", power_status);
        }
    
        Ok(())
    }
}    
