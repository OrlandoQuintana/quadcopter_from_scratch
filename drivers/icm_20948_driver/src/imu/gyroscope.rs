use crate::spi_core::SpiCore;
use std::sync::{Arc, Mutex};
use embedded_hal::spi::SpiBus;

/// Gyroscope struct, generic over SPI type
pub struct Gyroscope<SPI>
where
    SPI: SpiBus,
{
    spi_core: Arc<Mutex<SpiCore<SPI>>>,
}

impl<SPI> Gyroscope<SPI>
where
    SPI: SpiBus,
{
    /// Creates a new Gyroscope instance
    pub fn new(spi_core: Arc<Mutex<SpiCore<SPI>>>) -> Self {
        Self { spi_core }
    }

    pub fn read(&mut self) -> Result<[f32; 3], SPI::Error> {
        let mut spi_core = self.spi_core.lock().unwrap();
        let raw_data = spi_core.burst_read_combine(0x33)?; // GYRO_XOUT_H is at 0x33

        const GYRO_SENSITIVITY: f32 = 131.0; // For +/- 250 dps sensitivity
        const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0; // Conversion factor from degrees to radians

        //ICM-20948's x/y/z orientation needs to be modified to match the orientation that is conventionally
        //expected in Extended Kalman Filters. (EKF) Swap x and y, make z negative
        Ok([
            (raw_data[1] as f32 / GYRO_SENSITIVITY) * DEG_TO_RAD,
            (raw_data[0] as f32 / GYRO_SENSITIVITY) * DEG_TO_RAD,
            -(raw_data[2] as f32 / GYRO_SENSITIVITY) * DEG_TO_RAD,
        ])
    }
}
