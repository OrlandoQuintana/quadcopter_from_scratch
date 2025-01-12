use crate::spi_core::SpiCore;
use std::sync::{Arc, Mutex};
use embedded_hal::spi::SpiBus;


/// Accelerometer struct, generic over SPI type
pub struct Accelerometer<SPI>
where
    SPI: SpiBus,
{
    spi_core: Arc<Mutex<SpiCore<SPI>>>,
}

impl<SPI> Accelerometer<SPI>
where
    SPI: SpiBus,
{
    /// Creates a new Accelerometer instance
    pub fn new(spi_core: Arc<Mutex<SpiCore<SPI>>>) -> Self {
        Self { spi_core }
    }
    
    pub fn read(&mut self) -> Result<[f32; 3], SPI::Error> {
        let mut spi_core = self.spi_core.lock().unwrap();
        let raw_data = spi_core.burst_read_combine(0x2D)?; // ACCEL_XOUT_H is at 0x2D

        const ACCEL_SENSITIVITY: f32 = 16384.0; // For +/- 2g sensitivity
        const GRAVITY: f32 = 9.8; // Acceleration due to gravity in m/s²

        //ICM-20948's x/y/z orientation needs to be modified to match the orientation that is conventionally
        //expected in Extended Kalman Filters. (EKF) Swap x and y, make z negative
        Ok([
            (raw_data[1] as f32 / ACCEL_SENSITIVITY) * GRAVITY,
            (raw_data[0] as f32 / ACCEL_SENSITIVITY) * GRAVITY,
            -(raw_data[2] as f32 / ACCEL_SENSITIVITY) * GRAVITY,
        ])
    }
    
}
