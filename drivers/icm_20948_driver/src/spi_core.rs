use embedded_hal::spi::SpiBus;


/// Core SPI struct
pub struct SpiCore<SPI>
where
    SPI: SpiBus,
{
    spi: SPI,
}

impl<SPI> SpiCore<SPI>
where
    SPI: SpiBus,
{
    /// Creates a new SPI core instance
    pub fn new(spi: SPI) -> Self {
        SpiCore { spi }
    }

    /// Writes a value to a register
    pub fn write_register(&mut self, reg: u8, value: u8) -> Result<(), SPI::Error> {
        let tx_buf = [reg & 0x7F, value]; // MSB clear for write mode
        self.spi.write(&tx_buf)
    }

    /// Reads a single register and returns its value
    pub fn read_register(&mut self, reg: u8) -> Result<u8, SPI::Error> {
        let mut rx_buf = [0, 0]; // Receive buffer
        let tx_buf = [reg | 0x80, 0x00]; // MSB set for read mode + dummy byte
        self.spi.transfer(&mut rx_buf, &tx_buf)?;
        Ok(rx_buf[1]) // The second byte contains the register value
    }

    /// Reads two consecutive registers and combines them into a signed 16-bit value
    pub fn read_16bit_register(&mut self, high_reg: u8, low_reg: u8) -> Result<i16, SPI::Error> {
        let high = self.read_register(high_reg)? as i16;
        let low = self.read_register(low_reg)? as i16;
        Ok((high << 8) | low) // Combine high and low bytes
    }

    /// Performs a burst read to retrieve multiple consecutive register values
    pub fn burst_read(&mut self, start_reg: u8, buffer: &mut [u8]) -> Result<(), SPI::Error> {
        let mut tx_buf = vec![0; buffer.len() + 1];
        tx_buf[0] = start_reg | 0x80; // MSB set for read mode
        let mut rx_buf = vec![0; buffer.len() + 1];

        self.spi.transfer(&mut rx_buf, &tx_buf)?;
        buffer.copy_from_slice(&rx_buf[1..]); // Copy response to the buffer (skip dummy byte)

        Ok(())
    }


    /// Performs a burst read and combined high and low bits
    pub fn burst_read_combine(&mut self, start_reg: u8) -> Result<[i16; 3], SPI::Error> {
        let mut buffer = [0u8; 6];
        self.burst_read(start_reg, &mut buffer)?;

        // Combine high and low bytes for x, y, and z
        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok([x, y, z])
    }

}
