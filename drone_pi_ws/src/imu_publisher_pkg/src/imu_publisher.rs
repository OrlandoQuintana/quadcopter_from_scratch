use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::Imu as ImuMsg;
use icm20948_driver_rust::imu::{Accelerometer, Gyroscope, IMU};
use icm20948_driver_rust::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags};
use linux_embedded_hal::SpidevBus;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use biquad::{Biquad, Coefficients, DirectForm1, ToHertz};



/// Struct containing the ROS2 node, publisher, and IMU components
struct IMUPublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<ImuMsg>>,
    imu: IMU<SpidevBus>,
    accel: Accelerometer<SpidevBus>,
    gyro: Gyroscope<SpidevBus>,
    filter_x: Mutex<DirectForm1<f32>>,
    filter_y: Mutex<DirectForm1<f32>>,
    filter_z: Mutex<DirectForm1<f32>>,
}

impl IMUPublisherNode {
    /// Create a new IMU Publisher Node
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "imu_publisher").unwrap();
        let publisher = node
            .create_publisher::<ImuMsg>("/raw_imu", QOS_PROFILE_DEFAULT)
            .unwrap();

        // Configure Butterworth filter coefficients
        let coeffs = Coefficients::<f32>::from_params(
            biquad::Type::LowPass,
            500.0.hz(), // Sampling frequency (adjust as per your IMU's rate)
            2.0.hz(),   // Cutoff frequency
            0.707,      // Q factor (Butterworth characteristic)
        )
        .unwrap();

        // Create filters for each axis
        let filter_x = Mutex::new(DirectForm1::<f32>::new(coeffs));
        let filter_y = Mutex::new(DirectForm1::<f32>::new(coeffs));
        let filter_z = Mutex::new(DirectForm1::<f32>::new(coeffs));

        // Configure IMU
        let mut spidev = Spidev::open("/dev/spidev0.0").expect("Failed to open SPI device");
        spidev
            .configure(
                &SpidevOptions::new()
                    .bits_per_word(8)
                    .max_speed_hz(1_000_000)
                    .mode(SpiModeFlags::SPI_MODE_0)
                    .build(),
            )
            .expect("Failed to configure SPI device");

        let spidev_bus = SpidevBus::from(linux_embedded_hal::SpidevBus(spidev));
        let spi = Arc::new(Mutex::new(SpiCore::new(spidev_bus)));
        let imu = IMU::new(Arc::clone(&spi));
        let accel = Accelerometer::new(Arc::clone(&spi));
        let gyro = Gyroscope::new(Arc::clone(&spi));

        Ok(Self {
            node: Arc::clone(&node),
            publisher: Arc::clone(&publisher),
            imu,
            accel,
            gyro,
            filter_x,
            filter_y,
            filter_z,
        })
    }

    /// Initialize the IMU explicitly
    fn initialize_imu(&mut self) -> Result<(), String> {
        self.imu.initialize().map_err(|e| format!("IMU initialization failed: {:?}", e))?;
        Ok(())
    }

    /// Publish IMU data to the ROS2 topic
    fn publish_data(&mut self) -> Result<(), RclrsError> {
        let mut imu_msg = ImuMsg::default();

        // Read accelerometer data
        if let Ok(accel_data) = self.accel.read() {
            // Filter accelerometer data
            let filtered_x = self.filter_x.lock().unwrap().run(accel_data[0] as f32);
            let filtered_y = self.filter_y.lock().unwrap().run(accel_data[1] as f32);
            let filtered_z = self.filter_z.lock().unwrap().run(accel_data[2] as f32);

            imu_msg.linear_acceleration.x = filtered_x as f64;
            imu_msg.linear_acceleration.y = filtered_y as f64;
            imu_msg.linear_acceleration.z = filtered_z as f64;

            println!(
                "Accel -> x: {:.3}, y: {:.3}, z: {:.3}",
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            );
        } else {
            println!("publish_data: Failed to read accelerometer");
        }

        // Read gyroscope data
        if let Ok(gyro_data) = self.gyro.read() {

            // Gyroscope Calibration
            // Run the gyroscope for X iterations with the gyroscope completely at rest. take the average
            // reading and subtract from all gyroscope readings to get a calibrated reading
            imu_msg.angular_velocity.x = (gyro_data[0] - (-0.00125)) as f64; // X Bias
            imu_msg.angular_velocity.y = (gyro_data[1] - (0.013)) as f64; // Y Bias
            imu_msg.angular_velocity.z = (gyro_data[2] - (0.006)) as f64; // Z Bias

            println!(
                "Gyro -> x: {:.3}, y: {:.3}, z: {:.3}",
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            );
        } else {
            println!("publish_data: Failed to read gyroscope");
        }

        // Publish the message
        self.publisher.publish(imu_msg).unwrap();

        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::new(std::env::args()).unwrap();
    let mut publisher_node = IMUPublisherNode::new(&context).unwrap();

    // Initialize the IMU explicitly
    if let Err(err) = publisher_node.initialize_imu() {
        eprintln!("Failed to initialize IMU: {}", err);
        return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, err)));
    }

    let node_handle = publisher_node.node.clone(); // Clone the ROS2 node for spinning
    let publisher_node = Arc::new(Mutex::new(publisher_node));
    let publisher_node_thread = Arc::clone(&publisher_node);

    // Spawn a thread for publishing data
    std::thread::spawn(move || {
        let mut last_time = std::time::Instant::now();
        loop {
            // Calculate elapsed time since the last loop
            let elapsed = last_time.elapsed();
            let elapsed_ms = elapsed.as_millis() as u64;

            // Check if we've exceeded 2 ms
            if elapsed_ms >= 2 {
                // Update the timestamp for the next cycle
                last_time = std::time::Instant::now();

                // Call the publish data method
                if let Ok(mut node) = publisher_node_thread.lock() {
                    if let Err(err) = node.publish_data() {
                        eprintln!("Error publishing IMU data: {:?}", err);
                    }
                } else {
                    eprintln!("Failed to lock publisher node.");
                }
            } else {
                // Sleep for the remaining time in the 2 ms window
                std::thread::sleep(Duration::from_micros((2000 - elapsed.as_micros() as u64) as u64));
            }
        }
    });

    // Spin the node to process callbacks
    rclrs::spin(node_handle)?; // Only spin the node, without locking the publisher logic

    Ok(())
}
