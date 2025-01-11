use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::Imu as ImuMsg;
use icm_20948_driver::imu::{Accelerometer, Gyroscope, IMU};
use icm_20948_driver::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags};
use linux_embedded_hal::SpidevBus;
use std::sync::{Arc, Mutex};
use std::time::Duration;


/// Struct containing the ROS2 node, publisher, and IMU components
struct IMUPublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<ImuMsg>>,
    imu: IMU<SpidevBus>,
    accel: Accelerometer<SpidevBus>,
    gyro: Gyroscope<SpidevBus>,
}

impl IMUPublisherNode {
    /// Create a new IMU Publisher Node
    fn new(context: &Context) -> Result<Self, RclrsError> {
        // Create the ROS2 node and publisher
        let node = create_node(context, "imu_publisher").unwrap();
        let publisher = node
            .create_publisher::<ImuMsg>("/raw_imu", QOS_PROFILE_DEFAULT)
            .unwrap();

        // Configure the SPI device
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

        // Create shared SPI bus and IMU components
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
        })
    }

    /// Initialize the IMU explicitly
    fn initialize_imu(&mut self) -> Result<(), String> {
        println!("Initializing IMU...");
        self.imu.initialize().map_err(|e| format!("IMU initialization failed: {:?}", e))?;
        println!("IMU successfully initialized.");
        Ok(())
    }

    /// Publish IMU data to the ROS2 topic
    fn publish_data(&mut self) -> Result<(), RclrsError> {
        println!("publish_data: Starting");

        let mut imu_msg = ImuMsg::default();

        // Read accelerometer data
        println!("publish_data: Reading accelerometer");
        if let Ok(accel_data) = self.accel.read() {
            imu_msg.linear_acceleration.x = accel_data[0] as f64;
            imu_msg.linear_acceleration.y = accel_data[1] as f64;
            imu_msg.linear_acceleration.z = accel_data[2] as f64;
            println!(
                "publish_data: Accel -> x: {}, y: {}, z: {}",
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            );
        } else {
            println!("publish_data: Failed to read accelerometer");
        }

        // Read gyroscope data
        println!("publish_data: Reading gyroscope");
        if let Ok(gyro_data) = self.gyro.read() {
            imu_msg.angular_velocity.x = gyro_data[0] as f64;
            imu_msg.angular_velocity.y = gyro_data[1] as f64;
            imu_msg.angular_velocity.z = gyro_data[2] as f64;
            println!(
                "publish_data: Gyro -> x: {}, y: {}, z: {}",
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            );
        } else {
            println!("publish_data: Failed to read gyroscope");
        }

        // Publish the message
        println!("publish_data: Publishing message");
        self.publisher.publish(imu_msg).unwrap();

        println!("publish_data: Completed");
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting IMU Publisher Node...");

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
        println!("IMU Publisher thread running...");
        loop {
            std::thread::sleep(Duration::from_millis(10)); // Approx. 100 Hz
            println!("Attempting to lock publisher node...");
            if let Ok(mut node) = publisher_node_thread.lock() {
                println!("Lock acquired. Calling publish_data...");
                if let Err(err) = node.publish_data() {
                    eprintln!("Error publishing IMU data: {:?}", err);
                }
                println!("After calling publish_data");
            } else {
                eprintln!("Failed to lock publisher node.");
            }
        }
    });

    // Spin the node to process callbacks
    println!("Spinning ROS2 node...");
    rclrs::spin(node_handle)?; // Only spin the node, without locking the publisher logic

    Ok(())
}
