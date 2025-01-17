use rclrs::{create_node, Context, Node, RclrsError, Subscription, Publisher, QOS_PROFILE_DEFAULT};
use rust_ekf::EKF;
use sensor_msgs::msg::Imu;
use std::{
    env,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

pub struct QuaternionPublisherNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<Imu>>,
    _publisher: Arc<Publisher<Imu>>,
    data: Arc<Mutex<Option<Imu>>>,
    ekf: Mutex<EKF>, // Add EKF instance
}

impl QuaternionPublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "quaternion_publisher").unwrap();

        let data: Arc<Mutex<Option<Imu>>> = Arc::new(Mutex::new(None));
        let data_mut = Arc::clone(&data);

        let _subscriber = node
            .create_subscription::<Imu, _>(
                "/raw_imu", // Subscribes to raw IMU data
                QOS_PROFILE_DEFAULT,
                move |msg: Imu| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();

        let _publisher = node
            .create_publisher::<Imu>(
                "/quaternion_estimate", // Publishes quaternion estimates
                QOS_PROFILE_DEFAULT,
            )
            .unwrap();

        Ok(Self {
            node,
            _subscriber,
            _publisher,
            data,
            ekf: Mutex::new(EKF::new()), // Initialize EKF
        })
    }

    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            // Extract IMU data
            let gyro_data = [
                data.angular_velocity.x as f64,
                data.angular_velocity.y as f64,
                data.angular_velocity.z as f64,
            ];
            let accel_data = [
                data.linear_acceleration.x as f64,
                data.linear_acceleration.y as f64,
                data.linear_acceleration.z as f64,
            ];

            let gyro_bias_x = -0.00125;       // X bias (approximately zero)
            let gyro_bias_y = 0.013;     // Y bias (calculated)
            let gyro_bias_z = 0.006;     // Z bias (calculated)
            
            // Subtract biases from raw gyroscope readings
            let corrected_gyro_x = gyro_data[0] - gyro_bias_x;
            let corrected_gyro_y = gyro_data[1] - gyro_bias_y;
            let corrected_gyro_z = gyro_data[2] - gyro_bias_z;
            

            // Lock and update EKF
            let mut ekf = self.ekf.lock().unwrap();
            ekf.predict([corrected_gyro_x as f64, corrected_gyro_y as f64, corrected_gyro_z as f64]);
            ekf.update(accel_data);

            // Get updated quaternion from EKF state
            let state = ekf.get_state();
            let quaternion = geometry_msgs::msg::Quaternion {
                w: state[0], // q0
                x: state[1], // q1
                y: state[2], // q2
                z: state[3], // q3
            };

            // Create an Imu message
            let imu_msg = Imu {
                header: std_msgs::msg::Header {
                    frame_id: "imu_link".to_string(), // Set the appropriate frame ID
                    stamp: builtin_interfaces::msg::Time {
                        sec: 0,      // Default to zero
                        nanosec: 0,  // Default to zero
                    },                    
                    ..Default::default()
                },
                orientation: quaternion.clone(), // Publish quaternion
                orientation_covariance: [0.0; 9], // Set to zeros or a meaningful covariance if available
                angular_velocity: data.angular_velocity.clone(),
                angular_velocity_covariance: [0.0; 9], // Optional: fill with real data
                linear_acceleration: data.linear_acceleration.clone(),
                linear_acceleration_covariance: [0.0; 9], // Optional: fill with real data
            };

            // Publish the message
            self._publisher.publish(&imu_msg)?;

            println!("gyro x: {}, gyro y {}: gyro z {}:", corrected_gyro_x as f64, corrected_gyro_y as f64, corrected_gyro_z as f64);
            //println!("Published Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        }
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();

    let quaternion_publisher_node = Arc::new(QuaternionPublisherNode::new(&context).unwrap());
    let quaternion_publisher_node_other_thread = Arc::clone(&quaternion_publisher_node);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(5)); // 100 Hz loop
        quaternion_publisher_node_other_thread.data_callback().unwrap();
    });

    rclrs::spin(quaternion_publisher_node.node.clone())
}
