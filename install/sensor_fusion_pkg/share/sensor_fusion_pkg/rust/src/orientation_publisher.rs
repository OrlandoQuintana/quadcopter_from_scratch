use rclrs::{create_node, Context, Node, RclrsError, Subscription, Publisher, QOS_PROFILE_DEFAULT};
use rust_ekf::EKFEuler;
use sensor_msgs::msg::Imu;
use geometry_msgs::msg::Quaternion;
use std::{
    env,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

pub struct OrientationPublisherNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<Imu>>,
    _publisher: Arc<Publisher<Imu>>, // Change to publish Quaternion
    data: Arc<Mutex<Option<Imu>>>,
    ekf: Mutex<EKFEuler>, // Add EKFEuler instance
}

impl OrientationPublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "orientation_publisher").unwrap();

        let data: Arc<Mutex<Option<Imu>>> = Arc::new(Mutex::new(None));
        let data_mut = Arc::clone(&data);

        let _subscriber = node
            .create_subscription::<Imu, _>(
                "/raw_imu",
                QOS_PROFILE_DEFAULT,
                move |msg: Imu| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();

        let _publisher = node
            .create_publisher::<Imu>(
                "/estimated_orientation",
                QOS_PROFILE_DEFAULT,
            )
            .unwrap();

        Ok(Self {
            node,
            _subscriber,
            _publisher,
            data,
            ekf: Mutex::new(EKFEuler::new()), // Initialize EKFEuler
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

            // Lock and update EKF
            let mut ekf = self.ekf.lock().unwrap();
            ekf.predict(gyro_data);
            ekf.update(accel_data);

            // Get updated state
            let state = ekf.get_state();
            let roll = state[0]; // Roll angle
            let pitch = state[1]; // Pitch angle
            let yaw = state[2]; //yaw angle

            // Convert roll and pitch to quaternion
            let quaternion = self.euler_to_quaternion(roll, pitch, yaw);


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
                orientation: quaternion,
                orientation_covariance: [0.0; 9], // Set to zeros or a meaningful covariance if available
                angular_velocity: data.angular_velocity.clone(),
                angular_velocity_covariance: [0.0; 9], // Optional: fill with real data
                linear_acceleration: data.linear_acceleration.clone(),
                linear_acceleration_covariance: [0.0; 9], // Optional: fill with real data
            };


            // Publish quaternion
            self._publisher.publish(&imu_msg)?;

            println!("Roll angle: {:.3}, Pitch angle: {:.3}, Yaw angle: {:.3}", roll, pitch, yaw);
        }
        Ok(())
    }

    fn euler_to_quaternion(&self, roll: f64, pitch: f64, yaw: f64) -> Quaternion {
        // Compute trigonometric terms
        let cy = (yaw / 2.0).cos();  // Cosine of half yaw
        let sy = (yaw / 2.0).sin();  // Sine of half yaw
        let cr = (roll / 2.0).cos(); // Cosine of half roll
        let sr = (roll / 2.0).sin(); // Sine of half roll
        let cp = (pitch / 2.0).cos(); // Cosine of half pitch
        let sp = (pitch / 2.0).sin(); // Sine of half pitch
    
        // Compute quaternion components
        Quaternion {
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            w: cr * cp * cy + sr * sp * sy,
        }
    }
    
}

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();

    let orientation_publisher_node = Arc::new(OrientationPublisherNode::new(&context).unwrap());
    let orientation_publisher_node_other_thread = Arc::clone(&orientation_publisher_node);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(5)); // 100 Hz loop
        orientation_publisher_node_other_thread.data_callback().unwrap();
    });

    rclrs::spin(orientation_publisher_node.node.clone())
}
