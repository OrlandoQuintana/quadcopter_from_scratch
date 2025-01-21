use rclrs::{create_node, Context, Node, RclrsError, Subscription, Publisher, QOS_PROFILE_DEFAULT};
use rust_ekf::EKF;
use sensor_msgs::msg::Imu;
use geometry_msgs::msg::{Vector3, Quaternion};
use std::{
    env,
    sync::{Arc, Mutex, Condvar},
    thread,
    time::Instant,
};

pub struct QuaternionPublisherNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<Imu>>,
    _publisher: Arc<Publisher<Imu>>,
    data: Arc<Mutex<Option<Imu>>>,
    ekf: Mutex<Option<EKF>>, // Add EKF instance as an option type
    last_update_time: Mutex<Option<Instant>>, // Track time of last callback
    trigger: Arc<(Mutex<bool>, Condvar)>, // Trigger for new data
}

impl QuaternionPublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "quaternion_publisher").unwrap();

        let data: Arc<Mutex<Option<Imu>>> = Arc::new(Mutex::new(None));
        let data_mut = Arc::clone(&data);

        let trigger = Arc::new((Mutex::new(false), Condvar::new()));
        let trigger_clone = Arc::clone(&trigger);

        let _subscriber = node.create_subscription::<Imu, _>(
            "/raw_imu", // Subscribes to raw IMU data
            QOS_PROFILE_DEFAULT,
            move |msg: Imu| {
                // Store incoming message
                *data_mut.lock().unwrap() = Some(msg);

                // Notify the waiting thread
                let (lock, cvar) = &*trigger_clone;
                let mut triggered = lock.lock().unwrap();
                *triggered = true;
                cvar.notify_one();
            },
        )?;

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
            ekf: Mutex::new(None), // Initialize EKF as None type
            last_update_time: Mutex::new(None), // Start without timing data
            trigger,
        })
    }

    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            let accel_data = [
                data.linear_acceleration.x as f64,
                data.linear_acceleration.y as f64,
                data.linear_acceleration.z as f64,
            ];
        
            // Calculate `dt` dynamically
            let mut last_update_time = self.last_update_time.lock().unwrap();
            let dt = if let Some(last_time) = *last_update_time {
                // Compute the time since the last callback
                let elapsed = last_time.elapsed();
                elapsed.as_secs_f64() // Convert to seconds as f64
            } else {
                // Use a default `dt` for the first iteration
                0.002 // Assume 2 ms (500 Hz)
            };
            *last_update_time = Some(Instant::now()); // Update the last callback time


            // Lock the EKF once
            let mut ekf_lock = self.ekf.lock().unwrap();
        
            // EKF struct was initialized with none type accel data. We essentially re-initialize it here with the current accel data.
            ekf_lock.get_or_insert_with(|| {
                println!("EKF initialized with initial accelerometer data");
                EKF::new(Some(accel_data))
            });
        
            // Access the EKF and perform predict/update
            if let Some(ekf) = ekf_lock.as_mut() {
                let gyro_data = [
                    data.angular_velocity.x as f64,
                    data.angular_velocity.y as f64,
                    data.angular_velocity.z as f64,
                ];

                ekf.predict(gyro_data, dt); // Pass raw gyro data and dynamically calculated timestep dt to the ekf's predict method
                ekf.update(accel_data); // Pass raw accelerometer data to the ekf's update method

                // Get updated quaternion from EKF state
                let state = ekf.get_state();
                let quaternion = Quaternion {
                    w: state[0], // q0
                    x: state[1], // q1
                    y: state[2], // q2
                    z: state[3], // q3
                };

                let gyro_data_ekf = Vector3 {
                    x: state[4], // ekf filtered gyro x data
                    y: state[5], // ekf filtered gyro y data
                    z: state[6], // ekf filtered gyro z data
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
                    angular_velocity: gyro_data_ekf.clone(), // Gyro data post-EKF
                    angular_velocity_covariance: [0.0; 9], // Optional: fill with real data
                    linear_acceleration: data.linear_acceleration.clone(), // Biquad-filtered accelerometer data
                    linear_acceleration_covariance: [0.0; 9], // Optional: fill with real data
                };

                // Publish the message
                self._publisher.publish(&imu_msg)?;

                println!("Published Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
            }
        }
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args())?;

    let quaternion_publisher_node = Arc::new(QuaternionPublisherNode::new(&context)?);
    let trigger_clone = Arc::clone(&quaternion_publisher_node.trigger);

    // Spawn a thread to handle the data_callback
    let quaternion_publisher_node_thread = Arc::clone(&quaternion_publisher_node);
    thread::spawn(move || {
        loop {
            // Wait for a trigger
            let (lock, cvar) = &*trigger_clone;
            let mut triggered = lock.lock().unwrap();
            while !*triggered {
                triggered = cvar.wait(triggered).unwrap();
            }
            *triggered = false; // Reset the trigger

            // Call the data callback
            if let Err(e) = quaternion_publisher_node_thread.data_callback() {
                eprintln!("Error in data callback: {:?}", e);
            }
        }
    });

    // Spin the node
    rclrs::spin(quaternion_publisher_node.node.clone())
}