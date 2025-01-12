use rclrs::{create_node, Context, Node, RclrsError, Subscription, Publisher, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::Imu;
use std::{
    env,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

pub struct OrientationPublisherNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<Imu>>,
    _publisher: Arc<Publisher<Imu>>,
    data: Arc<Mutex<Option<Imu>>>,
}

impl OrientationPublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        // Core ROS2 node
        let node = create_node(context, "orientation_publisher").unwrap();

        // Shared data for storing IMU messages
        let data: Arc<Mutex<Option<Imu>>> = Arc::new(Mutex::new(None));
        let data_mut = Arc::clone(&data);

        // Subscriber setup
        let _subscriber = node
            .create_subscription::<Imu, _>(
                "/raw_imu", // Topic name to subscribe to
                QOS_PROFILE_DEFAULT,
                move |msg: Imu| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();

        // Publisher setup
        let _publisher = node
            .create_publisher::<Imu>(
                "/estimated_orientation", // Topic name to publish to
                QOS_PROFILE_DEFAULT,
            )
            .unwrap();

        

        // Return the initialized struct
        Ok(Self {
            node,
            _subscriber,
            _publisher,
            data,
        })
    }

    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            println!("Accel X: {}", data.linear_acceleration.x as f32);
            println!("Accel Y: {}", data.linear_acceleration.y as f32);
            println!("Accel Z:{}", data.linear_acceleration.z as f32);
            println!("Gyro X: {}", data.angular_velocity.x as f32);
            println!("Gyro Y: {}", data.angular_velocity.y as f32);
            println!("Gyro Z: {}", data.angular_velocity.z as f32);
        } else {
            println!("No message available yet.");
        }
        Ok(())
    } 


}

fn main() -> Result<(), RclrsError> {
    // ROS2 context initialization
    let context = Context::new(env::args()).unwrap();

    // Create the filtered IMU node
    let orientation_publisher_node = Arc::new(OrientationPublisherNode::new(&context).unwrap());
    let orientation_publisher_node_other_thread = Arc::clone(&orientation_publisher_node);

    // Spawn a thread to process and publish data periodically
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(10)); // 100 Hz loop
        orientation_publisher_node_other_thread.data_callback().unwrap();
        println!("test");
    });

    // Spin the node
    rclrs::spin(orientation_publisher_node.node.clone())
}
