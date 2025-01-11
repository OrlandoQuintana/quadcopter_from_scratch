/// Creates a SimplePublisherNode, initializes a node and publisher, and provides
/// methods to publish a simple "Hello World" message on a loop in separate threads.
use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std::{env, sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;
/// SimplePublisherNode struct contains node and publisher members.
/// Used to initialize a ROS 2 node and publisher, and publish messages.

//use sensor_msgs::msg::Imu;


struct SimplePublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<StringMsg>>,
}
impl SimplePublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, publisher })
    }
    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self.publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: i32 = 0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        count = publisher_other_thread.publish_data(count).unwrap();
    });
    rclrs::spin(publisher.node.clone())
}