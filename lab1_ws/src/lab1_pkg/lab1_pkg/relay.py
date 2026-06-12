import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from ackermann_msgs.msg import AckermannDriveStamped


# The first node will be named talker.cpp or talker.py and needs to meet these criteria:

# talker listens to two ROS parameters v and d.
# talker publishes an AckermannDriveStamped message with the speed field equal to the v parameter and steering_angle field equal to the d parameter, and to a topic named drive.
# talker publishes as fast as possible.
# To test node, set the two ROS parameters through command line, a launch file, or a yaml file.



class RelaySubscriber(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.multiplier = 3.0
        self.relay_subscription = self.create_subscription(
            msg_type=AckermannDriveStamped,
            topic='/drive',
            qos_profile=10,
            callback=self.subscriber_callback
        )

        self.relay_publisher = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic='/drive_relay',
            qos_profile=10
        )
        self.get_logger().info(f"{node_name} initalised.....")

    def subscriber_callback(self,pub_msg):
        # self.get_logger().info(f"{msg} ")
        speed = pub_msg.drive.speed 
        steering_angle = pub_msg.drive.steering_angle 
        
        sub_msg = AckermannDriveStamped()
        sub_msg.drive.speed = speed * self.multiplier
        sub_msg.drive.steering_angle = steering_angle * self.multiplier
        self.get_logger().info(f"Speed : {speed}, Steering_angle : {steering_angle}")
        self.get_logger().info(f"Speed : {sub_msg.drive.speed}, Steering_angle : {sub_msg.drive.steering_angle}")
        self.relay_publisher.publish(sub_msg)

        
        
        



def main(args=None):
    rclpy.init(args=args)

    relay_subscriber = RelaySubscriber('relay_subscriber')

    rclpy.spin(relay_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

