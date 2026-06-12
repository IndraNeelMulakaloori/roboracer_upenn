import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from ackermann_msgs.msg import AckermannDriveStamped


# The first node will be named talker.cpp or talker.py and needs to meet these criteria:

# talker listens to two ROS parameters v and d.
# talker publishes an AckermannDriveStamped message with the speed field equal to the v parameter and steering_angle field equal to the d parameter, and to a topic named drive.
# talker publishes as fast as possible.
# To test node, set the two ROS parameters through command line, a launch file, or a yaml file.



class TalkerPublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        ## talker listens to two ROS parameters v and d
        self.declare_parameters(
            namespace='',
            parameters=[
                ('v',0.0),
                ('d', 0.0)
            ]
            )

        self.talker_publisher = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic='/drive',
            qos_profile = 10, 
        )
        self.create_timer(
            timer_period_sec= 0.1,
            callback=self.timer_callback,
             
        )

        self.get_logger().info(f"{node_name} initalised.....")

    def timer_callback(self):
        speed = self.get_parameter('v').value
        steering = self.get_parameter('d').value

        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        
        self.get_logger().info(f"Speed : {speed}, Steering_angle : {steering}")

        self.talker_publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    talker_publisher = TalkerPublisher('talker_publisher')

    rclpy.spin(talker_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

