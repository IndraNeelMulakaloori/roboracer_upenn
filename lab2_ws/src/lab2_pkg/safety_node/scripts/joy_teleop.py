#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        
        # Subscribe to the raw joystick USB data
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publish to the car's motor controller
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Physical limits of the F1Tenth Car
        self.max_speed = 4.0        # meters per second
        self.max_steering = 0.25    # radians (~20 degrees)
        self.get_logger().info(f"Intialised Teleop node")

    def joy_callback(self, msg):
        drive_msg = AckermannDriveStamped()
        
        # Xbox Left Stick Up/Down is axes[1]
        drive_msg.drive.speed = msg.axes[1] * self.max_speed
        
        # Xbox Left Stick Left/Right is axes[0]
        # (Sometimes this is Right Stick, which is axes[3])
        drive_msg.drive.steering_angle = msg.axes[0] * self.max_steering

        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()