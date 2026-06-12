#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle Odometery message.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            qos_profile= 5
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            qos_profile= 5,
            callback=self.scan_callback
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            qos_profile= 5,
            callback=self.odom_callback
        )
        # TODO: create ROS subscribers and publishers.
        self.threshold = 0.5
        self.get_logger().info(f"Intialised ros2 node")
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        # self.get_logger().info(f"Current speed : {self.speed}")

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        if abs(self.speed) < 0.1:
            return
        ## Calculate angles
        ranges = np.array(scan_msg.ranges)
        num_beams = len(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment

        ## Filtering Ranges andf angles
        valid_mask   = np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]


        ## forward speed 
        closing_speeds = self.speed * np.cos(valid_angles)
        ## Lateral distance ( to avoid braking in the parallel walls)
        y_distances = valid_ranges  * np.sin(valid_angles)

        ## creatign a bubble such that to apply AEB only when the
        ## closing speeds is > 0 and the lateral_disatnce <= 0.25
        lateral_distance_threshold = 0.25
        closing_distance_mask = closing_speeds > 0.0
        lateral_distance_mask = np.abs(y_distances) <= lateral_distance_threshold
        
        ## Must be approachig ntowards wall sideways or forward
        valid_target_mask = lateral_distance_mask & closing_distance_mask
        

        ttc_array = np.full(valid_ranges.shape, np.inf)
        ttc_array[valid_target_mask] = valid_ranges[valid_target_mask] / (closing_speeds[valid_target_mask] + 1e-6)
        # self.get_logger().info(f"{ttc_array}")
        
            
        # TODO: publish command to brake
        min_ttc = np.min(ttc_array)
        self.get_logger().info(f"Min TTC: {min_ttc:.3f} | Speed: {self.speed:.2f}")
        if min_ttc <= self.threshold:
            self.get_logger().info(f"Distance near to threshold break triggered..")
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
            self.drive_publisher.publish(ack_msg)
        

            


        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()