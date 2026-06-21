#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.create_subscription(
            msg_type=LaserScan,
            topic=lidarscan_topic,
            qos_profile=5,
            callback=self.scan_callback
        )
        self.drive_publisher = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic=drive_topic,
            qos_profile=5
        )
        # set PID gains
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('lookahead', 1.0)
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.lookahead_distance = self.get_parameter('lookahead').value
        self.ki = 0.0

        # TODO: store history
        # self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.target_dist = 1.0
        self.a_angle = 45
        self.b_angle = 90
        self.get_logger().info(f"Parameters using : kp : {self.kp}, kd : {self.kd}, LA : {self.lookahead_distance}")
    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. 
        Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        if angle < self.angle_min or angle > self.angle_max :
                    return None
        index = int((angle - self.angle_min ) / self.angle_increment)
        distance = range_data[index]
        # Replaced the sweeping while loop with the Failsafe Clip
        if np.isnan(distance) or np.isinf(distance):
            # distance = 100.0
            index -= 1
            distance = range_data[index]
        return distance

    def get_error(self, range_data, desired_dist):
        """
        Calculates the error to the wall. 
        Follow the wall to the left (going counter clockwise in the Levine loop). 
        You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        ## Angles for left hand wall
        angle_a = np.radians(self.a_angle)
        angle_b = np.radians(self.b_angle)
        swing = angle_b - angle_a

        ## distances a nd b  calualting alpha 
        distance_a = self.get_range(range_data, angle_a)
        distance_b = self.get_range(range_data, angle_b)
        alpha = np.arctan2((distance_a * np.cos(swing)-distance_b),(distance_a * np.sin(swing)))

        ## calcualtign current and future errors
        
        current_error = distance_b * np.cos(alpha)
        projected_error = current_error + self.lookahead_distance * np.sin(alpha)
        
        ## this is becasue we are lookign at left wall
        ## left means steerign is left in ros2 covneting
        ## so postive drives right throguht the weall rather than moving away
        ## tahts why fro -ve error we ahve to subtract projected error from desired_dist
        self.error = projected_error - desired_dist 
        return self.error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.prev_time
        # If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
        # If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
        # Otherwise, the speed should be 0.5 meters per second
        angle = self.kp * error + self.kd * ((error - self.prev_error) / dt)
        abs_angle = np.abs(angle)
        angle_mappings = np.deg2rad([0,10,20])
        if abs_angle >= angle_mappings[0] and abs_angle <= angle_mappings[1]:
             velocity = 1.5
        elif abs_angle >= angle_mappings[1] and abs_angle <= angle_mappings[2]:
             velocity = 1.0
        else :
             velocity = 0.5
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

        
        
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_publisher.publish(drive_msg)
        self.prev_error = error
        self.prev_time = current_time

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        range_data = msg.ranges
        self.angle_min,self.angle_max,self.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        desired_distance = self.target_dist
        error = self.get_error(range_data,desired_distance) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()