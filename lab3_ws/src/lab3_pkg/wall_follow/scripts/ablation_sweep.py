#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import time
import math
import csv
import itertools

class SweepMonitor(Node):
    def __init__(self):
        super().__init__('ablation_sweep_monitor')
        
        # Subscribe to odometry to track distance and velocity
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        # Publisher to teleport the car back to the start
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vel = 0.0
        self.lap_state = "START" # States: START, RACING, FINISHED, CRASHED
        self.low_vel_start_time = None

    def odom_callback(self, msg):
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract velocity (magnitude of linear x and y)
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        self.current_vel = math.sqrt(v_x**2 + v_y**2)

    def reset_car(self):
        """Teleports the car back to the starting line."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # TODO: Update these to the exact starting coordinates of your track!
        msg.pose.pose.position.x = 0.0 
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0 
        
        self.pose_pub.publish(msg)
        self.get_logger().info("Car reset to starting line.")

def main(args=None):
    rclpy.init(args=args)
    monitor = SweepMonitor()
    
    # 1. Define your Parameter Sweep Ranges
    kp_values = [0.1, 0.5, 0.7, 1.0]
    # kp_values = [0.5]
    kd_values = [0.0, 0.001, 0.01, 0.1]
    # kd_values = [0.001]
    lookahead_values = [0.2, 0.5, 1.0, 1.5]
    # lookahead_values = [1.0]
    
    combinations = list(itertools.product(kp_values, kd_values, lookahead_values))
    
    # 2. Setup CSV Logging
    csv_filename = 'pid_ablation_results.csv'
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Kp', 'Kd', 'Lookahead', 'Lap Time (s)', 'Result'])
        
        # 3. The Master Loop
        for kp, kd, la in combinations:
            print(f"\n[{combinations.index((kp,kd,la)) + 1}/{len(combinations)}]")
            
            # Reset state for the new run
            monitor.lap_state = "START"
            monitor.low_vel_start_time = None
            
            # Teleport car and give simulator a second to register
            monitor.reset_car()
            time.sleep(1.0)
            
            # Start the brain (Replace 'your_package_name' with your actual package!)
            node_cmd = [
                'ros2', 'run', 'wall_follow', 'wall_follow_node.py',
                '--ros-args',
                '-p', f'kp:={kp}',
                '-p', f'kd:={kd}',
                '-p', f'lookahead:={la}'
            ]
            process = subprocess.Popen(node_cmd)
            
            start_time = time.time()
            max_timeout = 45.0 # Max seconds before we assume it's lost
            lap_time = 999.0
            result = "TIMEOUT"
            
            # Levine Hall Track Sectors
            track_sector = 0
            # The Dynamic Monitor Loop
            # The Dynamic Monitor Loop
            while (time.time() - start_time) < max_timeout:
                # Manually spin the node once to fetch fresh /odom data
                rclpy.spin_once(monitor, timeout_sec=0.05)
                
                # Calculate distance from origin (start line)
                distance = math.sqrt(monitor.current_x**2 + monitor.current_y**2)
                run_time = time.time() - start_time
                
                # ---------------------------------------------------------
                # 1. GLOBAL CRASH DETECTOR (Runs immediately, in all sectors!)
                # ---------------------------------------------------------
                # We give the car 1.0 second of "grace period" to accelerate from a dead stop
                if run_time > 10.0:
                    if monitor.current_vel < 0.05:
                        if monitor.low_vel_start_time is None:
                            monitor.low_vel_start_time = time.time()
                        elif (time.time() - monitor.low_vel_start_time) > 2.0:
                            print("   -> CRASH DETECTED. Killing test.")
                            result = "CRASHED"
                            break
                    else:
                        monitor.low_vel_start_time = None # Reset if it moves again

                # ---------------------------------------------------------
                # 2. SECTOR TRACKING
                # ---------------------------------------------------------
                if track_sector == 0 and distance > 3.0:
                    track_sector = 1
                    monitor.lap_state = "RACING"
                    print("   -> [Sector 1] Car left the starting zone.")
                    
                elif track_sector == 1 and distance > 14.0:
                    track_sector = 2
                    print("   -> [Sector 2] Crossed far Levine midpoint.")
                    
                elif track_sector == 2 and distance < 7.0:
                    track_sector = 3
                    print("   -> [Sector 3] Returned near origin. Finish line active.")
                    
                elif track_sector == 3 and distance < 1.5:
                    lap_time = round(run_time, 3)
                    result = "SUCCESS"
                    print(f"   -> FINISHED! Valid Lap time: {lap_time}s")
                    break

            # ---------------------------------------------------------
            # 3. ZOMBIE KILLER (The Cleanup)
            # ---------------------------------------------------------
            # Force kill the specific subprocess
            process.kill() 
            process.wait()
            
            # Brute-force kill ANY lingering wall_follow nodes in the OS
            subprocess.run(['pkill', '-f', 'wall_follow_node.py'], check=False)
            
            writer.writerow([kp, kd, la, lap_time, result])
            file.flush() 
            time.sleep(2.0) # Short wait before teleporting for the next test

    print(f"\nSweep complete! Results saved to {csv_filename}")
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()