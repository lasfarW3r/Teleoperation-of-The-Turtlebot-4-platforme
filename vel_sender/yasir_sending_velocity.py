import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import os

class TeleopLidarCamera(Node):
    def __init__(self):
        super().__init__('teleop_lidar_camera')

        # ROS 2 Publishers & Subscribers
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.laser_subscription = self.create_subscription(LaserScan, 'robot2/scan', self.laser_callback, 10)

        # Variables
        self.speed = 0.5  
        self.min_distance = float('inf')  
        self.bridge = CvBridge()  # Bridge for ROS <-> OpenCV

        self.camera_process = None  # Store the camera process
        self.camera_running = False  # Track camera state

        self.get_logger().info("Bdina : L3 => Left, Right; R2 => Accelerate. D-Pad Up/Down to adjust speed.")

    def joy_callback(self, msg):
        """ Handles PS4 controller input """
        if msg.axes[7] == 1.0:  
            self.speed = min(self.speed + 0.2, 6.0)  
        elif msg.axes[7] == -1.0:  
            self.speed = max(0.2, self.speed - 0.2)  

        twist = Twist()
        twist.linear.x = msg.buttons[7] * self.speed
        twist.angular.z = msg.axes[0] * self.speed  

        # Stop if obstacle detected
        # if self.min_distance < 0.6:  
        #     twist.linear.x = 0.0  
        #     self.get_logger().warn("Obstacle détecté ! Arrêt du robot.")

        self.vel_publisher.publish(twist)
        self.get_logger().info(f"Vitesse: {self.speed:.2f}")

        # Check if "X" button (button index 0 on PS4 controller) is pressed
        if msg.buttons[0] == 1:
            self.toggle_camera()

    def laser_callback(self, msg):
        """ Handles LiDAR data """
        valid_ranges = [d for d in msg.ranges if 0.5 < d < float('inf')]
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')

    def toggle_camera(self):
        """ Start or Stop Camera Node """
        if not self.camera_running:
            self.get_logger().info("Launching Camera Node...")
            self.camera_process = subprocess.Popen(["ros2", "run", "vel_sender", "camera"])
            self.camera_running = True
        else:
            self.get_logger().info("Stopping Camera Node...")
            self.camera_process.terminate()  # Stop the camera node
            self.camera_running = False

def main():
    rclpy.init()
    node = TeleopLidarCamera()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
