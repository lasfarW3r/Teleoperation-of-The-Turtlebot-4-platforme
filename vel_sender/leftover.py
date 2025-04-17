import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import paho.mqtt.client as mqtt
import json


class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_control')
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.speed = 0.5  
        self.mqtt_client = mqtt.Client("PS4_Controller")  
        self.mqtt_client.connect("192.168.1.107", 1883, 60) 

        self.get_logger().info("Bdina : L3 => Left, Right; R2 => Accelerate. D-Pad Up/Down to adjust speed.")

    def joy_callback(self, msg):
    
        if msg.axes[7] == 1.0:  
            self.speed = min(self.speed + 0.2, 2.0)  
        elif msg.axes[7] == -1.0:  
            self.speed = max(0.2, self.speed - 0.2)  

        twist = Twist()
        twist.linear.x = msg.buttons[7] * self.speed  
        twist.angular.z = msg.axes[0] * self.speed  

        self.vel_publisher.publish(twist)

        mqtt_message = {
            "linear_x": twist.linear.x,
            "angular_z": twist.angular.z,
            "speed": self.speed
        }
        self.mqtt_client.publish("turtlebot/velocity", json.dumps(mqtt_message))

        self.get_logger().info(f"Speed: {self.speed:.2f}, Linear.x: {twist.linear.x:.2f}, Angular.z: {twist.angular.z:.2f}")
        self.get_logger().info(f"Sent MQTT: {mqtt_message}")


def main():
    rclpy.init()
    node = PS4ControllerNode()
    rclpy.spin(node)
    node.mqtt_client.disconnect()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()
