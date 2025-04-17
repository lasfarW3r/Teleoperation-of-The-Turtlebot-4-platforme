
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class CameraToOpenCV(Node):
#     def __init__(self):
#         super().__init__('camera_to_opencv')
#         self.bridge = CvBridge()  # Used to convert ROS images to OpenCV
#         # Create a subscription to the camera topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/robot2/oakd/rgb/preview/image_raw',  # Adjusted topic
#             self.image_callback,
#             10
#         )
#         self.get_logger().info("Subscribed to robot2/oakd/rgb/preview/image_raw")
#     def image_callback(self, msg):
#         try:
#             # Convert the ROS Image message to an OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Log the shape of the image
#             self.get_logger().info(f"Received image with shape: {cv_image.shape}")
            
#             # Display the image using OpenCV
#             cv2.imshow("YW3R TB4 Cam", cv_image)
#             cv2.waitKey(1)  # Refresh the window
#         except Exception as e:
#             self.get_logger().error(f"Error converting image: {e}")


# def main():
#     rclpy.init()
#     node = CameraToOpenCV()
#     rclpy.spin(node)
#     cv2.destroyAllWindows()  # Close OpenCV window when node shuts down
#     rclpy.shutdown()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading

class PIDSpeedControl(Node):
    def __init__(self):
        super().__init__('speed_regulator')

        # Publisher pour envoyer les commandes de vitesse
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber pour récupérer la vitesse actuelle
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialisation PID
        self.Kp = 2.0
        self.Ki = 0.1
        self.Kd = 0.05

        self.target_speed = 0.5  # Vitesse cible initiale (modifiable par l'utilisateur)
        self.current_speed = 0.0
        self.integral = 0.0
        self.previous_error = 0.0

        # Thread pour changer la vitesse en direct
        self.running = True
        self.input_thread = threading.Thread(target=self.get_user_input, daemon=True)
        self.input_thread.start()

    def get_user_input(self):
        """Permet à l'utilisateur de changer la vitesse cible en direct."""
        while self.running:
            try:
                new_speed = float(input("Entrez une nouvelle vitesse cible (m/s) : "))
                self.target_speed = new_speed
                self.get_logger().info(f"Nouvelle vitesse cible : {self.target_speed:.2f} m/s")
            except ValueError:
                self.get_logger().warn("Veuillez entrer un nombre valide.")

    def odom_callback(self, msg):
        """Met à jour la vitesse actuelle du robot en lisant /odom."""
        self.current_speed = msg.twist.twist.linear.x

    def compute_pid(self):
        """Applique l'algorithme PID pour ajuster la vitesse."""
        error = self.target_speed - self.current_speed
        self.integral += error
        derivative = error - self.previous_error

        # Correction PID
        pid_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error

        # Envoi de la commande de vitesse avec limites
        twist = Twist()
        twist.linear.x = max(0.0, min(pid_output, 3.0))  # Vitesse limitée entre 0 et 3 m/s
        self.vel_publisher.publish(twist)

        # Log pour voir l'effet du PID
        self.get_logger().info(f"Vitesse actuelle: {self.current_speed:.2f} m/s | Cible: {self.target_speed:.2f} m/s | Correction PID: {pid_output:.2f}")

    def run(self):
        """Boucle principale du noeud."""
        while rclpy.ok() and self.running:
            self.compute_pid()
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self):
        """Arrête proprement le thread d'entrée utilisateur."""
        self.running = False
        self.input_thread.join()


def main():
    rclpy.init()
    node = PIDSpeedControl()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
