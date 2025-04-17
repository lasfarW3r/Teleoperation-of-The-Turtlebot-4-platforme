import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SpeedRegulatorNode(Node):
    def __init__(self):
        super().__init__('speed_regulator')
        self.vel_publisher = self.create_publisher(Twist, 'robot2/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'robot2/odom', self.odom_callback, 10)
        
        self.speed_target = 0.5  # Vitesse souhaitée (modifiable)
        self.current_speed = 0.0  # Vitesse mesurée via /odom
        
        # Paramètres PID
        self.Kp = 1.5
        self.Ki = 0.1
        self.Kd = 0.05
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.get_logger().info("Démarrage du régulateur de vitesse...")
        
        # Timer pour appliquer le PID toutes les 100 ms
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """Mise à jour de la vitesse actuelle du robot."""
        self.current_speed = msg.twist.twist.linear.x

    def control_loop(self):
        """Application du PID pour réguler la vitesse."""
        error = self.speed_target - self.current_speed
        self.integral += error * 0.1
        derivative = (error - self.prev_error) / 0.1
        
        # Calcul du signal de correction PID
        pid_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Limiter la sortie du PID pour éviter des valeurs extrêmes
        max_speed = 2.0
        min_speed = -2.0
        pid_output = max(min(pid_output, max_speed), min_speed)
        
        # Publier la nouvelle vitesse
        twist = Twist()
        twist.linear.x = pid_output
        self.vel_publisher.publish(twist)
        
        # Mise à jour de l'erreur précédente
        self.prev_error = error
        
        # Affichage des valeurs pour le débogage
        self.get_logger().info(f"Vitesse actuelle: {self.current_speed:.2f} m/s | Cible: {self.speed_target:.2f} m/s | Correction PID: {pid_output:.2f}")


def main():
    rclpy.init()
    node = SpeedRegulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()