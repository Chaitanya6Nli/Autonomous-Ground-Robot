import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal = Pose()
        self.goal.position.x = 2.0  # Set your goal x
        self.goal.position.y = 2.0  # Set your goal y
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        error = math.sqrt((self.goal.position.x - current_x)**2 + (self.goal.position.y - current_y)**2)
        
        self.integral += error
        derivative = error - self.prev_error
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        twist = Twist()
        twist.linear.x = min(output, 0.2)  # Limit max speed
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(self.goal.position.y - current_y, self.goal.position.x - current_x)
        current_angle = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        angle_error = angle_to_goal - current_angle
        
        twist.angular.z = 0.5 * angle_error
        
        self.publisher_.publish(twist)
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()