import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2  # m/s
        self.goal_x = 2.0  # Set your goal point here
        self.obstacle_distance = float('inf')
        self.current_x = 0.0
        self.start_x = None

    def timer_callback(self):
        msg = Twist()
        
        if self.start_x is None:
            self.get_logger().info('Waiting for initial position...')
            return

        # Task 1: Move with constant x-axis velocity
        msg.linear.x = self.linear_velocity

        # Task 2: Stop when reaching goal point
        distance_traveled = abs(self.current_x - self.start_x)
        if distance_traveled >= self.goal_x:
            msg.linear.x = 0.0
            self.get_logger().info(f'Goal reached! Distance traveled: {distance_traveled:.2f} m')

        # Task 3: Stop before obstacle
        if self.obstacle_distance < 0.5:
            msg.linear.x = 0.0
            self.get_logger().info(f'Obstacle detected at {self.obstacle_distance:.2f} m!')

        self.publisher_.publish(msg)
        self.get_logger().info(f'Current position: {self.current_x:.2f}, Distance to goal: {self.goal_x - distance_traveled:.2f}')

    def scan_callback(self, msg):
        # Update obstacle distance (assuming the obstacle is directly in front)
        self.obstacle_distance = min(msg.ranges)

    def odom_callback(self, msg):
        # Update current x position
        self.current_x = msg.pose.pose.position.x
        if self.start_x is None:
            self.start_x = self.current_x
            self.get_logger().info(f'Starting position set to {self.start_x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtlebotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
