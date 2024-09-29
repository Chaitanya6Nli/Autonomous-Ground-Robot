import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
import math
import numpy as np
from .path_planner import AStarPlanner

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal = Pose()
        self.goal.position.x = 2.0  # Set your goal x
        self.goal.position.y = 2.0  # Set your goal y
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0
        self.path = []
        self.current_target = 0
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.planner = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.planner = AStarPlanner(self.map_data, self.resolution, self.origin)

    def odom_callback(self, msg):
        if self.planner is None:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        if not self.path:
            start = self.planner.world_to_map((current_x, current_y))
            goal = self.planner.world_to_map((self.goal.position.x, self.goal.position.y))
            self.path = self.planner.plan(start, goal)
            if self.path is None:
                self.get_logger().info('No path found!')
                return
            self.path = [self.planner.map_to_world(p) for p in self.path]
            self.current_target = 0

        target = self.path[self.current_target]
        
        error = math.sqrt((target[0] - current_x)**2 + (target[1] - current_y)**2)
        
        if error < 0.1:  # If close to current target, move to next
            self.current_target += 1
            if self.current_target >= len(self.path):
                self.get_logger().info('Goal reached!')
                return
            target = self.path[self.current_target]
            error = math.sqrt((target[0] - current_x)**2 + (target[1] - current_y)**2)

        self.integral += error
        derivative = error - self.prev_error
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        twist = Twist()
        twist.linear.x = min(output, 0.2)  # Limit max speed
        
        # Calculate angle to target
        angle_to_target = math.atan2(target[1] - current_y, target[0] - current_x)
        current_angle = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        angle_error = angle_to_target - current_angle
        
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