import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        try:
            key = self.get_key()
            if key == 'w':
                twist.linear.x = 0.2
            elif key == 's':
                twist.linear.x = -0.2
            elif key == 'a':
                twist.angular.z = 0.2
            elif key == 'd':
                twist.angular.z = -0.2
            elif key == 'q':
                raise KeyboardInterrupt
        except KeyboardInterrupt:
            print("Exiting...")
            sys.exit()

        self.publisher_.publish(twist)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()