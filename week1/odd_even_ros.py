import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import random

class IntegerPublisher(Node):
    def __init__(self):
        super().__init__('integer_publisher')
        self.publisher_ = self.create_publisher(Int32, 'integers', 10)
        self.timer = self.create_timer(1.0, self.publish_integer)

    def publish_integer(self):
        msg = Int32()
        msg.data = random.randint(1, 100)  # Generate a random integer between 1 and 100
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

class OddEvenClassifier(Node):
    def __init__(self):
        super().__init__('odd_even_classifier')
        self.subscription = self.create_subscription(
            Int32,
            'integers',
            self.classify_integer,
            10)
        self.publisher_ = self.create_publisher(String, 'oddeven', 10)

    def classify_integer(self, msg):
        result = 'even' if msg.data % 2 == 0 else 'odd'
        result_msg = String()
        result_msg.data = result
        self.publisher_.publish(result_msg)
        self.get_logger().info(f'Received: {msg.data}, Classified as: {result}')

def main(args=None):
    rclpy.init(args=args)
    
    integer_publisher = IntegerPublisher()
    odd_even_classifier = OddEvenClassifier()

    try:
        rclpy.spin(integer_publisher)
        rclpy.spin(odd_even_classifier)
    except KeyboardInterrupt:
        pass
    finally:
        integer_publisher.destroy_node()
        odd_even_classifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()