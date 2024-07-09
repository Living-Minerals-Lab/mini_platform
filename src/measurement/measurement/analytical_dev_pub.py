import rclpy
from rclpy.node import Node
from utils.analytical_dev_ctrl import Z300Controller
from std_msgs.msg import Int64

class AnalyticalDevicePublisher(Node):

    def __init__(self):
        super().__init__('analytical_dev_pub')

        self.device_controller = Z300Controller()
        self._publisher = self.create_publisher(msg_type=Int64,
                                                 topic='anal_dev_status',
                                                 qos_profile=10
                                                 )
        
        self.timer = self.create_timer(timer_period_sec=0.5, 
                                       callback=self.timer_callback)

    def timer_callback(self):
        msg = Int64()
        msg.data = self.device_controller.dev_status.value
        self._publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = AnalyticalDevicePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()