import rclpy
from rclpy.node import Node
from utils.gantry_ctrl import OpenBuildsGantryController
from std_msgs.msg import String


class GantryPublisher(Node):

    def __init__(self):
        super().__init__('gantry_pub')
        self.publisher_ = self.create_publisher(String, 'gantry_status', 10)
        timer_period = 0.1  # secondsrm
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gantry_controller = OpenBuildsGantryController('http://localhost:3000')
        # self.gantry_controller = OpenBuildsGantryController('http://131.243.227.233:3000')


    def timer_callback(self):
        msg = String()
        msg.data = self.gantry_controller.gantry_status
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        # self.get_logger().info(self.gantry_controller.client.sid)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GantryPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()