import rclpy
from rclpy.node import Node
from utils.analytical_dev_ctrl import Z300Controller
from custom_interfaces.srv import IsRdy


class IsAnalyticalDeviceReadyService(Node):

    def __init__(self):
        super().__init__('is_analytical_dev_rdy_srv')

        self.device_controller = Z300Controller()
        self.srv = self.create_service(IsRdy, 'is_analytical_dev_rdy', self.is_device_rdy_callback)
    
    def is_device_rdy_callback(self, request, response):
        response.ready = self.device_controller.is_device_ready()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IsAnalyticalDeviceReadyService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()