import rclpy
from rclpy.node import Node
from utils.gantry_ctrl import OpenBuildsGantryController
from custom_interfaces.srv import IsGantryRdy


class IsGantryReadyService(Node):

    def __init__(self):
        super().__init__('is_gantry_rdy_srv')

        self.gantry_controller = OpenBuildsGantryController('http://localhost:3000')
        self.srv = self.create_service(IsGantryRdy, 'is_gantry_rdy', self.is_gantry_rdy_callback)
    
    def is_gantry_rdy_callback(self, request, response):
        response.ready = self.gantry_controller.gantry_ready()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IsGantryReadyService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()