import imp
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveGantry

from utils.gantry_ctrl import OpenBuildsGantryController

class MoveGantryService(Node):
    def __init__(self):
        super().__init__('move_gantry_srv')
        self.gantry_controller = OpenBuildsGantryController('http://localhost:3000')
        self.srv = self.create_service(MoveGantry, 'move_gantry', self.move_gantry_callback)

    def move_gantry_callback(self, request, response):
        response.response = 'succeed'
        self.get_logger().info(f'Incoming request\n{request.cmd}')
        self.gantry_controller.run_one_line_gcode(request.cmd)

        return response

def main(args=None):
    rclpy.init(args=args)

    node = MoveGantryService()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()