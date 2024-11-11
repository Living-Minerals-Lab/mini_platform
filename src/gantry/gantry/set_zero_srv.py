import rclpy
from rclpy.node import Node
from utils.gantry_ctrl import OpenBuildsGantryController
from custom_interfaces.srv import IsRdy, SetZero


class SetZeroService(Node):

    def __init__(self):
        super().__init__('is_gantry_rdy_srv')

        self.ob_gantry_ctrl = OpenBuildsGantryController('http://localhost:3000')
        self.srv = self.create_service(SetZero, 'set_zero', self.set_zero_callback)
    
    def set_zero_callback(self, request, response):
        self.ob_gantry_ctrl.set_zero(request.axis)
        # response.ready = self.gantry_controller.gantry_ready()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetZeroService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()