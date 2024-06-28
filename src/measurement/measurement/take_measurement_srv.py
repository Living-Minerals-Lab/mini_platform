import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from utils.auto_measure import conduct_measure

class TakeMeasurementService(Node):

    def __init__(self):
        super().__init__('take_measurement_srv')

        self.srv = self.create_service(Empty, 'take_measurement', self.take_measurement_callback)
    
    def take_measurement_callback(self, request, response):
        # response.ready = self.gantry_controller.gantry_ready()
        conduct_measure()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TakeMeasurementService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()