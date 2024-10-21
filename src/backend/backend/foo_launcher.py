#!/usr/bin/python3

import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String

class Launcher(Node): # modify name

    def __init__(self):
        super().__init__('backend_launcher') # modify name

        # self.get_logger().info('this is a template node.') # comment out this line

        self.subscription = self.create_subscription(String, 'launch_cmd', self.launcher_callback, 10)
    
    def launcher_callback(self, msg):
        subprocess.run(msg.data, shell=True)

def main(args=None):
    rclpy.init(args=args)
    node = Launcher() # modify name
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()