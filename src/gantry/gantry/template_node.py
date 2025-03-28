#!/usr/bin/python3

import rclpy
from rclpy.node import Node

class TemplateNode(Node): # modify name

    def __init__(self):
        super().__init__('template_node') # modify name

        self.get_logger().info('this is a template node.') # comment out this line

def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode() # modify name
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()