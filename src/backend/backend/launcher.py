#!/usr/bin/python3

import rclpy
import subprocess
import time
import os
import signal
from rclpy.node import Node
from custom_interfaces.srv import LaunchRequest

class BackendLauncher(Node):
    def __init__(self):
        super().__init__('backend_launcher')
        self.launched_files = {}
        self.srv = self.create_service(LaunchRequest, 'launch', self.launch_callback)

    def launch_callback(self, request, response):
        """
        This callback handles three types of requests from the front end.
        1. Run a ROS2 node / launch a ROS2 launch file in a separate process.
        2. Execute a ROS2 CLI command (this isn't the designed purpose for this service and is only temporary a workaound as
        using roslibjs for action calls is not supported for ROS2).
        3. Kill the processes for type 1 requests.
        """
        response.file_name = request.file_name
        
        # handle type 1 requests
        if request.cmd in ['launch', 'run']:
            if self.get_luanch_key(request) in self.launched_files:
                response.message = f'{request.package} {request.file_name} has already been launched.'
                
                response.is_launched = True
            else:
                self.launch_one(request, response)
        # handle tpye 2 requests
        elif request.cmd == 'cli':
            bash_cmd = ' '.join([request.package, request.file_name, request.args])
            subprocess.run(bash_cmd, shell=True)
        # handle type 3 requests
        else:
            if self.get_luanch_key(request) not in self.launched_files:
                response.message = f'{request.package} {request.file_name} has not been launched yet.'
                response.is_launched = False
            else:
                self.kill_one(request, response)
        
        return response
        
    def launch_one(self, request, response):
        bash_cmd = ' '.join(['ros2 ', request.cmd, request.package, request.file_name, request.args])
        self.get_logger().info(f'Executing: {bash_cmd}')
        p = subprocess.Popen(bash_cmd, shell=True, start_new_session=True)

        # Wait for the process to fail if luanch is not successful.  
        time.sleep(1)
        state = p.poll()
        if state is None: # launch succeeded
            response.is_launched = True
            response.message = f'{request.package} {request.file_name} was launched successfully.'
            self.launched_files[self.get_luanch_key(request)] = p.pid
        else: # launch failed
            response.is_launched = False
            response.message = f'{request.package} {request.file_name} failed to launch.'

    def kill_one(self, request, response):
        pid = self.launched_files[self.get_luanch_key(request)]
        os.killpg(os.getpgid(pid), signal.SIGINT)
        del self.launched_files[self.get_luanch_key(request)]
        response.is_launched = False
        response.message = f'{request.package} {request.file_name} stopped successfully.'
    
    def kill_all(self):
        for pk_file in self.launched_files:
            os.killpg(os.getpgid(self.launched_files[pk_file]),  signal.SIGINT)
        # del self.launched_files[pk_file]
        self.launched_files = {}
    
    def get_luanch_key(self, request):
        return request.package + ' ' + request.file_name

def main(args=None):
    rclpy.init(args=args)

    node = BackendLauncher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        print(e)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.kill_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()