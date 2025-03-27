import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.parameter
import time
import argparse
import sys

from utils.analytical_dev_ctrl import Z300Controller
from custom_interfaces.action import ExportSpectrum
from std_msgs.msg import String
from utils.servers import RealServer
from typing import override

class Z300ExportActionServer(RealServer):
    """
    Z300 action server for export a spectrum.

    Node Name:
        * **z300_export_action_server** *

    Action Servers:
        * **/export_spectrum** (:class:`custom_interfaces.action.ExportSpectrum`) *
    """
    def __init__(self):
        super().__init__(node_name='z300_export_action_server',
                         action_name='export_spectrum',
                         action_type=ExportSpectrum)
        
        self.z300_ctrl = Z300Controller('http://192.168.50.2:1234')
    
    @override
    def generate_feedback_message(self, elapsed_time):
        """
        Create a feedback message that populates the time elapsed.

        Returns:
            :class:`std_msgs.msg.String`: the populated feedback message
        """
        msg = self.action_type.Feedback() 
        msg.feedback = f'Time elapsed: {elapsed_time:.2%} s.'
        return msg

    @override
    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        if self.z300_ctrl.is_device_ready():
            self.node.get_logger().info('Received and accepted a goal: export')
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.node.get_logger().info('Received and rejected a goal because z300 is already running')
            return rclpy.action.server.GoalResponse.REJECT
    
    @override
    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Emit 'export' event to LIBS socket.io server and wait for the device to be ready.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        self.node.get_logger().info('Executing a goal: export')
        
        self.z300_ctrl.export()

        freq = 1
        interval = 1.0 / freq
        start_time = time.time()
 
        while True:
            time.sleep(interval)
            elapsed_time = time.time() - start_time
            with self.goal_lock:
                if goal_handle.is_active:
                    if goal_handle.is_cancel_requested:
                        result = self.generate_cancelled_result()
                        message = f'Goal cancelled at {elapsed_time:.2%}'
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        return result
                    # ideally would never come to this branch because repeated goals would be rejected
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = f'Goal pre-empted at {elapsed_time:.2%}'
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        return result
                    elif self.z300_ctrl.is_device_ready():
                        self.node.get_logger().info(f'Sending feedback {elapsed_time:.2%}')
                        result = self.generate_success_result()
                        message = 'Goal executed with success: export'
                        self.node.get_logger().info(message)
                        goal_handle.succeed()
                        return result
                    else:
                        self.node.get_logger().info(f'Sending feedback {elapsed_time:.2%}')
                        goal_handle.publish_feedback(
                            self.generate_feedback_message(elapsed_time)
                        )
                else:  # ! active
                    self.node.get_logger().info('Goal is no longer active, aborting')
                    result = self.action_type.Result()
                    return result


def main():
    """
    Entry point
    """
    parser = argparse.ArgumentParser(description='Start z300 export action server')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    z300 = Z300ExportActionServer()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(z300.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        z300.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()