import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.parameter
import time
import argparse
import sys

from utils.analytical_dev_ctrl import Z300Controller
from custom_interfaces.action import AnalyzeSpectrum
from std_msgs.msg import String
from utils.servers import RealServer

class Z300AnalyzeActionServer(RealServer):
    """
    Z300 action server for analyze a spectrum.

    Node Name:
        * **z300_analyze_action_server** *

    Action Servers:
        * **/analyze_spectrum** (:class:`custom_interfaces.action.AnalyzeSpectrum`) *
    """
    def __init__(self):
        super().__init__(node_name='z300_analyze_action_server',
                         action_name='analyze_spectrum',
                         action_type=AnalyzeSpectrum)
        
        self.z300_ctrl = Z300Controller('http://192.168.50.43:1234')

    def generate_feedback_message(self, elapsed_time):
        """
        Create a feedback message that populates the time elapsed.

        Returns:
            :class:`std_msgs.msg.String`: the populated feedback message
        """
        msg = self.action_type.Feedback() 
        msg.feedback = f'Time elapsed: {elapsed_time:.2} s.'
        return msg

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        if self.z300_ctrl.is_device_ready():
            self.node.get_logger().info('Received and accepted a goal: analyze')
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.node.get_logger().info('Received and rejected a goal because z300 is already running')
            return rclpy.action.server.GoalResponse.REJECT

    def generate_success_result(self):
        res = self.action_type.Result()
        rng, area = [], []
        for r in self.z300_ctrl.res['analyze']['val']:
            rng.append(r)
            area.append(self.z300_ctrl.res['analyze']['val'][r])
        res.peak_range = rng
        res.peak_area = area
        return res

    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Emit 'analyze' event to LIBS socket.io server and wait for the device to be ready.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        self.node.get_logger().info('Executing a goal: analyze')
        self.z300_ctrl.analyze()

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
                        message = f'Goal cancelled at {elapsed_time:.2} s'
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        return result
                    # ideally would never come to this branch because repeated goals would be rejected
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = f'Goal pre-empted at {elapsed_time:.2} s'
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        return result
                    elif self.z300_ctrl.res['analyze']['msg'] is not None:
                        self.node.get_logger().info(f'Time elapsed: {elapsed_time:.2} s')
                        if self.z300_ctrl.res['analyze']['msg'] == 'success':
                            message = f'Goal executed with success: analyze {self.z300_ctrl.res["analyze"]["val"]}'
                            self.node.get_logger().info(message)
                            result = self.generate_success_result()
                            goal_handle.succeed()
                        else:
                            message = f'Goal execution failed: analyze {self.z300_ctrl.res["analyze"]["val"]}'
                            self.node.get_logger().info(message)
                            result = self.generate_success_result()
                            goal_handle.abort()
                        return result
                    else:
                        self.node.get_logger().info(f'Time elapsed: {elapsed_time:.2} s')
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
    parser = argparse.ArgumentParser(description='Start z300 analyze action server')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    z300 = Z300AnalyzeActionServer()

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