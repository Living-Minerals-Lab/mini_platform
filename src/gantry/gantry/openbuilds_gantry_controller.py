import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.parameter
import time
import pyautogui
import argparse
import sys

from utils.gantry_ctrl import OpenBuildsGantryController
from custom_interfaces.action import MoveGantry
from std_msgs.msg import String
from utils.servers import RealServer

class OpenBuildsGantryROSController(RealServer):
    """
    Controller for OpenBuilds gantry system.

    Node Name:
        * **ob_gantry_controller** *

    Action Servers:
        * **/move_gantry** (:class:`custom_interfaces.action.MoveGantry`) *

    Publisher: 
        * **/ob_gantry_status** (:class:`std_msgs.msg.String`) *

    """
    def __init__(self):
        super().__init__(node_name='ob_gantry_controller',
                         action_name='move_gantry',
                         action_type=MoveGantry)
        
        self.ob_gantry_ctrl = OpenBuildsGantryController('http://localhost:3000')

        # In addition to the action server, create a publisher publishing the device status continuously
        # ['Run', 'Running', 'Idle', 'Pending']
        self._publisher = self.node.create_publisher(String, 'gantry_status', 10)
        self.timer = self.node.create_timer(timer_period_sec=0.5, callback=self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.ob_gantry_ctrl.gantry_status
        self._publisher.publish(msg)

    def generate_feedback_message(self, elapsed):
        """
        Create a feedback message that populates the percent completed.

        Returns:
            :class:`custom_inferfaces.action.MoveGantry`: the populated feedback message
        """
        msg = self.action_type.Feedback() 
        msg.msg = f'Elapsed {elapsed} sec.'
        return msg

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        if self.ob_gantry_ctrl.gantry_ready():
            self.node.get_logger().info('Received and accepted a goal')
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.node.get_logger().info('Received and rejected a goal because gantry is already running')
            return rclpy.action.server.GoalResponse.REJECT
    
    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Send the request to the gantry system.
        Wait until the requested movement is completed. 
        Update result status and content according while waiting. 

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        self.node.get_logger().info('Executing a goal')
        self.node.get_logger().info('Sending command to gantry.')

        self.ob_gantry_ctrl.run_one_line_gcode(goal_handle.request.cmd)

        self.node.get_logger().info('Gantry is moving.')

        freq = 10
        interval = 1.0 / freq
        start_time = time.time()
 
        while True:
            time.sleep(interval)
            elapsed = time.time() - start_time
            with self.goal_lock:
                if goal_handle.is_active:
                    if goal_handle.is_cancel_requested:
                        result = self.generate_cancelled_result()
                        message = f'Goal cancelled at {elapsed:.2f}'
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        return result
                    # ideally would never come to this branch because repeated goals would be rejected
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = f'Goal pre-empted at {elapsed:.2f}'
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        return result
                    elif self.ob_gantry_ctrl.gantry_ready(): 
                        self.node.get_logger().info(f'Sending feedback {elapsed:.2f}')
                        result = self.generate_success_result()
                        message = 'Goal executed with success'
                        self.node.get_logger().info(message)
                        goal_handle.succeed()
                        return result
                    else:
                        self.node.get_logger().info(f'Sending feedback {elapsed:.2f}')
                        goal_handle.publish_feedback(
                            self.generate_feedback_message(elapsed)
                        )
                else:  # ! active
                    self.node.get_logger().info('Goal is no longer active, aborting')
                    result = self.action_type.Result()
                    return result

def main():
    """
    Entry point for the mock rotation controller node.
    """
    parser = argparse.ArgumentParser(description='Start a ROS server for Openbuilds gantry.')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    ob_gantry = OpenBuildsGantryROSController()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(ob_gantry.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        ob_gantry.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()