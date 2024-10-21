import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.parameter
import time
import argparse
import sys

from utils.analytical_dev_ctrl import Z300Controller
from custom_interfaces.action import TakeMeasurement
from std_msgs.msg import Int64
from utils.servers import RealServer

class Z300ROSController(RealServer):
    """
    Controller for Z300 LIBS gun.

    Node Name:
        * **z300_controller** *

    Action Servers:
        * **/take_measurement** (:class:`custom_interfaces.action.Takemeasurement`) *

    Publisher: 
        * **/z300_status** (:class:`std_msgs.msg.Int64`) *
    """
    def __init__(self):
        super().__init__(node_name='z300_controller',
                         action_name='take_measurement',
                         action_type=TakeMeasurement)
        
        self.z300_ctrl = Z300Controller('http://localhost:1234')
        
        # In addition to the action server, create a publisher publishing the device status continuously
        # 1: idle; 0: running 
        self._publisher = self.node.create_publisher(msg_type=Int64, topic='z300_status', qos_profile=10)
        
        self.timer = self.node.create_timer(timer_period_sec=0.5, callback=self.timer_callback)

    def timer_callback(self):
        msg = Int64()
        msg.data = self.z300_ctrl.dev_status.value
        self._publisher.publish(msg)

    def generate_feedback_message(self, percent_completed):
        """
        Create a feedback message that populates the percent completed.

        Returns:
            :class:`custom_inferfaces.action.TakeMeasurement`: the populated feedback message
        """
        msg = self.action_type.Feedback() 
        msg.feedback = f'Action completed: {percent_completed:.2%}.'
        return msg

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        if self.z300_ctrl.dev_status.value == 1:
            self.node.get_logger().info('Received and accepted a goal')
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.node.get_logger().info('Received and rejected a goal because z300 is already running')
            return rclpy.action.server.GoalResponse.REJECT
    
    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Locate the measure button.
        Then click the button to start spectra collection.
        Check pre-emption, cancel while wait for a fixed amount of time.
        Update result status and content according while waiting. 

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        self.node.get_logger().info('Executing a goal')
        self.node.get_logger().info('Locating trigger button')

        self.z300_ctrl.dev_status.value = 0

        # x, y = self.z300_ctrl.get_button_pos_multi_scale(self.z300_ctrl.measure_button_path)
        
        self.node.get_logger().info('Collecting LIBS spectra')
        
        self.z300_ctrl.pull_trigger()

        freq = 1
        interval = 1.0 / freq
        start_time = time.time()
        percent_completed = 0
 
        while True:
            time.sleep(interval)
            percent_completed = (time.time() - start_time) / self.z300_ctrl.measure_duration
            with self.goal_lock:
                if goal_handle.is_active:
                    if goal_handle.is_cancel_requested:
                        result = self.generate_cancelled_result()
                        message = f'Goal cancelled at {percent_completed:.2%}'
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        self.z300_ctrl.dev_status.value = 1
                        return result
                    # ideally would never come to this branch because repeated goals would be rejected
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = f'Goal pre-empted at {percent_completed:.2%}'
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        self.z300_ctrl.dev_status.value = 1
                        return result
                    elif percent_completed >= 1:
                        percent_completed = 1
                        self.node.get_logger().info(f'Sending feedback {percent_completed:.2%}')
                        result = self.generate_success_result()
                        message = 'Goal executed with success'
                        self.node.get_logger().info(message)
                        goal_handle.succeed()
                        self.z300_ctrl.dev_status.value = 1
                        return result
                    else:
                        self.node.get_logger().info(f'Sending feedback {percent_completed:.2%}')
                        goal_handle.publish_feedback(
                            self.generate_feedback_message(percent_completed)
                        )
                else:  # ! active
                    self.node.get_logger().info('Goal is no longer active, aborting')
                    result = self.action_type.Result()
                    self.z300_ctrl.dev_status.value = 1
                    return result

def main():
    """
    Entry point for the mock rotation controller node.
    """
    parser = argparse.ArgumentParser(description='Start a ROS server for z300')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    z300 = Z300ROSController()

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