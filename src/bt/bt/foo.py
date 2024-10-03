from ast import arg, main
import py_trees
import py_trees_ros
import rclpy
import time
import typing
import sys
import py_trees.console as console
import py_trees_ros.subscribers as subscribers

from rclpy.node import Node
from std_msgs.msg import String, Int64
from py_trees.behaviours import Failure
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from py_trees import logging
from py_trees.composites import Sequence, Selector, Parallel
from utils.analytical_dev_ctrl import Z300Controller
from utils.bt import AreAllSamplesMeasured
from utils.gantry_ctrl import OpenBuildsGantryController
from custom_interfaces.action import TakeMeasurement, MoveGantry

class AreAllSampleMeasured(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.bb = self.attach_blackboard_client(name="are all sample measured?")
        self.bb.register_key('gcode', access=Access.WRITE)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        if not self.bb.gcode:
            self.logger.debug(f'All samples have been measured.')
            return Status.SUCCESS

        return Status.FAILURE

class SetNextSample(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.bb = self.attach_blackboard_client(name="set next sample")
        self.bb.register_key('gcode', access=Access.WRITE)
        self.bb.register_key('gantry_command', access=Access.WRITE)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        if not self.bb.gcode:
            self.logger.debug(f'No samples need to be measured.')
            return Status.FAILURE
        
        goal = MoveGantry.Goal()
        # goal.cmd = self.bb.gcode.pop()
        goal.cmd = self.bb.gcode[-1]
        self.bb.set(name='gantry_command', value=goal)

        return Status.SUCCESS
    
class UpdateLeftSamples(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.bb = self.attach_blackboard_client(name="update left samles")
        self.bb.register_key('gcode', access=Access.WRITE)
        # self.bb.register_key('gantry_command', access=Access.WRITE)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        if not self.bb.gcode:
            self.logger.debug(f'No samples need to be measured.')
            return Status.FAILURE
        
        # goal = MoveGantry.Goal()
        # goal.cmd = self.bb.gcode.pop()
        self.bb.gcode.pop()
        # self.bb.set(name='gantry_command', value=goal)

        return Status.SUCCESS

def gen_gcode(x_interval: float, y_interval: float, x_points: int, y_points: int, z_safe: float) -> list[str]:
    """Generate a gcode script that visits positions on a grid.
    All units are millimeter.

    Args:
        x_interval (float): inverval between two adjacent positions on x-scale.
        y_interval (float): inverval between two adjacent positions on x-scale.
        x_points (int): number of positions on x-scale.
        y_points (int): number of positions on y-scale.
        z_safe (float): safety distance on z-scale used when traveling between positions.
    Returns:
        list[str]: generated gcode script.
    """
    gcode = []


    x_pos, y_pos = [], []
 
    for i in range(x_points):
        x_pos.append(x_interval * i)

    for i in range(y_points):
        y_pos.append(y_interval * i)
    
    for x in x_pos:
        for y in y_pos:
            gcode.append('G17 G21 G90 ' + '\n'
                         f'G00 X{x} Y{y}' + '\n')
    
    return gcode

def create_root():
    root = Parallel(
        name="mini_platform",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics_to_bb = Sequence(name="topics_to_bb", memory=True)

    gantry_to_bb = subscribers.ToBlackboard(name='gantry_to_bb', 
                                            topic_name='gantry_status',
                                            topic_type=String,
                                            blackboard_variables={'gantry_status': 'data'},
                                            qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                            )
    
    analytical_to_bb = subscribers.ToBlackboard(name='anal_dev_to_bb', 
                                                topic_name='z300_status',
                                                topic_type=Int64,
                                                blackboard_variables={'z300_status': 'data'},
                                                qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                                )

    tasks = Selector(name='main task', memory=False)
    
    all_samples_measured = AreAllSampleMeasured('are all samples measured?')

    safety = Failure(name='not safe?')

    measure_one_sample = Sequence('measure one sample', True)

    set_next_goal = SetNextSample('set next goal')

    wait_for_goal = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForGoal",
        variable_name="/gantry_command"
    )

    move = Sequence(name='move', memory=True)

    move_up = py_trees_ros.action_clients.FromConstant(name='move_up',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      action_goal=MoveGantry.Goal(cmd='G17 G21 G90 \n G00 Z10 \n'),
                                                      )
    
    move_xy = py_trees_ros.action_clients.FromBlackboard(name='move_xy',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      key='gantry_command',
                                                      )
    
    move_down = py_trees_ros.action_clients.FromConstant(name='move_down',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      action_goal=MoveGantry.Goal(cmd='G17 G21 G90 \n G00 Z0 \n'),
                                                      )
    
    measure = py_trees_ros.action_clients.FromConstant(name='measure',
                                                       action_type=TakeMeasurement,
                                                       action_name='take_measurement',
                                                       action_goal=TakeMeasurement.Goal(),
                                                       )
    
    update_left_samples = UpdateLeftSamples('update left samples')

    root.add_children([topics_to_bb, tasks])

    topics_to_bb.add_children([gantry_to_bb, analytical_to_bb])

    tasks.add_children([safety, all_samples_measured, measure_one_sample])

    measure_one_sample.add_children([set_next_goal, wait_for_goal, move, measure, update_left_samples])

    move.add_children([move_up, move_xy, move_down])

    return root

def main():
    logging.level = logging.Level.DEBUG

    blackboard = py_trees.blackboard.Client(name='Global')
    blackboard.register_key('gantry_status', access=Access.READ)
    blackboard.register_key('z300_status', access=Access.READ)
    blackboard.register_key('gcode', access=Access.WRITE)
    blackboard.register_key('gantry_command', access=Access.WRITE)

    blackboard.gcode = gen_gcode(120 / 2, 36 / 2, 3, 3, 10)

    rclpy.init(args=None)

    root = create_root()

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="foo", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(2000)


    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
