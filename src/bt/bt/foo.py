from ast import arg
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
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from py_trees import logging
from py_trees.composites import Sequence, Selector, Parallel
from utils.analytical_dev_ctrl import Z300Controller
from utils.gantry_ctrl import OpenBuildsGantryController
from custom_interfaces.action import TakeMeasurement

class Move(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.gantry_ctrl = OpenBuildsGantryController('http://localhost:3000')
        self.bb = self.attach_blackboard_client(name="Move")
        self.bb.register_key('gcode', access=Access.WRITE)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        if not self.bb.gcode:
            self.logger.debug(f'All samples have been measured.')
            return Status.FAILURE
        
        command = self.bb.gcode.pop(0)
        self.gantry_ctrl.run_one_line_gcode(command)
        return Status.SUCCESS

class MoveRdy(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        
        self.bb = self.attach_blackboard_client(name="Move Ready")
        self.bb.register_key('gantry_status', access=Access.READ)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        status = self.bb.gantry_status

        if status == 'Idle':
            return Status.SUCCESS
        else:
            return Status.FAILURE

class Measure(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        
    
    def setup(self, **kwargs: typing.Any) -> None:
        self.z300_ctrl = Z300Controller()
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')

        if not self.z300_ctrl.is_device_ready():
            return Status.RUNNING
        
        self.z300_ctrl.measure()
        return Status.SUCCESS

class MeasureRdy(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        
        self.bb = self.attach_blackboard_client(name="Move Ready")
        self.bb.register_key('anal_dev_status', access=Access.READ)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        status = self.bb.anal_dev_status

        if status == 1:
            return Status.SUCCESS
        else:
            return Status.FAILURE

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
    gcode = ['G17 G21 G90 ']


    x_pos, y_pos = [], []
 
    for i in range(x_points):
        x_pos.append(x_interval * i)

    for i in range(y_points):
        y_pos.append(y_interval * i)
    
    for x in x_pos:
        for y in y_pos:
            gcode.append(f'G00 Z{z_safe}' + '\n')
            gcode.append(f'G00 X{x} Y{y}' + '\n')
            gcode.append(f'G00 Z0.0' + '\n')

    gcode.append('M2')
    
    return gcode

def create_root() -> Behaviour:
    """
    Create a behavior tree for controling the mini platform.

    Returns:
        the root of the tree
    """
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
                                                topic_name='anal_dev_status',
                                                topic_type=Int64,
                                                blackboard_variables={'anal_dev_status': 'data'},
                                                qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                                )

    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_child(topics_to_bb)
    topics_to_bb.add_child(gantry_to_bb)
    topics_to_bb.add_child(analytical_to_bb)
    root.add_child(priorities)
    priorities.add_child(flipper)
    priorities.add_child(idle)

    return root

def create_root_simple_seq():
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
                                                topic_name='anal_dev_status',
                                                topic_type=Int64,
                                                blackboard_variables={'anal_dev_status': 'data'},
                                                qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                                )

    tasks = Sequence(name='tasks',
                    memory=True)
    
    
    move = Move('move')

    measu = Measure('measure')

    root.add_child(move)
    root.add_child(measu)

    return root

def main():
    logging.level = logging.Level.DEBUG
    blackboard = py_trees.blackboard.Client(name='Global')
    blackboard.register_key('gantry_status', access=Access.READ)
    blackboard.register_key('anal_dev_status', access=Access.READ)
    blackboard.register_key('gcode', access=Access.WRITE)

    blackboard.gcode = gen_gcode(15, 20, 2, 2, 10)

    rclpy.init(args=None)
    # root = create_root()
    root = create_root_simple_seq()
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

    tree.tick_tock(5000, post_tick_handler=lambda a: print(blackboard))


    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

def test_action_client():
    logging.level = logging.Level.DEBUG

    root = Sequence('only measure', memory=True)

    measure = py_trees_ros.action_clients.FromConstant(name='measure',
                                                       action_type=TakeMeasurement,
                                                       action_name='take_measurement',
                                                       action_goal=TakeMeasurement.Goal(),
                                                       )

    root.add_child(measure)

    rclpy.init(args=None)

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

    tree.tick_tock(5000, post_tick_handler=lambda a: print(measure.blackboard))


    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    
    test_action_client()
