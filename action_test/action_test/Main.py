#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from action_test.competition_interface import CompetitionInterface
from action_test.robot_control import RobotControl

def main(args=None):
    '''
    main function for the move_robot_with_action_client script.
    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)

    robot_control = RobotControl()
    interface = CompetitionInterface(robot_control)

    executor = MultiThreadedExecutor()

    executor.add_node(interface)
    executor.add_node(robot_control)

    interface.start_competition()
    interface.wait(3) # Wait for controllers to come online

    robot_control.ChangeGripper("kts1","trays")

    
    while rclpy.ok():
        try:
            executor.spin_once()
            if robot_control.floor_robot_at_home and robot_control.ceiling_robot_at_home:
                break
        except KeyboardInterrupt:
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()