#!/usr/bin/env python3

from math import pi
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
import rclpy
from queue import Queue
import threading
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from ariac_msgs.msg import *

from std_srvs.srv import Trigger
from ariac_msgs.srv import *
from tf2_ros import TransformException
from competition_tutorials.data import *
from geometry_msgs.msg import Pose,PoseStamped,Vector3,Quaternion
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_multiply, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointControllerState,JointTrajectoryControllerState
from competition_tutorials.Kinematics import *
from competition_tutorials.interpolation import *
from tf_transformations import *
import threading


class RobotControl(Node):
    floor_robot_joint_names = ['floor_shoulder_pan_joint',
                               'floor_shoulder_lift_joint',
                               'floor_elbow_joint',
                               'floor_wrist_1_joint',
                               'floor_wrist_2_joint',
                               'floor_wrist_3_joint',]

    floor_robot_home_joint_positions = [0.0, -pi/2, pi/2, -pi/2, -pi/2, 0.0]

    ceiling_robot_joint_names = ['ceiling_shoulder_pan_joint',
                                 'ceiling_shoulder_lift_joint',
                                 'ceiling_elbow_joint',
                                 'ceiling_wrist_1_joint',
                                 'ceiling_wrist_2_joint',
                                 'ceiling_wrist_3_joint',]

    ceiling_robot_home_joint_positions = [0.0, -pi/2, pi/2, pi, -pi/2, 0.0]
    
    
    
    def __init__(self):
        super().__init__('robot_control')

        sim_time = Parameter(
            "use_sim_time",
            Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        self._floor_robot_action_client = ActionClient(
            self, FollowJointTrajectory, '/floor_robot_controller/follow_joint_trajectory')

        self._ceiling_robot_action_client = ActionClient(
            self, FollowJointTrajectory, '/ceiling_robot_controller/follow_joint_trajectory')
        
        self.linear_action_client=ActionClient(self, FollowJointTrajectory, '/linear_rail_controller/follow_joint_trajectory')
        
        self.kitting_arm_joint_state_subscriber = self.create_subscription(JointTrajectoryControllerState, '/floor_robot_controller/state', self.kitting_arm_joint_state_callback, qos_profile_sensor_data)
        
        self.floor_robot_at_home = False
        self.ceiling_robot_at_home = False
        self.Success_done=True
        self.traj_quene=Queue()
        
        self.kitting_arm_joint_states=None
        
        self.kitting_arm_joint_names = [
            'floor_shoulder_pan_joint','floor_shoulder_lift_joint',  'floor_elbow_joint',
            'floor_wrist_1_joint', 'floor_wrist_2_joint','floor_wrist_3_joint'
        ]
        self.linear_joint_names = ['linear_actuator_joint']
        self.kitting_typical_joints = {
            "standby" : [1.5820963319369694, -1.6356888311746314, 1.921040497950596, -1.8276216909939276, -1.5708049327979872, -3.1302637783918614],
            "bin_agv_insert_joint": [-9.908213582932035e-06, -1.6356881698442969, 1.9210396904708134, 4.434232672827393, -1.570804295066237, -3.130262516117118],
            "flip_init_state" : [3.139979598681924, -1.0823125299292018, 1.7835319716553002, 5.542528446925819, -1.4273425719694686 - pi/2, -3.1399976775082745],
            "conveyor_insert": [1.4820999965423397+pi/2, -1.6356888311746864, 1.9210404979505746, -1.8276216909939889, -1.5708049327960403, -3.1302637783910976],

        }  
        self.kitting_base_x = -1.30 
        self.kitting_base_y = 0
        self.kitting_base_z = 0.93 
        # self.gripper = GripperManager(ns='floor')

        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
  
        self.init_matrix = numpy.matrix([[0.00000000e+00, -1.00000000e+00, 0.00000000e+00, 0.171000000e+00],
                                        [0.00000000e+00,  0.00000000e+00,  1.00000000e+00, -0.62300000e+00],
                                        [-1.00000000e+00, 0.00000000e+00,  0.00000000e+00, 0.500000e+00],
                                        [0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])

        self.init_rotation = numpy.matrix([[0.00000000e+00, -1.00000000e+00, 0.00000000e+00],
                                        [0.00000000e+00,  0.00000000e+00,  1.00000000e+00],
                                        [-1.00000000e+00, 0.00000000e+00,  0.00000000e+00]])

    def kitting_arm_joint_state_callback(self, msg):
        # print("msg.position",msg)
        self.kitting_arm_joint_states =msg.actual.positions

    def move_floor_robot(self, traj):

        goal_msg = FollowJointTrajectory.Goal()

        # Fill in the trajectory message
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = traj.joint_names
        goal_msg.trajectory.points = traj.points
        
        # future.add_done_callback(self.goal_response_callback) 
        self._floor_robot_action_client.wait_for_server()    
        # self.traj_quene.put(goal_msg)          
        # if self.Success_done:
        self.Success_done=False
        self._floor_robot_send_goal_future = self._floor_robot_action_client.send_goal_async(goal_msg)
    
        self._floor_robot_send_goal_future.add_done_callback(
        self.floor_robot_goal_response_callback)
    
    def move_linear_robot(self, traj):

        goal_msg = FollowJointTrajectory.Goal()

        # Fill in the trajectory message
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = traj.joint_names
        goal_msg.trajectory.points = traj.points
        
        # future.add_done_callback(self.goal_response_callback) 
        self.linear_action_client.wait_for_server()              

        self.linear_action_send_goal_future = self.linear_action_client.send_goal_async(
            goal_msg)
        
        self._floor_robot_send_goal_future.add_done_callback(
            self.floor_robot_goal_response_callback)

    def move_ceiling_robot_home(self, move_time):
        point = JointTrajectoryPoint()
        point.positions = self.ceiling_robot_home_joint_positions
        point.time_from_start = Duration(seconds=move_time).to_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.ceiling_robot_joint_names
        goal_msg.trajectory.points.append(point)

        self._ceiling_robot_action_client.wait_for_server()

        self._ceiling_robot_send_goal_future = self._ceiling_robot_action_client.send_goal_async(
            goal_msg)

        self._ceiling_robot_send_goal_future.add_done_callback(
            self.ceiling_robot_goal_response_callback)

    def floor_robot_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._floor_robot_get_result_future = goal_handle.get_result_async()
        self._floor_robot_get_result_future.add_done_callback(
            self.floor_robot_get_result_callback)
    
    def ceiling_robot_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._ceiling_robot_get_result_future = goal_handle.get_result_async()
        self._ceiling_robot_get_result_future.add_done_callback(
            self.ceiling_robot_get_result_callback)

    def floor_robot_get_result_callback(self, future):
        result = future.result().result
        result: FollowJointTrajectory.Result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Move succeeded")
            self.Success_done=True
        else:
            self.get_logger().error(result.error_string)

        self.floor_robot_at_home = True
    
    def ceiling_robot_get_result_callback(self, future):
        result = future.result().result
        result: FollowJointTrajectory.Result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Move succeeded")
        else:
            self.get_logger().error(result.error_string)

        self.ceiling_robot_at_home = True
        
    def ChangeGripper(self, station, gripper_type):
        self.kitting_robot_init("bin_agv_insert_joint")

        self.move_to(station)    
        target_matrix = self.FrameWorldPose(station +"_tool_changer_" + gripper_type + "_frame")
        position,rpy = Matrix2Pos_rpy(target_matrix)
        position[0]=-position[0]
        position[1]=-(position[1]+kitting_robot_park_location[station][1])
        position[2]=position[2]-0.035
        target_matrix = Rot2Matrix(self.init_rotation, position)

        p1 = copy.deepcopy(target_matrix)
        p2 = copy.deepcopy(target_matrix)
        p1[2,3]=p1[2,3]+0.1
        self.MOV_M(p1)
        self.MOV_M(p2)
        
    def kitting_robot_init(self, state,Wait_flag = True): 
        while not self.kitting_arm_joint_states and rclpy.ok():
            self.wait(0.1)
            print('waiting for initialization ...')
        # 复位
        init_position = copy.deepcopy(self.kitting_typical_joints[state])
        self.MOV_A(init_position, eps =0.02,sleep_flag = Wait_flag)
        print ('initialization success !')
        return True


        
    def MOV_A(self, target_joint_angles,time_from_start = 0.0,eps = 0.01, sleep_flag=True):         #输入的是关节角度向量Angle，直接控制运动
        '''
        time_from_start,默认按照最快时间执行
        eps = 0.0025
        '''
        q_begin = copy.deepcopy(self.kitting_arm_joint_states)
        q_end = target_joint_angles
        delta = [abs(q_begin[i] - q_end[i]) for i in range(0,len(q_begin))]
        if time_from_start == 0.0:
            time_from_start = max(delta)/kitting_angle_velocity
        #generate trajectory
        traj = traj_generate(self.kitting_arm_joint_names,q_begin,q_end,time_from_start)
        ##print traj

        self.move_floor_robot(traj)
        #self.wait(time_from_start)


    def move_to(self, location,eps = 0.02, flip = False):

        end_point =-kitting_robot_park_location[location][1]
        q_begin = self.kitting_base_y 
        distance = abs(q_begin - end_point)
        move_time = distance/kitting_velocity
        q_begin = [q_begin]
        q_end= [end_point]   
        traj = traj_generate(self.linear_joint_names,q_begin,q_end,move_time)
        self.move_linear_robot(traj)
        #self.wait(move_time)

        # self.robot_info.location = location

    def MOV_M(self, target_matrix,eps = 0.005,flip_flag=False):  #input:Matrix
        q_begin = copy.deepcopy(self.kitting_arm_joint_states)

        target = IKinematic(target_matrix,q_begin,A_k)
        if target==None:
            return False
        # #print "IKinematic_result:",target
        if flip_flag:
            target[-1] = q_begin[-1]
        # get max time
        delta = [abs(q_begin[i] - target[i]) for i in range(0,len(q_begin))]
        ##print delta
        distance = max(delta)
        time_from_start = distance/kitting_angle_velocity
        traj = traj_generate(self.kitting_arm_joint_names,q_begin,target,time_from_start)
        self.move_floor_robot(traj)
        #self.wait(time_from_start)

        return True

    def FrameWorldPose(self,frame_id):
        t =TransformStamped()
        pose = Pose()

        try:
            t= self.tf_buffer.lookup_transform("slide_bar", frame_id,  rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform assembly station  to world: {ex}')
            return

        print("---------",t)
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation

        target = Pose2Matrix(pose)
        return target      

    def wait(self, duration):
        start = self.get_clock().now()
        
        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                raise KeyboardInterrupt


