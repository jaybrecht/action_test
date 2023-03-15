#!/usr/bin/env python3

import rclpy
from collections import deque
import threading
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Duration
from rclpy.parameter import Parameter
from ariac_msgs.msg import *

from std_srvs.srv import Trigger
from ariac_msgs.srv import *
from tf2_ros import TransformException
from action_test.data import *
from geometry_msgs.msg import Pose,PoseStamped,Vector3,Quaternion
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_multiply, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointControllerState,JointTrajectoryControllerState
from action_test.Kinematics import *
from action_test.interpolation import *
from tf_transformations import *
from action_test.robot_control import RobotControl


class Subtask:
    def __init__(self, order_id, product_type, is_last_subtask):
        self.order_id = order_id
        self.is_done = False
        self.is_flip = False
        self.product_type = product_type
        self.is_last_subtask = is_last_subtask

class KittingSubtask(Subtask):
    def __init__(self, order_id, agv_number, tray_id, destination, product_type,product_quadrant, is_last_subtask):
        super().__init__(order_id, product_type, is_last_subtask)
        self.agv_number = agv_number
        self.tray_id = tray_id
        self.destination = destination
        self.product_quadrant = product_quadrant

class AssemblySubtask(Subtask):
    def __init__(self, order_id, agv_numbers, station, product_type, is_last_subtask,assembled_pose,install_direction,grap_pose):
        super().__init__(order_id, product_type, is_last_subtask)
        self.agv_numbers =agv_numbers
        self.station = station
        self.assembled_pose=assembled_pose
        self.install_direction=install_direction
        self.grap_pose=grap_pose
        


class CombinedSubtask(Subtask):
    def __init__(self, order_id, station, product_type,product_quadrant, is_last_subtask):
        super().__init__(order_id, product_type,product_quadrant, is_last_subtask)
        self.station = station


class CompetitionInterface(Node):
    # Dictionary to convert competition_state constants to strings
    states_ = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }

   
    def __init__(self,robot:RobotControl):
        super().__init__('competition_node')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        self.competition_state = None

        self.subscription = self.create_subscription(
            CompetitionState, 
            '/ariac/competition_state',
            self.competition_state_cb,
            10)
        
        self.floor=robot
        # self.ceiling=ceiling
        
        self.starter = self.create_client(Trigger, '/ariac/start_competition')
        self.pre_assembly_poses_getter = self.create_client(GetPreAssemblyPoses, '/ariac/get_pre_assembly_poses')
     
    ## ###################################### Order  ###############################################################################
            # 定义子任务列表
        self.kitting_subtasks = []
        self.assembly_subtasks = []
        self.combined_subtasks = []
        self.order_listsub_ = self.create_subscription(Order,'/ariac/orders',self.process_orders,10)
        self.order_listsub_       # 防止被python垃圾回收机制自动释放
        
        self.client = self.create_client(SubmitOrder, '/ariac/submit_order')
        self.quality_checker = self.create_client(PerformQualityCheck, '/ariac/perform_quality_check')
        self.orders = {"kitting_subtasks": deque(), "assembly_subtasks": deque(), "combined_subtasks": deque()}
        self.agv_has_tray=[False,False,False,False]
        self.floor_Drop=False



   ## ###################################### Sensor  ###############################################################################       
        self.AGV_location={
            'agv1': 'agv1_ks1_tray',
            'agv2': 'agv2_ks2_tray',
            'agv3': 'agv3_ks3_tray',
            'agv4': 'agv4_ks4_tray',
        }
        self.logical_camera_conveyor_parts = []

        
        self.has_blocked = False
        self.has_blocked_for_check = False
        
        self.tray_table_1 = []
        self.tray_table_2 = []

        #logical_camera_0_parts
        self.bin1_parts = []
        self.bin4_parts = []
        self.agv1_ks1_tray_parts = []

        #logical_camera_1_parts
        self.bin2_parts = []
        self.bin3_parts = []
        self.agv2_ks2_tray_parts = []

        #logical_camera_2_parts
        self.bin6_parts = []
        self.bin7_parts = []
        self.agv3_ks3_tray_parts = []

        #logical_camera_3_parts
        self.bin5_parts = []
        self.bin8_parts = []
        self.agv4_ks4_tray_parts = []

        self.new_part_dict = {
            'agv1_ks1_tray':None,
            'agv2_ks2_tray':None,
            'agv3_ks3_tray':None,
            'agv4_ks4_tray':None,

            'agv1_as1_tray':None,
            'agv2_as1_tray':None,
            'agv3_as3_tray':None,
            'agv4_as3_tray':None,

            'agv1_as2_tray':None,
            'agv2_as2_tray':None,
            'agv3_as4_tray':None,
            'agv4_as4_tray':None,
            'bin1':None,
            'bin2':None,
            'bin3':None,
            'bin4':None,
            'bin5':None,
            'bin6':None,
            'bin7':None,
            'bin8':None,
            'conveyor':None,
        }

        self.new_part_flag_dict = {
            'agv1_ks1_tray':False,
            'agv2_ks2_tray':False,
            'agv3_ks3_tray':False,
            'agv4_ks4_tray':False,

            'agv1_as1_tray':False,
            'agv2_as1_tray':False,
            'agv3_as3_tray':False,
            'agv4_as3_tray':False,

            'agv1_as2_tray':False,
            'agv2_as2_tray':False,
            'agv3_as4_tray':False,
            'agv4_as4_tray':False,

            'bin1':False,
            'bin2':False,
            'bin3':False,
            'bin4':False,
            'bin5':False,
            'bin6':False,
            'bin7':False,
            'bin8':False,
            'conveyor':False,
        }

        self.logical_camera_as_11_parts = []
        self.logical_camera_as_12_parts = []
        self.logical_camera_as_21_parts = []
        self.logical_camera_as_22_parts = []
        self.logical_camera_as_33_parts = []
        self.logical_camera_as_34_parts = []
        self.logical_camera_as_43_parts = []
        self.logical_camera_as_44_parts = []

        self.logical_camera_conveyor_parts = []
        self.logical_camera_update_rate = 0.1
        
        self.heart_beat = self.get_clock().now()
        self.heart_beat = self.heart_beat.nanoseconds/1e9

        self.u_id_count = 0
        self.AGV_state={
            'agv1': 'init',
            'agv2': 'init',
            'agv3': 'init',
            'agv4': 'init',
        }


        self.assembly_parts = []
        self.conveyor_parts = []
        self.parts_type_dict={}
        self.parts_location_dict={}
        self.parts_on_conveyor_dict={}

        self.kts1_camera_flag = False
        self.kts2_camera_flag = False

        self.update_flag   = False
        self.camera_0_flag = False
        self.camera_1_flag = False
        self.camera_2_flag = False
        self.camera_3_flag = False
        self.camera_4_flag = False

        self.camera_as_11_flag = False
        self.camera_as_12_flag = False
        self.camera_as_21_flag = False
        self.camera_as_22_flag = False
        self.camera_as_33_flag = False
        self.camera_as_34_flag = False
        self.camera_as_43_flag = False
        self.camera_as_44_flag = False

        self.AGV1_location_flag = False
        self.AGV2_location_flag = False
        self.AGV3_location_flag = False
        self.AGV4_location_flag = False

        self.AGV1_state_flag = False
        self.AGV2_state_flag = False
        self.AGV3_state_flag = False
        self.AGV4_state_flag = False

        self.parts_on_conveyor_dict={}

        self.threadLock = threading.Lock()
        
        qos_profile = qos_profile_sensor_data
        self.kts1_camera_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/kts1_camera/image',self.kts1_camera_cb,qos_profile_sensor_data)
        self.kts2_camera_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/kts2_camera/image',self.kts2_camera_cb,qos_profile_sensor_data)
        self.logical_camera_0_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_0/image',self.logical_camera_0_callback,qos_profile_sensor_data)
        self.logical_camera_1_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_1/image',self.logical_camera_1_callback,qos_profile_sensor_data)
        self.logical_camera_2_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_2/image',self.logical_camera_2_callback,qos_profile_sensor_data)
        self.logical_camera_3_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_3/image',self.logical_camera_3_callback,qos_profile_sensor_data)
        
        self.logical_camera_as_11_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station1_1/image',self.logical_camera_as_11_callback,qos_profile_sensor_data)
        self.logical_camera_as_12_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station2_1/image',self.logical_camera_as_21_callback,qos_profile_sensor_data)
        self.logical_camera_as_21_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station1_2/image',self.logical_camera_as_12_callback,qos_profile_sensor_data)
        self.logical_camera_as_22_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station2_2/image',self.logical_camera_as_22_callback,qos_profile_sensor_data)
        self.logical_camera_as_33_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station3_3/image',self.logical_camera_as_33_callback,qos_profile_sensor_data)
        self.logical_camera_as_34_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station4_3/image',self.logical_camera_as_43_callback,qos_profile_sensor_data)
        self.logical_camera_as_43_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station3_4/image',self.logical_camera_as_34_callback,qos_profile_sensor_data)
        self.logical_camera_as_44_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_station4_4/image',self.logical_camera_as_44_callback,qos_profile_sensor_data)

        self.logical_camera_conveyor_sub_ = self.create_subscription(AdvancedLogicalCameraImage,'/ariac/sensors/logical_camera_conveyor/image',self.logical_camera_conveyor_callback,qos_profile_sensor_data) 

        self.AGV1_status_sub_ = self.create_subscription(AGVStatus, "/ariac/agv1_status", self.AGV1_status_callback,qos_profile_sensor_data)
        self.AGV2_status_sub_ = self.create_subscription(AGVStatus, "/ariac/agv2_status", self.AGV2_status_callback,qos_profile_sensor_data)
        self.AGV3_status_sub_ = self.create_subscription(AGVStatus, "/ariac/agv3_status", self.AGV3_status_callback,qos_profile_sensor_data)
        self.AGV4_status_sub_ = self.create_subscription(AGVStatus, "/ariac/agv4_status", self.AGV4_status_callback,qos_profile_sensor_data)


    def competition_state_cb(self, msg: CompetitionState):
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {self.states_[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    def start_competition(self):
        self.get_logger().info('Waiting for competition to be ready')

        # Wait for competition to be ready
        while (self.competition_state != CompetitionState.READY):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return
        
        self.get_logger().info('Competition is ready. Starting...')

        # Call ROS service to start competition
        while not self.starter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/start_competition to be available...')

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self.starter.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().info('Unable to start competition')
    
        
    def wait(self, duration):
        start = self.get_clock().now()
        
        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
    
## ###################################### Order  ###############################################################################    
    def submit_order(self, order_id):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = SubmitOrder.Request()
        request.order_id = order_id

        future = self.client.call_async(request)
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            raise KeyboardInterrupt

        if future.result().success:
            self.get_logger().info(f'Submit order {order_id}')
        else:
            self.get_logger().warn('Unable Submit order')

    def perform_quality_check(self, order_id: str) -> bool:
        while not self.quality_checker .wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service quality_check not available, waiting again...')
        # Send quality check request
        request = PerformQualityCheck.Request()
        request.order_id = order_id
        future = self.quality_checker.call_async(request)
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            raise KeyboardInterrupt  
        
        if future.result().all_passed:
            self.get_logger().info(f'Check order {order_id} Well')
            return True,future.result()
        else:
            self.get_logger().warn('Check order False')   
            print("future.result()------------",future.result()) 
            return False, future.result()

        
    def process_orders(self,order_msg):
        print("order_msg",order_msg)
        # 解析订单信息
        order_id = order_msg.id
        order_type = order_msg.type
        order_priority = order_msg.priority
        order_kitting_task = order_msg.kitting_task
        order_assembly_task = order_msg.assembly_task
        order_combined_task = order_msg.combined_task

        # 将订单拆分成子任务
        if order_type == 0:
            for idx,part in enumerate(order_kitting_task.parts) :
                product_type=determine_part_name(part.part.type,part.part.color)
                product_quadrant=part.quadrant
                subtask = KittingSubtask(order_id, order_kitting_task.agv_number, order_kitting_task.tray_id, order_kitting_task.destination, product_type,product_quadrant, idx==len(order_kitting_task.parts)-1) 
                if order_priority:
                    self.orders["kitting_subtasks"].appendleft(subtask)
                else:
                    self.orders["kitting_subtasks"].append(subtask)                

        elif order_type == 1:
            # 2.移动指定的agv到指定的station
            for agv in order_assembly_task.agv_numbers:
                self.move_agv(agv,order_assembly_task.station)
                self.lock_agv_tray(agv)
            self.move_to_ceiling('as'+str(subtask.station))
            agv_part_pose=self.get_assembly_poses(subtask.order_id)
            for idx,part in enumerate(order_assembly_task.parts) :
                if agv_part_pose.part==part.part:
                    grap_pose=agv_part_pose.pose
                product_type=determine_part_name(part.part.type,part.part.color)
                subtask = AssemblySubtask(order_id, order_assembly_task.agv_numbers, order_assembly_task.station,product_type, idx==len(order_kitting_task.parts)-1,part.assembled_pose,part.install_direction,grap_pose) 
                if order_priority:
                    self.orders["assembly_subtasks"].appendleft(subtask)
                else:
                    self.orders["assembly_subtasks"].append(subtask)    
        elif order_type == 2:
            products = order_combined_task.parts
            subtasks = [CombinedSubtask(order_id, order_combined_task.station, product, idx==len(products)-1) for idx, product in enumerate(products)]
            # self.combined_subtasks.extend(subtasks)
        print("order_list",self.orders)
 
    def cmd_generate(self, order_list):
        while order_list["kitting_subtasks"]:
            print("order_list[kitting_subtasks]::::::",order_list["kitting_subtasks"],"len():::",len(order_list["kitting_subtasks"])) 
            subtask = order_list["kitting_subtasks"].popleft() 
            self.floor_kitting_operation(subtask)  
            # if self.ceiling.robot_info.is_idle:
            #     subtask = order_list["kitting_subtasks"].popleft() 
            #     if not self.floor.robot_info.is_idle:   # 需要开启线程
            #         t2=threading.Thread(target=self.ceiling_kitting_operation,args=(subtask,))
            #         t2.start()  
            #     else:  
            #         self.ceiling_kitting_operation(subtask)
            # self.orders["kitting_subtasks"].remove(subtask)  #自动匹配第一个相同的删除

                       
    def handle_quality_check_result(self,msg):
        if  msg.valid_id and (not msg.incorrect_tray):
            # 处理未通过质量检查的零件
            for i, quadrant in enumerate([msg.quadrant1, msg.quadrant2, msg.quadrant3, msg.quadrant4], start=1):
                if not quadrant.all_passed:
                    if quadrant.missing_part:
                        print(f"Missing part in quadrant {i}")
                    if quadrant.flipped_part:
                        print(f"Flipped part in quadrant {i}")
                        
                    if quadrant.faulty_part:
                        print(f"Faulty part in quadrant {i}")
                        
                    if quadrant.incorrect_part_type:
                        print(f"Incorrect part type in quadrant {i}")
                    if quadrant.incorrect_part_color:
                        print(f"Incorrect part color in quadrant {i}")
            return False
        else:
            # 处理不合法的工装托盘
            if not msg.valid_id:
                print("Invalid tray ID")
            if  msg.incorrect_tray:
                print("Incorrect tray for this order")
            return False

    def check_faluty_part(self,msg,quadrant_num):
        # 处理未通过质量检查的零件
        quadrants=[msg.quadrant1, msg.quadrant2, msg.quadrant3, msg.quadrant4]
        quadrant=quadrants[quadrant_num-1]
        if quadrant.faulty_part:
            print(f"Faulty part in quadrant {quadrant_num}")
            return True
        else:
            return False

    def floor_kitting_operation(self,subtask):
  
        ####### 1. Get part info
        target_part_type = subtask.product_type               
        target_part_list = self.search_part_on_bins(target_part_type)+self.search_part_on_conveyor(target_part_type)
        while not target_part_list:
            print("wait find part")
            target_part_list = self.search_part_on_bins(target_part_type)+self.search_part_on_conveyor(target_part_type)
            self.wait(1)
        if target_part_list:
            target_part = target_part_list[-1]
            print("target_part_list",target_part_list,"------target_part.location",target_part.location)
        else:
            print('Not found!')
            
        ####### 2.Do Kitting 
        if not self.agv_has_tray[subtask.agv_number-1]:
            tray=self.search_tray_by_tray_id(subtask.tray_id)
            self.floor.FloorRobotPickandPlaceTray(tray,subtask.agv_number)
            self.agv_has_tray[subtask.agv_number-1]=True
        # if target_part.location=="conveyor":
        #     self.floor.floor_grasp_on_conveyor(target_part,subtask.product_quadrant,subtask.agv_number)
        # else:
        #     self.floor.floor_grasp_on_bins(target_part,subtask)  
        
        # # print("subtask-----",subtask.is_last_subtask) 
        # if subtask.is_last_subtask:
        #     check_result,check_info=self.perform_quality_check(subtask.order_id)
        #     print("self.check_result-------",check_result,"check_info----------",check_info)
        #     if not check_result:
        #         self.handle_quality_check_result(check_info)
        #     else:
        #         self.move_agv(subtask.agv_number, subtask.destination)
        #         self.submit_order(subtask.order_id)        

        # self.threadLock.acquire()
        # self.floor.robot_info.is_idle=True
        # self.threadLock.release()
        
    # def ceiling_kitting_operation(self,subtask):
    #     self.threadLock.acquire()
    #     self.ceiling.robot_info.is_idle=False
    #     self.threadLock.release()
        
    #     ####### 1. Get part info
    #     target_part_type = subtask.product_type               
    #     target_part_list = self.search_part_on_bins(target_part_type)+self.search_part_on_conveyor(target_part_type)
    #     while not target_part_list:
    #         print("wait find part")
    #         target_part_list = self.search_part_on_bins(target_part_type)+self.search_part_on_conveyor(target_part_type)
    #         self.wait(1)
    #     if target_part_list:
    #         target_part = target_part_list[-1]
    #         print("target_part_list",target_part_list,"------target_part.location",target_part.location)
    #     else:
    #         print('Not found!')
            
    #     ####### 2.Do Kitting 
    #     # if not self.agv_has_tray[subtask.agv_number-1]:
    #     #     tray=self.search_tray_by_tray_id(subtask.tray_id)
    #     #     self.floor.FloorRobotPickandPlaceTray(tray,subtask.agv_number)
    #     #     self.agv_has_tray[subtask.agv_number-1]=True
    #     self.ceiling.gantry_robot_init()
    #     self.ceiling.move_to_ceiling(target_part.location)
    #     self.ceiling.pick_part_on_bin_agv_ceiling(target_part.location,target_part) 
    #     location='agv'+str(subtask.agv_number)+'_ks'+str(subtask.agv_number)+'_tray'   
    #     self.ceiling.ceiling_robot_place_part_on_kit_tray(location,subtask,target_part)
        
    #     self.threadLock.acquire()
    #     self.ceiling.robot_info.is_idle=True
    #     self.threadLock.release()
     
                    

    # def assembly_operation(self,subtask):    
    #         self.gantry_robot_init()
    #         # self.move_to_ceiling(target_part.location)                                ###########测试抓取和放置零件
    #         # self.pick_part_on_bin_agv_ceiling(target_part.location,target_part) 
    #         # location='agv'+str(subtask.agv_number)+'_ks'+str(subtask.agv_number)+'_tray'   
    #         # self.ceiling_robot_place_part_on_kit_tray(location,subtask,target_part)
    #         # if self.ceiling_robot_gripper_state.attached:
    #         #     self.ceiling_robot_place_part_on_kit_tray(location,subtask,target_part)
    #         # else:
    #         #     self.orders["kitting_subtasks"].appendleft(subtask)
    #         # self.move_to_ceiling("as1")
    #                                                                                 ###########测试换夹爪，无法实现
    #         # tray=self.search_tray_by_tray_id(subtask.tray_id)
    #         # self.ChangeGripper_ceiling(tray.location, "trays")
    #                                                                                     ###########测试assembly装配
    #         # 1.确保order指定的agv上有指定的零件，可以通过抓取实现，目前直接生成
            

            
    #         # 3.通过服务获取指定agv上零件的信息，并进行抓取

            

        
    def run(self):
        try:
            rclpy.spin_once(self)
        except KeyboardInterrupt:
            return
        self.cmd_generate(self.orders)

#region ## ###################################### Sensor  ###############################################################################      

    def left_bins_camera_cb(self, msg):
        if not self.left_bins_camera_recieved_data:
            self.get_logger().info("Received data from left bins camera")
            self.left_bins_camera_recieved_data = True

        self.left_bins_parts_ = msg.part_poses
        self.left_bins_camera_pose_ = msg.sensor_pose

    def right_bins_camera_cb(self, msg):
        if not self.right_bins_camera_recieved_data:
            self.get_logger().info("Received data from right bins camera")
            self.right_bins_camera_recieved_data = True

        self.right_bins_parts_ = msg.part_poses
        self.right_bins_camera_pose_ = msg.sensor_pose

    #零件列表更新 输入：某个零件类 零件列表 零件位置误差
    def parts_lsit_update(self, part, parts_list, part_move_skew = normal_part_move_skew):
        current_time = self.get_clock().now() 
        current_time=current_time.nanoseconds/1e9
        # print("current_time",current_time) 
        part.set_time_stamp(current_time)
        # part.pose.position.z = self.part_position_z_limit(part)
        in_list_flag = 0
        if parts_list:
            parts_list_len = len(parts_list)                     
            for parts_list_i in range(parts_list_len):
                if Part_Compare(part, parts_list[parts_list_i], part_move_skew):
                    if part.location == 'conveyor':
                        if abs(current_time - parts_list[parts_list_i].time_stamp) < 0.3:
                            part.final_check = parts_list[parts_list_i].final_check
                            part.u_id = parts_list[parts_list_i].u_id                           
                            parts_list[parts_list_i] = part
                            in_list_flag = 1
                            break
                    else:
                        part.u_id = parts_list[parts_list_i].u_id
                        part.final_check = parts_list[parts_list_i].final_check
                        last_msg_weight = 0.15
                        
                        part.pose.position.x = (1-last_msg_weight) * part.pose.position.x + last_msg_weight * parts_list[parts_list_i].pose.position.x
                        part.pose.position.y = (1-last_msg_weight) * part.pose.position.y + last_msg_weight * parts_list[parts_list_i].pose.position.y
                        part.pose.position.z = (1-last_msg_weight) * part.pose.position.z + last_msg_weight * parts_list[parts_list_i].pose.position.z
                        
                        part.pose.orientation.x = (1-last_msg_weight) * part.pose.orientation.x + last_msg_weight * parts_list[parts_list_i].pose.orientation.x
                        part.pose.orientation.y = (1-last_msg_weight) * part.pose.orientation.y + last_msg_weight * parts_list[parts_list_i].pose.orientation.y
                        part.pose.orientation.z = (1-last_msg_weight) * part.pose.orientation.z + last_msg_weight * parts_list[parts_list_i].pose.orientation.z
                        part.pose.orientation.w = (1-last_msg_weight) * part.pose.orientation.w + last_msg_weight * parts_list[parts_list_i].pose.orientation.w
                        parts_list[parts_list_i] = part
                        in_list_flag = 1
                        break 

        if not in_list_flag:
            if part.location == 'conveyor':
                # print("part.pose.position.y",part.pose.position.y)
                if part.pose.position.y >=3.9:#4.0
                    self.u_id_count = self.u_id_count +1
                    part.u_id = self.u_id_count
                    parts_list.append(part)
                    self.new_part_dict[part.location] = part
                    self.new_part_flag_dict[part.location] = True 
            else:
                self.u_id_count = self.u_id_count +1
                part.u_id = self.u_id_count
                parts_list.append(part)
                self.new_part_dict[part.location] = part
                self.new_part_flag_dict[part.location] = True 

    def kts1_camera_cb(self,msg):
        # print("kts1_camera_cb")
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.kts1_camera_flag = True
        if self.kts1_camera_flag:
            for model in msg.tray_poses:
                tray_id=model.id
                tray_name = "tray_"+str(tray_id)
                
                part_pose = Pose()              # part to world
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]  

                # print("tray_id",tray_id,"tray_pose",part_pose.position)

                #根据区域确定托盘是否在桌子上
                if  Define_tray_is_in_effective_table_range(tray_name, part_pose.position, tray_table_boundary['tray_table_1_x'],tray_table_boundary['tray_table_1_y'], tables_tray_hight):
                    part = sPart(tray_name,"tray_table_1",part_pose)
                    self.parts_lsit_update(part, self.tray_table_1)
                    # print(tray_name,"on tray_table_1")
                    continue

        self.kts1_camera_flag = False
        # print("total number1:%d", len(self.tray_table_1))
        # print("---------")

    def kts2_camera_cb(self,msg):
        # print("kts2_camera_cb")
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.kts2_camera_flag = True
        if self.kts2_camera_flag:
            for model in msg.tray_poses:
                tray_id=model.id
                tray_name = "tray_"+str(tray_id)
                
                part_pose = Pose()              # part to world
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]  

                # print("tray_id",tray_id,"tray_pose",part_pose.position)

                #根据区域确定托盘是否在桌子上
                if  Define_tray_is_in_effective_table_range(tray_name, part_pose.position, tray_table_boundary['tray_table_2_x'],tray_table_boundary['tray_table_2_y'], tables_tray_hight):
                    part = sPart(tray_name,"tray_table_2",part_pose)
                    self.parts_lsit_update(part, self.tray_table_2)
                    # print(tray_name,"on tray_table_2")
                    continue

        self.kts2_camera_flag = False
        # print("total number1:%d", len(self.tray_table_2))
        # print("---------")


    def AGV1_status_callback(self,msg):
        self.AGV1_location_flag = True
        if self.AGV1_location_flag:
            agv1_location = msg.location
            if agv1_location == 0:
                self.AGV_location['agv1'] = 'agv1_ks1_tray'
            if agv1_location == 1:
                self.AGV_location['agv1'] = 'agv1_as1_tray'
            if agv1_location == 2:
                self.AGV_location['agv1'] = 'agv1_as2_tray'
            if agv1_location == 99: #正在运动
                self.AGV_location['agv1'] = "moving"
                
             
            self.AGV1_location_flag = False 
    
    def AGV2_status_callback(self,msg):
        self.AGV2_location_flag = True
        if self.AGV2_location_flag:
            agv2_location = msg.location
            if agv2_location == 0:
                self.AGV_location['agv2'] = 'agv2_ks2_tray'
            if agv2_location == 1:
                self.AGV_location['agv2'] = 'agv2_as1_tray'
            if agv2_location == 2:
                self.AGV_location['agv2'] = 'agv2_as2_tray'
            if agv2_location == 99: #正在运动
                self.AGV_location['agv2'] = "moving"
             
            self.AGV1_location_flag = False 

    def AGV3_status_callback(self,msg):
        self.AGV3_location_flag = True
        if self.AGV3_location_flag:
            agv3_location = msg.location
            if agv3_location == 0:
                self.AGV_location['agv3'] = 'agv3_ks3_tray'
            if agv3_location == 1:
                self.AGV_location['agv3'] = 'agv3_as3_tray'
            if agv3_location == 2:
                self.AGV_location['agv3'] = 'agv3_as4_tray'
            if agv3_location == 99: #正在运动
                self.AGV_location['agv3'] = "moving"
             
            self.AGV1_location_flag = False 

    def AGV4_status_callback(self,msg):
        self.AGV4_location_flag = True
        if self.AGV4_location_flag:
            agv4_location = msg.location
            if agv4_location == 0:
                self.AGV_location['agv4'] = 'agv4_ks4_tray'
            if agv4_location == 1:
                self.AGV_location['agv4'] = 'agv4_as3_tray'
            if agv4_location == 2:
                self.AGV_location['agv4'] = 'agv4_as4_tray'
            if agv4_location == 99: #正在运动
                self.AGV_location['agv4'] = "moving"
             
            self.AGV1_location_flag = False 


    def logical_camera_0_callback(self,msg):  
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9      
        self.camera_0_flag = True
        if self.camera_0_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                part_pose = Pose()              # part to world
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]  
                
                # print("part_type1",part_color_type,"part_pose",part_pose.position)

                
                
                # 根据区域决定物体在bin1或者bin4或agv1上  ？？？？？
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin1_x"] ,bins_ks_boundary["bin1_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin1",part_pose)
                    self.parts_lsit_update(part, self.bin1_parts)
                    # print(part_color_type,"on bin1_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin4_x"] ,bins_ks_boundary["bin4_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin4",part_pose)
                    self.parts_lsit_update(part, self.bin4_parts)
                    #print(part_color_type,"on bin4_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_ks_boundary["agv1_x"] ,agv_ks_boundary["agv1_y"], agv_product_height_with_tray_flip) :
                # and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                    part = sPart(part_color_type,"agv1_ks1_tray",part_pose)
                    self.parts_lsit_update(part, self.agv1_ks1_tray_parts)
                    #print(part_color_type,"on agv1_y")
                    continue
            self.camera_0_flag =False
            #print("bin1_parts:")
            # print("total number1:%d", len(self.bin1_parts))
            # print("total number4:%d", len(self.bin4_parts))
            # print("total number agv1:%d", len(self.agv1_ks1_tray_parts))

    def logical_camera_1_callback(self,msg):         
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_1_flag = True
        if self.camera_1_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                part_pose = Pose()              # part to world
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]  
                
                # print("part_type1",part_color_type,"part_pose",part_pose.position)

                
                
                # to determin the location
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin2_x"] ,bins_ks_boundary["bin2_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin2",part_pose)
                    self.parts_lsit_update(part, self.bin2_parts)
                    #print(part_color_type,"on bin2_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin3_x"] ,bins_ks_boundary["bin3_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin3",part_pose)
                    self.parts_lsit_update(part, self.bin3_parts)
                    #print(part_color_type,"on bin3_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_ks_boundary["agv2_x"] ,agv_ks_boundary["agv2_y"], agv_product_height_with_tray_flip) :
                # and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                    part = sPart(part_color_type,"agv2_ks2_tray",part_pose)
                    self.parts_lsit_update(part, self.agv2_ks2_tray_parts)
                    #print(part_color_type,"on agv2_y")
                    continue
            self.camera_1_flag =False
            # print("total number2:%d", len(self.bin2_parts))
            # print("total number3:%d", len(self.bin3_parts))
            # print("total number agv2:%d", len(self.agv2_ks2_tray_parts))

    def logical_camera_2_callback(self,msg):  
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_2_flag = True
        if self.camera_2_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                part_pose = Pose()              # part to world
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]  
                
                # print("part_type1",part_color_type,"part_pose",part_pose.position)

                
                
                # to determin the location
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin6_x"] ,bins_ks_boundary["bin6_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin6",part_pose)
                    self.parts_lsit_update(part, self.bin6_parts)
                    #print(part_color_type,"on bin6_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin7_x"] ,bins_ks_boundary["bin7_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin7",part_pose)
                    self.parts_lsit_update(part, self.bin7_parts)
                    #print(part_color_type,"on bin7_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_ks_boundary["agv3_x"] ,agv_ks_boundary["agv3_y"], agv_product_height_with_tray_flip) :
                # and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                    part = sPart(part_color_type,"agv3_ks3_tray",part_pose)
                    self.parts_lsit_update(part, self.agv3_ks3_tray_parts)
                    #print(part_color_type,"on agv3_y")
                    continue
            self.camera_2_flag =False
            # print("total number6:%d", len(self.bin6_parts))
            # print("total number7:%d", len(self.bin7_parts))
            # print("total number agv3:%d", len(self.agv3_ks3_tray_parts))

    def logical_camera_3_callback(self,msg):  
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_3_flag = True
        if self.camera_3_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                part_pose = Pose()              # part to world
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]  
                
                # print("part_type1",part_color_type,"part_pose",part_pose.position)

                
                
                # to determin the location
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin5_x"] ,bins_ks_boundary["bin5_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin5",part_pose)
                    self.parts_lsit_update(part, self.bin5_parts)
                    # print(part_color_type,"on bin5_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, bins_ks_boundary["bin8_x"] ,bins_ks_boundary["bin8_y"], bins_product_height_flip):
                    part = sPart(part_color_type,"bin8",part_pose)
                    self.parts_lsit_update(part, self.bin8_parts)
                    #print(part_color_type,"on bin8_y")
                    continue
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_ks_boundary["agv4_x"] ,agv_ks_boundary["agv4_y"], agv_product_height_with_tray_flip) :
                # and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                    part = sPart(part_color_type,"agv4_ks4_tray",part_pose)
                    self.parts_lsit_update(part, self.agv4_ks4_tray_parts)
                    #print(part_color_type,"on agv4_y")
                    continue
            self.camera_3_flag =False
            # print("total number5:%d", len(self.bin5_parts))
            # print("total number8:%d", len(self.bin8_parts))
            # print("total number agv4:%d", len(self.agv4_ks4_tray_parts))

    def logical_camera_conveyor_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        for model in msg.part_poses:
            part_type = model.part.type
            part_color=model.part.color           
            part_color_type=determine_part_name(part_type,part_color)
            
            # frame_to_word
            part_pose = Pose()
            part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
            part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
            part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x
            
            # part_pose.position.z = part_on_conveyor_z[part_color_type]

            p = quaternion_multiply( \
                [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
            part_pose.orientation.x = p[0]
            part_pose.orientation.y = p[1]
            part_pose.orientation.z = p[2]
            part_pose.orientation.w = p[3]
                # to determin the location
            if  Is_In_Effective_Range(part_color_type, part_pose.position, convey_boundary["convey_x"] ,convey_boundary["convey_y"], convey_product_height_flip):
                part = sPart(part_color_type,"conveyor",part_pose)
                self.parts_lsit_update(part, self.logical_camera_conveyor_parts, conveyor_part_move_skew)
        #     print("part_color_type",part_color_type,"part_pose",part_pose.position)
        # print("total number CONVEYOR:%d", len(self.logical_camera_conveyor_parts))
        # print("-------------------------")

    def logical_camera_as_11_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_11_flag = True
        if self.camera_as_11_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv1_as1_x"], agv_as_boundary['agv1_as1_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv1'] == 'agv1_as1_tray':
                    part = sPart(part_color_type,"agv1_as1_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_11_parts)
                    #print(part_color_type,"on as_11_parts")
                    continue
            self.camera_as_11_flag = False           
            #print("total number as11:%d", len(self.logical_camera_as_11_parts))
            
        else:
            pass  

    def logical_camera_as_21_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_21_flag = True
        if self.camera_as_21_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
                
                

                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv1_as2_x"], agv_as_boundary['agv1_as2_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv1'] == 'agv1_as2_tray':
                    part = sPart(part_color_type,"agv1_as2_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_21_parts)
                    #print(part_color_type,"on as_21_parts")
                    continue
            self.camera_as_21_flag = False           
            #print("total number as21:%d", len(self.logical_camera_as_21_parts))
            
        else:
            pass  

    def logical_camera_as_12_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_12_flag = True
        if self.camera_as_12_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv2_as1_x"], agv_as_boundary['agv2_as1_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv2'] == 'agv2_as1_tray':
                    part = sPart(part_color_type,"agv2_as1_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_12_parts)
                    #print(part_color_type,"on as_12_parts")
                    continue
            self.camera_as_12_flag = False           
            #print("total number as12:%d", len(self.logical_camera_as_12_parts))
            
        else:
            pass  

    def logical_camera_as_22_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_22_flag = True
        if self.camera_as_22_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv2_as2_x"], agv_as_boundary['agv2_as2_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv2'] == 'agv2_as2_tray':
                    part = sPart(part_color_type,"agv2_as2_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_22_parts)
                    #print(part_color_type,"on as_22_parts")
                    continue
            self.camera_as_22_flag = False           
            #print("total number as22:%d", len(self.logical_camera_as_22_parts))
            
        else:
            pass  

    def logical_camera_as_33_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_33_flag = True
        if self.camera_as_33_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv3_as3_x"], agv_as_boundary['agv3_as3_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv3'] == 'agv3_as3_tray':
                    part = sPart(part_color_type,"agv3_as3_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_33_parts)
                    # print(part_color_type,"on as_33_parts")
                    continue
            self.camera_as_33_flag = False           
            # print("total number as33:%d", len(self.logical_camera_as_33_parts))
            
        else:
            pass  

    def logical_camera_as_43_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_43_flag = True
        if self.camera_as_43_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv3_as4_x"], agv_as_boundary['agv3_as4_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv3'] == 'agv3_as4_tray':
                    part = sPart(part_color_type,"agv3_as4_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_43_parts)
                    #print(part_color_type,"on as_43_parts")
                    continue
            self.camera_as_43_flag = False           
            #print("total number as43:%d", len(self.logical_camera_as_43_parts))
            
        else:
            pass  

    def logical_camera_as_34_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_34_flag = True
        if self.camera_as_34_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv4_as3_x"], agv_as_boundary['agv4_as3_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv4'] == 'agv4_as3_tray':
                    part = sPart(part_color_type,"agv4_as3_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_34_parts)
                    #print(part_color_type,"on as_34_parts")
                    continue
            self.camera_as_34_flag = False           
            #print("total number as34:%d", len(self.logical_camera_as_34_parts))
            
        else:
            pass  

    def logical_camera_as_44_callback(self,msg):
        self.heart_beat = self.get_clock().now()
        self.heart_beat=self.heart_beat.nanoseconds/1e9
        self.camera_as_44_flag = True
        if self.camera_as_44_flag:
            for model in msg.part_poses:
                part_type=model.part.type
                part_color=model.part.color
                part_color_type=determine_part_name(part_type,part_color)
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.sensor_pose.position.x+model.pose.position.z
                part_pose.position.y = msg.sensor_pose.position.y+model.pose.position.y
                part_pose.position.z = msg.sensor_pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.sensor_pose.orientation.x, msg.sensor_pose.orientation.y,msg.sensor_pose.orientation.z,msg.sensor_pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                # if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                #     part = sPart(part_type,"kit_tray_station",part_pose)
                #     self.AGV1_movable_tray = part
                # else:                
                # to determin the location
            
                if  Is_In_Effective_Range(part_color_type, part_pose.position, agv_as_boundary["agv4_as4_x"], agv_as_boundary['agv4_as4_y'], agv_product_height_with_tray_flip) \
                 and self.AGV_location['agv4'] == 'agv4_as4_tray':
                    part = sPart(part_color_type,"agv4_as4_tray",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_as_44_parts)
                    #print(part_color_type,"on as_44_parts")
                    continue
            self.camera_as_44_flag = False           
            #print("total number as44:%d", len(self.logical_camera_as_44_parts))
            
        else:
            pass  



    #通过零件的id删除该零件  输入：零件列表 零件类
    def search_del_part_use_id(self,parts_list, part):
        list_len = len(parts_list)
        for list_count in range(list_len):
            if parts_list[list_count].u_id == part.u_id:
                del parts_list[list_count]
                break
    
    #删除某个零件 输入：需要删除的零件类
    def del_part_from_parts_list(self, del_part):
        if del_part.location == 'bin1':
            self.search_del_part_use_id(self.bin1_parts, del_part)
        elif del_part.location == 'bin2':
            self.search_del_part_use_id(self.bin2_parts, del_part)
        elif del_part.location == 'bin3':
            self.search_del_part_use_id(self.bin3_parts, del_part)
        elif del_part.location == 'bin4':
            self.search_del_part_use_id(self.bin4_parts, del_part)            
        elif del_part.location == 'bin5':
            self.search_del_part_use_id(self.bin5_parts, del_part)            
        elif del_part.location == 'bin6':
            self.search_del_part_use_id(self.bin6_parts, del_part)            
        elif del_part.location == 'bin7':
            self.search_del_part_use_id(self.bin7_parts, del_part)            
        elif del_part.location == 'bin8':
            self.search_del_part_use_id(self.bin8_parts, del_part)
        
        elif del_part.location == 'agv1_ks1_tray':
            self.search_del_part_use_id(self.agv1_ks1_tray_parts, del_part)
        elif del_part.location == 'agv2_ks2_tray':
            self.search_del_part_use_id(self.agv2_ks2_tray_parts, del_part)                  
        elif del_part.location == 'agv3_ks3_tray':
            self.search_del_part_use_id(self.agv3_ks3_tray_parts, del_part)            
        elif del_part.location == 'agv4_ks4_tray':
            self.search_del_part_use_id(self.agv4_ks4_tray_parts, del_part) 

        elif del_part.location == 'agv1_as1_tray':
            self.search_del_part_use_id(self.logical_camera_as_11_parts, del_part)
        elif del_part.location == 'agv1_as2_tray':
            self.search_del_part_use_id(self.logical_camera_as_21_parts, del_part)
        elif del_part.location == 'agv2_as1_tray':
            self.search_del_part_use_id(self.logical_camera_as_12_parts, del_part)
        elif del_part.location == 'agv2_as2_tray':
            self.search_del_part_use_id(self.logical_camera_as_22_parts, del_part)            
        elif del_part.location == 'agv3_as3_tray':
            self.search_del_part_use_id(self.logical_camera_as_33_parts, del_part)
        elif del_part.location == 'agv3_as4_tray':
            self.search_del_part_use_id(self.logical_camera_as_43_parts, del_part)
        elif del_part.location == 'agv4_as3_tray':
            self.search_del_part_use_id(self.logical_camera_as_34_parts, del_part)
        elif del_part.location == 'agv4_as4_tray':
            self.search_del_part_use_id(self.logical_camera_as_44_parts, del_part)
        
        elif del_part.location == 'conveyor':
            self.search_del_part_use_id(self.logical_camera_conveyor_parts, del_part)  
        
        elif del_part.location == 'tray_table_1':
            self.search_del_part_use_id(self.tray_table_1, del_part)
        elif del_part.location == 'tray_table_2':
            self.search_del_part_use_id(self.tray_table_2, del_part)

    #搜索在传送带上的零件通过颜色类型  输入：零件颜色类型 assembly_battery_red
    def search_part_on_conveyor(self,part_color_type):
        # clear 
        self.parts_on_conveyor_dict.clear()
        if len(self.logical_camera_conveyor_parts)>=1:
            print ("parts on conveyor:",len(self.logical_camera_conveyor_parts))
            # if time out then delete
            current_time = self.get_clock().now()
            current_time=current_time.nanoseconds/1e9
            flag = False
            for part in self.logical_camera_conveyor_parts:
                if (current_time - part.time_stamp)*conveyor_vel > 6.5:
                    self.del_part_from_parts_list(part)

            effective_parts =[]
            for part in self.logical_camera_conveyor_parts:
                if (current_time - part.time_stamp)*conveyor_vel < 6.5:
                    effective_parts.append(part)

            for part in effective_parts:
                self.parts_on_conveyor_dict.setdefault(part.type,[]).append(part)
                    
            if part_color_type in self.parts_on_conveyor_dict.keys():
                # print "parts_need number:",len(self.parts_on_conveyor_dict[part_type])
                return self.parts_on_conveyor_dict[part_color_type]
            else:
                return []
        else:
            return []

    #搜索某个零件 输入：需要搜索的零件类
    def search_part_use_part(self, part):
        self.all_parts_list_old_update()

        parts_list = self.bin1_parts + self.bin4_parts + self.agv1_ks1_tray_parts + \
            self.bin2_parts + self.bin3_parts + self.agv2_ks2_tray_parts + \
            self.bin6_parts + self.bin7_parts + self.agv3_ks3_tray_parts + \
            self.bin5_parts + self.bin8_parts + self.agv4_ks4_tray_parts + \
            self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
            self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
            self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
            self.logical_camera_as_43_parts + self.logical_camera_as_44_parts 
        list_len = len(parts_list)
        for list_count in range(list_len):
            if parts_list[list_count].u_id == part.u_id:
                return parts_list[list_count]

    #传感器是否故障 输出：false故障了   true正常
    def is_alive(self):
        current_time = self.get_clock().now()       
        current_time=current_time.nanoseconds/1e9
        if current_time - self.heart_beat > 0.5:
            self.has_blocked = True
            self.has_blocked_for_check = True
            return False
        else:
            return True

    #更新环境中零件列表
    def all_parts_list_old_update(self):
        self.parts_list_old_update(self.bin1_parts)
        self.parts_list_old_update(self.bin2_parts)
        self.parts_list_old_update(self.bin3_parts)
        self.parts_list_old_update(self.bin4_parts)
        self.parts_list_old_update(self.bin5_parts)
        self.parts_list_old_update(self.bin6_parts)
        self.parts_list_old_update(self.bin7_parts)
        self.parts_list_old_update(self.bin8_parts)
        self.parts_list_old_update(self.agv1_ks1_tray_parts)
        self.parts_list_old_update(self.agv2_ks2_tray_parts)
        self.parts_list_old_update(self.agv3_ks3_tray_parts)
        self.parts_list_old_update(self.agv4_ks4_tray_parts)
        self.parts_list_old_update(self.tray_table_1)  
        self.parts_list_old_update(self.tray_table_2) 

    #更新某个容器中的零件列表  输入：某个箱子 self.bin1_parts
    def parts_list_old_update(self, parts_list, old_time = 1):

        
        while self.has_blocked and self.is_alive():
            self.wait(1)
            self.has_blocked = False

        if self.is_alive():
            current_time = self.get_clock().now()
            current_time=current_time.nanoseconds/1e9
            part_old_flag = True
            while part_old_flag:
                list_len = len(parts_list)
                part_old_flag = False
                self.threadLock.acquire()
                for list_count in range(0, list_len):
                    if list_count < len(parts_list) and parts_list[list_count].time_stamp:
                        if list_count < len(parts_list) and abs(current_time - parts_list[list_count].time_stamp) > old_time:
                            del parts_list[list_count]
                            part_old_flag = True
                            break
                self.threadLock.release()

    #零件类型分类 输出：字典key：零件颜色类型 vaule：零件
    def part_type_sort(self):
        self.all_parts_list_old_update()
        # clear 
        self.parts_type_dict.clear()
        # merge all_part

        all_parts = self.bin1_parts + self.bin4_parts + self.bin2_parts + self.bin3_parts +\
                    self.bin6_parts + self.bin7_parts + self.bin5_parts + self.bin8_parts + \
                    self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
                    self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
                    self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
                    self.logical_camera_as_43_parts + self.logical_camera_as_44_parts + \
                    self.agv1_ks1_tray_parts + self.agv2_ks2_tray_parts + self.agv3_ks3_tray_parts + self.agv4_ks4_tray_parts
        # print("self.bin1_parts",len(self.bin1_parts))
        # print("len(all_parts)",len(all_parts))
        if len(all_parts)>=1:
            # build dict
            for part in all_parts:
                self.parts_type_dict.setdefault(part.type,[]).append(part)
        else:
            pass

    #通过零件颜色类型搜索零件
    def search_part_type(self, part_color_type):
        self.part_type_sort()
        # print("self.parts_type_dict",len(self.parts_type_dict))
        if part_color_type in self.parts_type_dict.keys():
            return self.parts_type_dict[part_color_type]
        else:
            return []
        
    #零件类型分类 输出：字典key：零件颜色类型 vaule：零件
    def part_type_sort_on_bins(self):
        self.all_parts_list_old_update()
        # clear 
        self.parts_type_dict.clear()
        # merge all_part

        all_parts_on_bins = self.bin1_parts + self.bin4_parts + self.bin2_parts + self.bin3_parts +\
                    self.bin6_parts + self.bin7_parts + self.bin5_parts + self.bin8_parts + \
                    self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
                    self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
                    self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
                    self.logical_camera_as_43_parts + self.logical_camera_as_44_parts 
        # print("self.bin1_parts",len(self.bin1_parts))
        # print("len(all_parts)",len(all_parts))
        if len(all_parts_on_bins)>=1:
            # build dict
            for part in all_parts_on_bins:
                self.parts_type_dict.setdefault(part.type,[]).append(part)
        else:
            pass

    #通过零件颜色类型搜索零件
    def search_part_on_bins(self, part_color_type):
        self.part_type_sort_on_bins()
        # print("self.parts_type_dict",len(self.parts_type_dict))
        if part_color_type in self.parts_type_dict.keys():
            return self.parts_type_dict[part_color_type]
        else:
            return []
        
    #零件位置分类 输出：字典key：装零件的箱子 vaule：箱子上的零件
    def part_location_sort(self):
        self.all_parts_list_old_update()
        # clear 
        self.parts_location_dict.clear() 
        # merge all_part
        all_parts = self.bin1_parts + self.bin4_parts + self.bin2_parts + self.bin3_parts +\
                    self.bin6_parts + self.bin7_parts + self.bin5_parts + self.bin8_parts + \
                    self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
                    self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
                    self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
                    self.logical_camera_as_43_parts + self.logical_camera_as_44_parts + \
                    self.agv1_ks1_tray_parts + self.agv2_ks2_tray_parts + self.agv3_ks3_tray_parts + self.agv4_ks4_tray_parts
                    
        if len(all_parts)>=1:
            # build dict
            for part in all_parts:
                self.parts_location_dict.setdefault(part.location,[]).append(part)

            #print test    
            # for key,value in self.parts_location_dict.items():
            #     print(key+": ")
            #     for p in value:
            #         print(p.location)
        else:
            pass

    #通过位置搜索零件  输入“bin1”
    def search_part_location(self,part_location):

        self.part_location_sort()
        if part_location in self.parts_location_dict.keys():
            return self.parts_location_dict[part_location]
        else:
            return False

    #通过位置和零件颜色类型搜索零件
    def search_part_location_type(self,part_location,part_color_type): 
        """
        robot_system 使用
        """  
        self.part_location_sort()
        part_type_list = []
        if part_location in self.parts_location_dict.keys():
            part_list = self.parts_location_dict[part_location]
            if part_list:
                for part in part_list:
                    if part.type == part_color_type:
                        part_type_list.append(part)
                return part_type_list
            else:
              return False  
        else:
            return False     
    
    #托盘id---------托盘信息
    def search_tray_by_tray_id(self,tray_id):
        tray_=None
        # Check table 1
        for tray in self.tray_table_1:
            if tray.type == "tray_"+str(tray_id):
                tray_=tray
                tray_.location="kts1"
                return tray_

        for tray in self.tray_table_2:
            if tray.type == "tray_"+str(tray_id):
                tray_=tray
                tray_.location="kts2"
                return tray_
        print("tray_table_1------------",self.tray_table_1,"self.tray_table_2---------",self.tray_table_2)

        return False   

#endregion 
             
    def move_agv(self, agv_num, destination):
        service_name = '/ariac/move_agv{}'.format(agv_num)
        self.move_agv_clients= self.create_client(MoveAGV, service_name)

        while not self.move_agv_clients.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = MoveAGV.Request()
        request.location = destination

        future = self.move_agv_clients.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return False       

    def lock_agv_tray(self, agv_num: int) -> bool:
            srv_name = f'/ariac/agv{agv_num}_lock_tray'
            self.lock_agv_clients = self.create_client(Trigger, srv_name)

            request = Trigger.Request()
            # 异步发送请求
            future = self.lock_agv_clients .call_async(request)
            # 等待响应
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                return future.result().success
            else:
                self.get_logger().error('Exception while calling service: %r' % future.exception())
                return False              
    
    def get_assembly_poses(self, order_id: str) -> List[PartPose]:
        request = GetPreAssemblyPoses.Request()
        request.order_id = order_id

        future = self.pre_assembly_poses_getter.call_async(request)

        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        agv_part_poses = []
        if response.valid_id:
            agv_part_poses = response.parts

            if len(agv_part_poses) == 0:
                self.get_logger().warn('No part poses received')
                return False
        else:
            self.get_logger().warn('Not a valid order ID')
            return False

        return agv_part_poses            
                

