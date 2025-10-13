from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_ # idl
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_
from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Point32_,geometry_msgs_msg_dds__Twist_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.default import nav_msgs_msg_dds__Odometry_
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.get_logger(__name__)



kTopicHeightCmd = "rt/cmd_hispeed"
kTopicHeightState = "rt/hispeed_state"
kTopicG1MoveCmd = "rt/cmd_vel"
kTopicG1MoveState = "rt/slamware_ros_sdk_server_node/odom"

kTopicHeightAction = "rt/cmd_hispeed"
kTopicMoveAction = "rt/cmd_vel"

class G1_Mobilebase_Height_Controller:
    def __init__(self, init_type,control_type, fps = 30.0, Unit_Test = False, simulation_mode = False, filter_alpha=0.2):
        """
        初始化G1移动底盘和高度控制器
        
        参数:
            init_type: "only_height"只高度, "with_move"高度+移动
            control_type: "unitree_handle"使用unitree 手柄控制, "other"使用外接设备进行控制
            fps: 控制频率
        """
        logger_mp.info("Initialize G1_Mobilebase_Height_Controller...")
        self.init_state = True
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.init_type = init_type 
        self.control_type = control_type

        # 添加数据接收标志位
        self.height_data_received = False
        self.move_data_received = False

        # init buffer
        #用于控制高度和移动的指令
        self.g1_height_action_array_in = Array('d', 1, lock = True) 
        self.g1_move_action_array_in = Array('d', 2, lock = True)


        #用于接收高度和移动的状态
        self.g1_height_state_array_out  = Array('d', 2, lock=True)  
        self.g1_height_action_array_out = Array('d', 1, lock=True) #用于接收发布给高度action的值，准备保存到数据集中
        self.g1_move_state_array_out = None
        self.g1_move_action_array_out =None #用于接收发布给移动action的值，准备保存到数据集中


        # init dds
        if self.simulation_mode:
            ChannelFactoryInitialize(1)
        else:
            ChannelFactoryInitialize(0)
        # 高度控制发布
        self.HeightCmb_publisher = ChannelPublisher(kTopicHeightCmd, Point32_)
        self.HeightCmb_publisher.Init()
        # 高度状态订阅
        self.HeightState_subscriber = ChannelSubscriber(kTopicHeightState, Point32_)
        self.HeightState_subscriber.Init()
        #高度action订阅
        self.HeightAction_subscriber = ChannelSubscriber(kTopicHeightAction, Point32_)
        self.HeightAction_subscriber.Init()

        self.g1_height_msg = geometry_msgs_msg_dds__Point32_()

        # 当init_type为with_move时，使用移动控制, 否则只使用高度控制
        if self.init_type == "with_move":
            self.g1_move_state_array_out = Array('d', 2, lock=True)
            self.g1_move_action_array_out = Array('d', 2, lock=True) #用于接收发布给移动action的值，准备保存到数据集中
            # 移动控制发布
            self.G1MoveCmb_publisher = ChannelPublisher(kTopicG1MoveCmd, Twist_)
            self.G1MoveCmb_publisher.Init()
            self.g1_move_msg = geometry_msgs_msg_dds__Twist_()
            # 移动状态订阅
            self.G1MoveState_subscriber = ChannelSubscriber(kTopicG1MoveState, Odometry_)
            self.G1MoveState_subscriber.Init()
            # 移动action订阅
            self.G1MoveAction_subscriber = ChannelSubscriber(kTopicMoveAction, Twist_)
            self.G1MoveAction_subscriber.Init()

        self.subscribe_g1_mobilebase_state_thread = threading.Thread(target=self._subscribe_g1_mobilebase_state)
        self.subscribe_g1_mobilebase_state_thread.daemon = True
        self.subscribe_g1_mobilebase_state_thread.start()

        self.subscribe_g1_height_action_thread = threading.Thread(target=self._subscribe_g1_height_action)
        self.subscribe_g1_height_action_thread.daemon = True
        self.subscribe_g1_height_action_thread.start()
        if self.init_type == "with_move":   
            self.subscribe_g1_move_action_thread = threading.Thread(target=self._subscribe_g1_move_action)
            self.subscribe_g1_move_action_thread.daemon = True
            self.subscribe_g1_move_action_thread.start()

        while True:
            if self.init_type == "with_move":
                # 需要同时接收到高度和移动数据
                if self.height_data_received and self.move_data_received:
                    self.init_state = False
                    print("[初始化] 已接收到高度和移动数据")
                    break
                else:
                    status = f"[初始化] 等待DDS数据... 高度: {self.height_data_received}, 移动: {self.move_data_received}"
                    print(status)
            else:
                # 只需要接收到高度数据
                if self.height_data_received:
                    self.init_state = False
                    print("[初始化] 已接收到高度数据")
                    break
                else:
                    print(f"[初始化] 等待高度数据...")
            time.sleep(0.02)
        
        logger_mp.info("[G1_Mobilebase_Height_Controller] Subscribe dds ok.")
        #如果使用的不是unitree 手柄，则启动控制进程
        if self.control_type != "unitree_handle":
            self.running = True
            g1_height_control_process = Process(target=self.control_process, args=(self.init_type))

            g1_height_control_process.daemon = True
            g1_height_control_process.start()

        logger_mp.info("Initialize G1_Mobilebase_Height_Controller OK!\n")
    def _subscribe_g1_height_action(self):
        print("[_subscribe_g1_height_action] 线程开始运行")
        while True:
            try:
                height_action_msg = self.HeightAction_subscriber.Read()
                if height_action_msg is not None:
                    self.g1_height_action_array_out[0] = height_action_msg.z   # 高度的速度指令
            except Exception as e:
                print(f"[_subscribe_g1_height_action] 异常: {e}")
                time.sleep(0.1)

    def _subscribe_g1_move_action(self):
        print("[_subscribe_g1_move_action] 线程开始运行")
        while True:
            try:
                move_action_msg = self.G1MoveAction_subscriber.Read()
                if move_action_msg is not None:
                    self.g1_move_action_array_out[0] = move_action_msg.linear.x   # 线速度
                    self.g1_move_action_array_out[1] = move_action_msg.angular.z  # 角速度
            except Exception as e:
                print(f"[_subscribe_g1_move_action] 异常: {e}")
                time.sleep(0.1)

    def _subscribe_g1_mobilebase_state(self):
        print("[_subscribe_g1_mobilebase_state] 线程开始运行")
        move_data_counter = 0  # 添加计数器，减少打印频率
        while True:
            try:
                # 初始化阶段：尝试读取高度数据
                height_msg = self.HeightState_subscriber.Read()
                if height_msg is not None:
                    self.g1_height_state_array_out[0] = height_msg.y # mm to m
                    self.g1_height_state_array_out[1] = height_msg.z
                    
                    if not self.height_data_received:
                        self.height_data_received = True
                        print(f"[初始化] 首次接收到高度数据: 高度={height_msg.y:.2f}, 速度={height_msg.z:.2f}")
                        
                if self.init_type == "with_move":
                    # 初始化阶段：尝试读取移动数据
                    move_msg = self.G1MoveState_subscriber.Read()
                    if move_msg is not None:
                        self.g1_move_state_array_out[0] = move_msg.twist.twist.linear.x
                        self.g1_move_state_array_out[1] = move_msg.twist.twist.angular.z
                        
                        if not self.move_data_received:
                            self.move_data_received = True
                            print(f"[初始化] 首次接收到移动数据: 线速度={move_msg.twist.twist.linear.x:.3f}, 角速度={move_msg.twist.twist.angular.z:.3f}")
                        
                        # 应用死区（小于阈值时设为0，消除静止时的抖动）
                        # DEADZONE_THRESHOLD = 0.015
                        # if abs(self.g1_move_state_array_out[0]) < DEADZONE_THRESHOLD:
                        #     self.g1_move_state_array_out[0] = 0.0
                        # if abs(self.g1_move_state_array_out[1]) < DEADZONE_THRESHOLD:
                        #     self.g1_move_state_array_out[1] = 0.0
                time.sleep(0.01)
                
            except Exception as e:
                print(f"[_subscribe_g1_mobilebase_state] 异常: {e}")
                time.sleep(0.1) 
    def ctrl_g1_height(self, g1_height_target):
        self.g1_height_msg.z = g1_height_target
        # print(f"g1_height_target: {g1_height_target}")
        self.HeightCmb_publisher.Write(self.g1_height_msg)
    

    def ctrl_g1_move(self, g1_move_target):
        self.g1_move_msg.linear.x = g1_move_target[0]
        self.g1_move_msg.angular.z = g1_move_target[1]
        self.G1MoveCmb_publisher.Write(self.g1_move_msg)
        # print(f"g1_move_target: {g1_move_target[0],g1_move_target[1]}")

    def control_process(self, init_type):
        try:
            self.start_time = time.time()
            while self.running:
                # 获取目标高度
                target_height = self.g1_height_action_array_in[0]
                # 发送调整后的高度命令
                self.ctrl_g1_height(target_height)
                
                if init_type == "with_move":
                    g1_move_target = self.g1_move_action_array_in
                    self.ctrl_g1_move(g1_move_target)
                
                current_time = time.time()
                time_elapsed = current_time - self.start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("G1_Mobilebase_Height_Controller has been closed.")

