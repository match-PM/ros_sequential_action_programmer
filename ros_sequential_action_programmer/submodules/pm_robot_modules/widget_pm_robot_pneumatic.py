import sys
from PyQt6.QtWidgets import QApplication, QGroupBox, QGridLayout, QMenu, QMainWindow, QTableWidget, QTableWidgetItem, QWidget, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QPushButton, QFormLayout, QHBoxLayout, QCheckBox, QComboBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QAction
import yaml
from ament_index_python.packages import get_package_share_directory
from functools import partial, reduce
from typing import Union
from geometry_msgs.msg import Pose
from operator import attrgetter
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread 
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from copy import deepcopy, copy
import yaml
import tf2_py as tf2
from geometry_msgs.msg import Vector3, Quaternion
from importlib import import_module
from sensor_msgs.msg._joint_state import JointState
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from rosidl_runtime_py.convert import message_to_ordereddict
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AppTextWidget import AppTextOutput
from ament_index_python.packages import PackageNotFoundError
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import time
from ros_sequential_action_programmer.submodules.pm_robot_modules.joint_controller_class import JointJogControl
import math


class PmRobotPneumaticControlWidget(QWidget):
    def __init__(self, ros_node:Node):
        super().__init__()
        self.ros_node = ros_node
        self.logger = self.ros_node.get_logger()
        self.controller_names = []
        self.get_state_functions = []
        self.init_pneumatic_controllers()
        self.populate_controller_widget()
        self.init_states()

    def init_pneumatic_controllers(self):

        try:
            package_path = get_package_share_directory('pm_robot_description')
            file_path = package_path + '/config/pm_robot_control_real_HW.yaml'
            with open(file_path, 'r') as file:
                yaml_dict = yaml.safe_load(file)

            self.controller_names = yaml_dict['pm_pneumatic_controller']['ros__parameters']['cylinders']

        except Exception as e:
            self.ros_node.get_logger().error(f"Error getting pneumatic controllers: {e}")

    def populate_controller_widget(self):
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        get_states_button = QPushButton("Get States")
        get_states_button.clicked.connect(self.init_states)
        get_states_button.setFixedWidth(800)
        get_states_button.setFixedHeight(50)

        self.layout.addWidget(get_states_button)

        for controller in self.controller_names:
            controller_label = QLabel(controller)
            controller_label.setFont(QFont('Arial', 16, QFont.Weight.Bold))
            controller_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            group_box = QGroupBox()
            # Center title
            group_box.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.layout.addWidget(controller_label)
            self.layout.addWidget(group_box)
            layout_grid = QGridLayout()
            group_box.setLayout(layout_grid)

            move_backward_button = QPushButton("Move Backward")
            move_forward_button = QPushButton("Move Forward")
            # set button size
            move_backward_button.setFixedWidth(400)
            move_backward_button.setFixedHeight(50)
            move_forward_button.setFixedWidth(400)
            move_forward_button.setFixedHeight(50)
            move_backward_button.clicked.connect(partial(self.backward_button_clicked, controller, move_forward_button, move_backward_button))
            layout_grid.addWidget(move_backward_button, 0, 0)
            
            move_forward_button.clicked.connect(partial(self.forward_button_clicked, controller, move_forward_button, move_backward_button))
            
            self.get_state_functions.append(partial(self.get_state_cbk, controller, move_forward_button, move_backward_button))

            layout_grid.addWidget(move_forward_button, 0, 1)

    def forward_button_clicked(self, controller_name:str, move_forward_button:QPushButton, move_backward_button:QPushButton):
        self.logger.warn(f"Forward button clicked {controller_name}")
        move_success = self.move_forward(controller_name)
        if move_success:
            move_forward_button.setEnabled(False)
            move_backward_button.setEnabled(True)
            move_forward_button.setStyleSheet("background-color: green")
            move_backward_button.setStyleSheet("background-color: grey")


    def backward_button_clicked(self, controller_name:str, move_forward_button:QPushButton, move_backward_button:QPushButton):
        self.logger.warn(f"Backward button clicked {controller_name}")
        move_success = self.move_backward(controller_name)
        if move_success:
            move_backward_button.setStyleSheet("background-color: green")
            move_forward_button.setStyleSheet("background-color: grey")
            move_forward_button.setEnabled(True)
            move_backward_button.setEnabled(False)

    def get_state_cbk(self, controller_name:str, move_forward_button:QPushButton, move_backward_button:QPushButton):
        #self.logger.warn(f"Get state button clicked {controller_name}")
        is_forward = self.check_is_state_forward(controller_name)
        if is_forward:
            move_forward_button.setEnabled(False)
            move_backward_button.setEnabled(True)
            move_forward_button.setStyleSheet("background-color: green")
            move_backward_button.setStyleSheet("background-color: grey")
        else:
            move_forward_button.setEnabled(True)
            move_backward_button.setEnabled(False)
            move_forward_button.setStyleSheet("background-color: grey")
            move_backward_button.setStyleSheet("background-color: green")

    def init_states(self):
        for get_state_function in self.get_state_functions:
            get_state_function()

    def check_is_state_forward(self, controller_name:str)->bool:
        return True
    
    def move_forward(self, controller_name:str)->bool:
        client_name = controller_name + '_client'
        # _client = self.ros_node.create_client(ServiceAction, 'client_name')
        
        # if not _client.wait_for_service(timeout_sec=1.0):
        #     self.logger.error(f"Service {client_name} not available")
        #     return False
        # request = ServiceAction.Request()
        # request.action = 'forward'
        # request.controller = controller_name
        # future = _client.call_async(request)
        # rclpy.spin_until_future_complete(self.ros_node, future)
        # if future.result() is not None:
        #     self.logger.warn(f"Moving forward: {controller_name}")
        # _client.destroy()    
        self.logger.warn(f"Moving forward: {controller_name}")

        return True
    
    def move_backward(self, controller_name:str)->bool:
        client_name = controller_name + '_client'
        # _client = self.ros_node.create_client(ServiceAction, 'client_name')
        
        # if not _client.wait_for_service(timeout_sec=1.0):
        #     self.logger.error(f"Service {client_name} not available")
        #     return False
        # request = ServiceAction.Request()
        # request.action = 'forward'
        # request.controller = controller_name
        # future = _client.call_async(request)
        # rclpy.spin_until_future_complete(self.ros_node, future)
        # if future.result() is not None:
        #     self.logger.warn(f"Moving forward: {controller_name}")
        # _client.destroy()    
        self.logger.warn(f"Moving forward: {controller_name}")

        return True