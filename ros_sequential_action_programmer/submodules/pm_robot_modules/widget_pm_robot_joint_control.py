import sys
from PyQt6.QtWidgets import QApplication, QGridLayout, QMenu, QMainWindow, QTableWidget, QTableWidgetItem, QWidget, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QPushButton, QFormLayout, QHBoxLayout, QCheckBox, QComboBox
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

def degrees_to_radians(degrees)->float:
    return degrees * (math.pi / 180.0)

def radians_to_degrees(radians)->float:
    return radians * (180.0 / math.pi)

class PmRobotJointControlWidget(QWidget):
    def __init__(self, ros_node:Node):
        super().__init__()
        self.ros_node = ros_node

        gonio_right_controller_name = "/pm_robot_gonio_right_controller/follow_joint_trajectory"
        joints_gonio_right = ['Gonio_Right_Stage_1_Joint','Gonio_Right_Stage_2_Joint']

        self.gonio_right_controller = JointJogControl(self.ros_node, joints_gonio_right, gonio_right_controller_name)
        self.gonio_right_controller.set_current_joint_values([0.0, 0.0])
        self.gonio_right_controller.set_target_from_current()
        self.gonio_right_controller.send_target_joint_values()

        gonio_left_controller_name = "/pm_robot_gonio_left_controller/follow_joint_trajectory"
        joint_gonio_left = ['Gonio_Left_Stage_1_Joint','Gonio_Left_Stage_2_Joint']

        self.gonio_left_controller = JointJogControl(self.ros_node, joint_gonio_left, gonio_left_controller_name)
        self.gonio_left_controller.set_current_joint_values([0.0, 0.0])
        self.gonio_left_controller.set_target_from_current()
        self.gonio_left_controller.send_target_joint_values()


        t_axis_controller_name = "/pm_robot_t_axis_controller/follow_joint_trajectory"
        joints_t = ['T_Axis_Joint']

        self.t_axis_controller = JointJogControl(self.ros_node, joints_t, t_axis_controller_name)
        self.t_axis_controller.set_current_joint_values([0.0])
        self.t_axis_controller.set_target_from_current()
        self.t_axis_controller.send_target_joint_values()

        xyz_controller_name = "/pm_robot_xyz_axis_controller/follow_joint_trajectory"
        joints_xyz = ['X_Axis_Joint','Y_Axis_Joint','Z_Axis_Joint']

        self.xyz_controller = JointJogControl(self.ros_node, joints_xyz, xyz_controller_name)
        self.xyz_controller.set_current_joint_values([0.0, 0.0, 0.0])
        self.xyz_controller.set_target_from_current()
        self.xyz_controller.send_target_joint_values()

        self.init_ui()  
    
    def init_ui(self):
        central_widget = QWidget(self)
        #self.setCentralWidget(central_widget)
        self.font_label = QFont()  # Create a QFont object
        self.font_label.setPointSize(14)  # Set the font size to 16 points
        main_layout = QGridLayout()

        self.vertical_layout = QVBoxLayout()
        #main_layout.addLayout(self.vertical_layout,2,0,1,2)

        central_widget.setLayout(self.vertical_layout)

        self.add_jog_widgets_for_instance(self.gonio_right_controller, [-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0], "deg")
        self.add_jog_widgets_for_instance(self.gonio_left_controller, [-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0], "deg")
        self.add_jog_widgets_for_instance(self.t_axis_controller, [-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0], "deg")
        self.add_jog_widgets_for_instance(self.xyz_controller, [-10.0,-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0,10.0], "mm")



    def add_jog_widgets_for_instance(self, joint_control_instance:JointJogControl, incement_list:list, unit:str):
        joint_instance_layout = QVBoxLayout()
        self.vertical_layout.addLayout(joint_instance_layout)

        for index, joint in enumerate(joint_control_instance.joint_names):
            joint_jog_layout_v = QHBoxLayout()
            joint_jog_layout_v.setAlignment(Qt.AlignmentFlag.AlignCenter)

            # Three widget in the middle
            joint_layout = QVBoxLayout()
            joint_label_widget = QLabel(f'{joint} [{unit}]')
            joint_label_widget.setFont(self.font_label)
            joint_label_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
            display_widget_current_value = QLineEdit()
            display_widget_current_value.setReadOnly(True)
            display_widget_current_value.setFixedWidth(100)
            display_widget_current_value.setToolTip(f"Current value of '{joint}'joint")
            if unit == "deg":
                current_value = radians_to_degrees(joint_control_instance.get_current_joint_value_for_joint(joint))
                current_target_value = radians_to_degrees(joint_control_instance.target_joint_values[index])
            else:
                current_value = joint_control_instance.get_current_joint_value_for_joint(joint)
                current_target_value = joint_control_instance.target_joint_values[index]
            display_widget_current_value.setText(str(round(current_value,6)))
            display_widget_target_value = QLineEdit()
            display_widget_target_value.setFixedWidth(100)
            display_widget_target_value.setToolTip(f"Target value of '{joint}'joint")

            for index, inc_value in enumerate(incement_list):
                    if index == int(len(incement_list)/2):
                        joint_layout.addWidget(joint_label_widget)
                        joint_layout.addWidget(display_widget_current_value)
                        joint_layout.addWidget(display_widget_target_value)
                        joint_jog_layout_v.addLayout(joint_layout)

                    button = QPushButton(str(inc_value))
                    button.setFixedWidth(60)
                    button.clicked.connect(partial(self.clb_change_joint_target_value, display_widget_target_value, joint_control_instance, joint, inc_value, unit))
                    joint_jog_layout_v.addWidget(button)
       
            joint_instance_layout.addLayout(joint_jog_layout_v)
        
        send_button = QPushButton("Send Target Values")
        send_button.clicked.connect(joint_control_instance.send_target_joint_values)
        joint_instance_layout.addWidget(send_button)
    
    def clb_change_joint_target_value(self, target_value_widget:QLineEdit, joint_control_instance:JointJogControl, joint_name:str, inc_value:float, unit:str):
        joint_control_instance.set_target_from_current()
        current_value = joint_control_instance.get_current_joint_value_for_joint(joint_name)
        if unit == "deg":
            inc_value = degrees_to_radians(inc_value)
            new_value = current_value + inc_value
            disp_value = radians_to_degrees(new_value)
        else:
            new_value = current_value + inc_value
            disp_value = new_value
        joint_control_instance.set_target_joint_value(joint_name, new_value)
        target_value_widget.setText(str(round(disp_value,6)))