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



class PmRobotNozzleControlWidget(QWidget):
    def __init__(self, ros_node:Node):
        super().__init__()
        self.ros_node = ros_node
        self.logger = self.ros_node.get_logger()
        self.controller_names = []
        self.get_state_functions = []
        self.init_nozzle_controllers()
        self.populate_controller_widget()
        self.init_states()

    def init_nozzle_controllers(self):

        try:
            package_path = get_package_share_directory('pm_robot_description')
            file_path = package_path + '/config/pm_robot_control_real_HW.yaml'
            with open(file_path, 'r') as file:
                yaml_dict = yaml.safe_load(file)

            self.controller_names = yaml_dict['pm_nozzle_controller']['ros__parameters']['nozzles']

        except Exception as e:
            self.ros_node.get_logger().error(f"Error getting pneumatic controllers: {e}")

    def populate_controller_widget(self):
        self.layout = QVBoxLayout(self)
        #self.setLayout(self.layout)
        
        get_states_button = QPushButton("Get States")
        get_states_button.clicked.connect(self.init_states)
        get_states_button.setFixedWidth(1000)
        get_states_button.setFixedHeight(50)
        # align the get_states_button to the center
    
        self.layout.addWidget(get_states_button)
        self.layout.setAlignment(get_states_button, Qt.AlignmentFlag.AlignCenter)

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

            enable_pressure_button = QPushButton("Enable Pressure")
            enable_vaccum_button = QPushButton("Enable Vaccum")
            turn_off_button = QPushButton("Turn Off")
            # set button size
            enable_pressure_button.setFixedWidth(400)
            enable_pressure_button.setFixedHeight(50)
            enable_vaccum_button.setFixedWidth(400)
            enable_vaccum_button.setFixedHeight(50)
            turn_off_button.setFixedWidth(400)
            turn_off_button.setFixedHeight(50)

            enable_pressure_button.clicked.connect(partial(self.enable_pres_clicked, controller, enable_vaccum_button, enable_pressure_button, turn_off_button))
            layout_grid.addWidget(enable_pressure_button, 0, 0)

            enable_vaccum_button.clicked.connect(partial(self.enable_vac_button_clicked, controller, enable_vaccum_button, enable_pressure_button, turn_off_button))
            layout_grid.addWidget(enable_vaccum_button, 0, 1)

            turn_off_button.clicked.connect(partial(self.turn_off_clicked, controller, enable_vaccum_button, enable_pressure_button, turn_off_button))
            layout_grid.addWidget(turn_off_button, 0, 2)

            self.get_state_functions.append(partial(self.get_state_cbk, controller, enable_vaccum_button, enable_pressure_button, turn_off_button))

    def enable_vac_button_clicked(self, controller_name:str, 
                                  enable_vac_button:QPushButton, 
                                  enable_pres_button:QPushButton,
                                  turn_off_button:QPushButton):
        self.logger.warn(f" Enable vaccum button clicked {controller_name}")
        exec_success = self.enable_vacuum(controller_name)
        if exec_success:
            enable_vac_button.setEnabled(False)
            enable_pres_button.setEnabled(True)
            turn_off_button.setEnabled(True)
            enable_vac_button.setStyleSheet("background-color: green")
            enable_pres_button.setStyleSheet("background-color: grey")
            turn_off_button.setStyleSheet("background-color: grey")

    def enable_pres_clicked(self, controller_name:str, 
                                enable_vac_button:QPushButton, 
                                enable_pres_button:QPushButton,
                                turn_off_button:QPushButton):
        self.logger.warn(f"Enable pressure button clicked {controller_name}")
        exec_success = self.enable_pressure(controller_name)
        if exec_success:
            enable_vac_button.setEnabled(True)
            enable_pres_button.setEnabled(False)
            turn_off_button.setEnabled(True)
            enable_vac_button.setStyleSheet("background-color: grey")
            enable_pres_button.setStyleSheet("background-color: green")
            turn_off_button.setStyleSheet("background-color: grey")

    def turn_off_clicked(self, controller_name:str, 
                        enable_vac_button:QPushButton, 
                        enable_pres_button:QPushButton,
                        turn_off_button:QPushButton):
        
        self.logger.warn(f"Turn off button clicked {controller_name}")
        exec_success = self.turn_off(controller_name)
        if exec_success:
            enable_vac_button.setEnabled(True)
            enable_pres_button.setEnabled(True)
            turn_off_button.setEnabled(False)
            enable_vac_button.setStyleSheet("background-color: grey")
            enable_pres_button.setStyleSheet("background-color: grey")
            turn_off_button.setStyleSheet("background-color: green")

    def get_state_cbk(self, controller_name:str, 
                        enable_vac_button:QPushButton, 
                        enable_pres_button:QPushButton,
                        turn_off_button:QPushButton):
        
        #self.logger.warn(f"Get state button clicked {controller_name}")
        state = self.get_current_state(controller_name)
        if state == -1:  
            enable_vac_button.setEnabled(False)
            enable_pres_button.setEnabled(True)
            turn_off_button.setEnabled(True)
            enable_vac_button.setStyleSheet("background-color: green")
            enable_pres_button.setStyleSheet("background-color: grey")
            turn_off_button.setStyleSheet("background-color: grey")
        elif state == 1:
            enable_vac_button.setEnabled(True)
            enable_pres_button.setEnabled(False)
            turn_off_button.setEnabled(True)
            enable_vac_button.setStyleSheet("background-color: grey")
            enable_pres_button.setStyleSheet("background-color: green")
            turn_off_button.setStyleSheet("background-color: grey")
        elif state == 0:
            enable_vac_button.setEnabled(True)
            enable_pres_button.setEnabled(True)
            turn_off_button.setEnabled(False)
            enable_vac_button.setStyleSheet("background-color: grey")
            enable_pres_button.setStyleSheet("background-color: grey")
            turn_off_button.setStyleSheet("background-color: green")

    def init_states(self):
        for get_state_function in self.get_state_functions:
            get_state_function()

    def get_current_state(self, controller_name:str)->int:
        return 1
    
    def enable_pressure(self, controller_name:str)->bool:
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

    def enable_vacuum(self, controller_name:str)->bool:
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

    def turn_off(self, controller_name:str)->bool:
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
            