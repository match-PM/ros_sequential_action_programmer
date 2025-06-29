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
from pm_msgs.srv import EmptyWithSuccess,PneumaticGetPosition




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
        self.layout = QVBoxLayout(self)
        
        get_states_button = QPushButton("Get States")
        get_states_button.clicked.connect(self.init_states)
        get_states_button.setFixedWidth(800)
        get_states_button.setFixedHeight(50)

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
        if '1K' in controller_name or '2K' in controller_name:
            controller_name = f'N{controller_name}'
        
        client_name = f'/pm_pneumatic_controller/{controller_name}/GetPosition'
        _client = self.ros_node.create_client(PneumaticGetPosition, client_name)
        if not _client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f"Service {client_name} not available")
            return False
        request = PneumaticGetPosition.Request()
        response:PneumaticGetPosition.Response = _client.call(request)

        _client.destroy()
        
        if response.position == -1:
            return False
        else:
            return True
    
    def move_forward(self, controller_name:str)->bool:
        
        if '1K' in controller_name or '2K' in controller_name:
            controller_name = f'N{controller_name}'

        client_name = f'/pm_pneumatic_controller/{controller_name}/MoveForward'
        _client = self.ros_node.create_client(EmptyWithSuccess, client_name)
        self.logger.error(f"Call started {client_name}")
        if not _client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f"Service {client_name} not available")
            return False
        
        request = EmptyWithSuccess.Request()
        response:EmptyWithSuccess.Response = _client.call(request)
        _client.destroy()  
        self.logger.error(f"Call finished {client_name}")
        if not response.success:
            self.logger.error(f"Failed to move forward.")
            return False
                  
        return True
    
    def move_backward(self, controller_name:str)->bool:

        if '1K' in controller_name or '2K' in controller_name:
            controller_name = f'N{controller_name}'
    
        client_name = f'/pm_pneumatic_controller/{controller_name}/MoveBackward'
        _client = self.ros_node.create_client(EmptyWithSuccess, client_name)
        if not _client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f"Service {client_name} not available")
            return False
        request = EmptyWithSuccess.Request()
        response:EmptyWithSuccess.Response = _client.call(request)

        _client.destroy()

        if not response.success:
            self.logger.error(f"Failed to move backward.")
            return False

        return True
    



