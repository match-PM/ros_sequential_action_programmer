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
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import yaml

def append_calbiration_panel_to_menu(mainWindow: QMainWindow, menu_bar:QMenu, node:Node):
    """
    This function tries to append the jobpanel widget to the menu bar of the main window.
    It will only work if the 'pm_moveit_server' package is installed.
    param mainWindow: The main window of the application
    param menu_bar: The menu bar of the main window
    param node: a ros node
    """
    def create_jog_widget(main_window: QMainWindow, node:Node):
        main_window.panel = PmRobotCalibrationWidget(node)
        main_window.panel.show()
    
    try:
        package_dir = get_package_share_directory('pm_robot_calibration')
        open_panel = QAction("Pm Robot Calbiration Panel", mainWindow)
        open_panel.triggered.connect(partial(create_jog_widget, mainWindow, node))
        menu_bar.addAction(open_panel)

    except PackageNotFoundError:
        node.get_logger().warn("Package 'pm_robot_calbiration' not found! Pm Robot Calibration Panel will not be available.")
        return
    

   
    
class CalibrationService():
    def __init__(self, 
                 node: Node, 
                 client_name:str, 
                 service_type:str, 
                 dict_key: str):
        self.node = node
        self.client_name = client_name
        self.service_type = service_type
        self.dict_key = dict_key
        self.action = ServiceAction(node=self.node, 
                                    client=self.client_name, 
                                    service_type=self.service_type)
        
    def call_service(self):
        execute_success = self.action.execute()
        
        return execute_success
    
    def read_calbiration_yaml(self)-> dict:
        """
        This function reads the calibration yaml file and returns the data as a dictionary.
        """
        package_dir = get_package_share_directory('pm_robot_calibration')
        yaml_file_path = package_dir + 'last_calibrations.yaml'
        
        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)
        
        time_stamp = data.get(self.dict_key, "")
        return str(time_stamp)
    

class PmRobotCalibrationWidget(QWidget):
    def __init__(self, ros_node:Node):
        
        from pm_msgs.srv import EmptyWithSuccess
        
        super().__init__()
        self.ros_node = ros_node
        self.logger = self.ros_node.get_logger()
        self.calbiration_services: list[CalibrationService] = []
        self._init_calibraition_services()
        self._init_layout()
    
    
    def _init_calibraition_services(self):
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                            client_name='/pm_robot_calibration/calibrate_cameras',
                                                            service_type='pm_msgs/srv/EmptyWithSuccess',
                                                            dict_key='cameras'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_calibration_cube_to_cam_top',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='calibration_cube_to_cam_top'))

        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_laser_on_calibration_cube',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='laser_on_calibration_cube'))
    
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_confocal_top',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='confocal_top'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_calibration_target_to_cam_bottom',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='calibration_target_to_cam_bottom'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_confocal_bottom',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='confocal_bottom'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_gonio_left_chuck',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='gonio_left_chuck'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_gonio_right_chuck',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='gonio_right_chuck'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_gripper',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='gripper'))
        
        self.calbiration_services.append(CalibrationService(node=self.ros_node,
                                                    client_name='/pm_robot_calibration/calibrate_1K_dispenser',
                                                    service_type='pm_msgs/srv/EmptyWithSuccess',
                                                    dict_key='1K_dispenser'))
        
    def _init_layout(self):
        self.layout = QVBoxLayout(self)
        
        for index, service in enumerate(self.calbiration_services):
            service_layout = QHBoxLayout()
            service_label = QLabel(str(index))
            service: CalibrationService 
            service_button = QPushButton(service.client_name)
            service_button.clicked.connect(service.call_service)
            service_button.setFixedWidth(800)
            service_button.setFixedHeight(50)
            service_button.setStyleSheet("background-color: grey")
            service_button.setEnabled(True)
            service_button.setObjectName(service.client_name)
            service_layout.addWidget(service_label)
            service_layout.addWidget(service_button)
            self.layout.addLayout(service_layout)
