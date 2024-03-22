from PyQt6.QtWidgets import QScrollArea, QMenuBar, QMenu, QDialog, QHBoxLayout, QInputDialog, QTreeWidget, QTreeWidgetItem, QApplication, QGridLayout, QFrame, QMainWindow, QListWidget, QListWidgetItem, QDoubleSpinBox, QWidget, QVBoxLayout, QPushButton, QCheckBox, QLineEdit, QComboBox, QTextEdit,QLabel,QSlider, QSpinBox, QFontDialog, QFileDialog

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QAction
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread 
import sys
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RecomGenerator import RecomGenerator
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AppTextWidget import AppTextOutput
import yaml
from ament_index_python.packages import PackageNotFoundError
from functools import partial
import json
import subprocess
import os

try:
    from pm_vision_interfaces.srv import GetRunningAssistants, StartVisionAssistant, StopVisionAssistant
    from pm_vision_manager.va_py_modules.vision_assistant_class import VisionProcessClass
except:
    pass

def append_vision_widget_to_menu(mainWindow: QMainWindow, menu_bar:QMenu, node:Node):
    """
    This function tries to appends the vision manager widget to the menu bar of the main window.
    It will only work if the pm_vision_manager package is installed.
    param mainWindow: The main window of the application
    param menu_bar: The menu bar of the main window
    param node: a ros node
    """
    try:
        package_dir = get_package_share_directory('pm_vision_manager')
        open_vision = QAction("Vision Manager", mainWindow)
        #open_vision.triggered.connect(partial(create_vision_widget, mainWindow, node))
        open_vision.triggered.connect(open_new_vision_instance)
        menu_bar.addAction(open_vision)

    except PackageNotFoundError:
        print("pm_vision_manager not found")
        return

def create_vision_widget(main_window, node:Node):
    main_window.vision_widget = VisionWidget(node)
    main_window.vision_widget.show()

def open_new_vision_instance():
    command  = 'terminator --new-tab -x ros2 launch pm_vision_manager pm_vision.launch.py'
    subprocess.call(command, shell=True)



class VisionWidget(QWidget):
    def __init__(self, node:Node):
        super().__init__()
        self.node = node
        self.get_running_assistants_action = ServiceAction(node, client='/pm_vision_manager/GetRunningAssistants', service_type='pm_vision_interfaces/srv/GetRunningAssistants')
        self.stop_assistant_action = ServiceAction(node, client='/pm_vision_manager/StopVisionAssistant', service_type='pm_vision_interfaces/srv/StopVisionAssistant')
        self.stop_assistant_action.set_service_bool_identifier('success')
        self.start_assistant_action = ServiceAction(node, client='/pm_vision_manager/StartVisionAssistant', service_type='pm_vision_interfaces/srv/StartVisionAssistant')
        self.start_assistant_action.set_service_bool_identifier('success')
        self.initUI()
        self.get_running_assistants()
        self.update_files()

    def initUI(self):
        # Grid layout
        self.setWindowTitle("Vision Manager")
        self.main_layout = QGridLayout()
        self.running_assistans_widget = QListWidget()
        self.vision_processes_widget = QListWidget()
        self.camera_configs_widget = QListWidget()
        # add button
        self.stop_assistant_button = QPushButton("Stop Vision Assistant")
        self.stop_assistant_button.clicked.connect(self.stop_assistant)
        self.start_assistant_button = QPushButton("Start Vision Assistant")
        self.start_assistant_button.clicked.connect(self.start_vision_assistant)
        self.referesh_running_assistant_button = QPushButton("Refresh Assistants")
        self.referesh_running_assistant_button.clicked.connect(self.get_running_assistants)
        self.new_process_button = QPushButton("Create New Process")
        self.new_process_button.clicked.connect(self.create_new_process_file)
        label_processes = QLabel("Vision Processes:")
        label_cameras = QLabel("Camera Configs:")
        label_assistants = QLabel("Running Assistants:")
        label_text_output = QLabel("Log Output:")
        # add a text output box
        self.text_output = AppTextOutput()
        self.main_layout.addWidget(self.start_assistant_button, 0, 0)
        self.main_layout.addWidget(self.new_process_button, 0, 1)
        self.main_layout.addWidget(label_processes,1,0)
        self.main_layout.addWidget(self.vision_processes_widget,2,0)
        self.main_layout.addWidget(label_cameras,1,1)
        self.main_layout.addWidget(self.camera_configs_widget,2,1)
        self.main_layout.addWidget(label_assistants,3,0)
        self.main_layout.addWidget(self.running_assistans_widget, 4, 0, 1, 2)
        self.main_layout.addWidget(self.stop_assistant_button, 5, 0)
        self.main_layout.addWidget(self.referesh_running_assistant_button, 5, 1)
        self.main_layout.addWidget(label_text_output, 6, 0)
        self.main_layout.addWidget(self.text_output, 7, 0, 1, 2)
        self.setGeometry(100, 100, 800, 1000)
        self.setLayout(self.main_layout)

    def get_running_assistants(self):
        execute_success:bool = self.get_running_assistants_action.execute()
        if execute_success:
            self.running_assistans_widget.clear()
            for assistant in self.get_running_assistants_action.service_response.running_assistants:
                self.running_assistans_widget.addItem(assistant)
        else:
            self.text_output.append_red_text("Could not get running assistants! Client is not online!")

    def stop_assistant(self):
        stop_request = StopVisionAssistant.Request()
        assistant_item= self.running_assistans_widget.currentItem()

        if assistant_item:
            stop_request.process_uid = assistant_item.text()
            self.stop_assistant_action.set_service_request(stop_request)
            execute_success:bool = self.stop_assistant_action.execute()
            self.get_running_assistants()
        else:
            print("No assistant selected")
        

    def update_files(self):

        package_share_directory = get_package_share_directory('pm_vision_manager')
        path_config_path = package_share_directory + '/vision_assistant_path_config.yaml'
        with open(path_config_path, 'r') as file:
            FileData = yaml.safe_load(file)
            config = FileData["vision_assistant_path_config"]
            self.process_library_path=config["process_library_path"]
            camera_config_path=config["camera_config_path"]
            vision_processes = RecomGenerator.get_files_in_dir(directory=self.process_library_path,file_end='.json', exclude_str=['results'])
            vision_cameras = RecomGenerator.get_files_in_dir(directory=camera_config_path,file_end='.yaml', exclude_str=['vision_assistant'])
            self.vision_processes_widget.clear()
            self.camera_configs_widget.clear()
            for process in vision_processes:
                self.vision_processes_widget.addItem(process)
            for camera in vision_cameras:
                self.camera_configs_widget.addItem(camera)

    def create_new_process_file(self):
        new_process_file, _ = QFileDialog.getSaveFileName(self, "Save File", self.process_library_path, "JSON Files (*.json)")
        if new_process_file:
            process_name = new_process_file.split('/')[-1]
            if not new_process_file.endswith('.json'):
                new_process_file = new_process_file + '.json'
            with open(new_process_file, 'w') as file:
                default_process_dict = VisionProcessClass.create_default_process_dict(process_name)
                json.dump(default_process_dict, file, indent=4)
                
            self.update_files()

    def start_vision_assistant(self):
        selected_process = self.vision_processes_widget.currentItem()
        selected_camera = self.camera_configs_widget.currentItem()
        if selected_process and selected_camera:
            new_id, dialog_ok = self.enter_id_dialog()
            if not dialog_ok:
                return
            if new_id == '':
                self.text_output.append_red_text("No Id entered!")
                return
            start_request = StartVisionAssistant.Request()
            start_request.process_filename = selected_process.text()
            start_request.camera_config_filename = selected_camera.text()
            start_request.open_process_file = True
            start_request.process_uid = new_id
            self.start_assistant_action.set_service_request(start_request)
            execute_success:bool = self.start_assistant_action.execute()
            if execute_success:
                self.text_output.append_green_text("Assistant started successfully!")
            else:
                self.text_output.append_red_text("Assistant could not be started!")
            self.get_running_assistants()

        else:
            self.text_output.append_red_text("No process or camera selected!")

    def enter_id_dialog(self) -> bool:
        text, ok = QInputDialog.getText(self, 'Input Dialog', 'Enter a unique process Id for new assistant:')
        return text, ok

class DummyMain(QMainWindow):

    def __init__(self, node:Node):
        super().__init__()
        self.node = node
        self.w = None  # No external window yet.
        self.button = QPushButton("Push for Window")
        self.button.clicked.connect(self.show_new_window)
        self.setCentralWidget(self.button)

    def show_new_window(self, checked):
        if self.w is None:
            self.w = VisionWidget(self.node)
        self.w.show()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=6) 

    app = QApplication(sys.argv)
    just_a_node = Node('my_Node')
    executor.add_node(just_a_node)

    thread = Thread(target=executor.spin)
    thread.start()
    ex = DummyMain(just_a_node)
    try:
        ex.show()
        sys.exit(app.exec())

    finally:
        just_a_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    

if __name__ == '__main__':
    open_new_vision_instance()
    #main()

