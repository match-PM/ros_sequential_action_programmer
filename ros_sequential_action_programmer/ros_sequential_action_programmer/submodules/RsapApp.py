from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool, QMimeData

from PyQt6.QtWidgets import QScrollArea, QMessageBox, QDialog, QHBoxLayout, QDialog, QInputDialog, QTreeWidget, QTreeWidgetItem, QApplication, QGridLayout, QFrame, QMainWindow, QListWidget, QListWidgetItem, QDoubleSpinBox, QWidget, QVBoxLayout, QPushButton, QCheckBox, QLineEdit, QComboBox, QTextEdit,QLabel,QSlider, QSpinBox, QFontDialog, QFileDialog, QMenu
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction, QClipboard
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from PyQt6 import QtCore
from functools import partial
import yaml
from yaml.loader import SafeLoader
import sys
from collections import OrderedDict
from rclpy.node import Node
from datetime import datetime
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer
from ros_sequential_action_programmer.submodules.RsapApp_submodules.PopupRecWindow import PopupRecWindow
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionSelectionMenu import ActionSelectionMenu, SelectionMenu
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AddServiceDialog import AddServiceDialog
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AddUserInteractionDialog import AddUserInteractionDialog
from ros_sequential_action_programmer.submodules.RsapApp_submodules.StatusIndicator import StatusIndicator
from ros_sequential_action_programmer.submodules.RsapApp_submodules.UserDialog import UserDialog
from ros_sequential_action_programmer.submodules.RsapApp_submodules.GuiParameterFileActions import GuiParameterFileActions
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
import ast
from ros_sequential_action_programmer.submodules.RsapApp_submodules.UserInteractionActionDialog import UserInteractionActionDialog
from ros_sequential_action_programmer.submodules.RsapApp_submodules.app_worker import RsapExecutionWorker, RsapExecutionRunWorker
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AppTextWidget import AppTextOutput
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ConfigWindow import DictionaryValueEditor, NestedDictionaryEditor
from ros_sequential_action_programmer.submodules.RsapApp_submodules.NoScrollComboBox import NoScrollComboBox
from ros_sequential_action_programmer.submodules.RsapApp_submodules.action_list_widget_adv import ActionSequenceListWidget
from ros_sequential_action_programmer.submodules.RsapApp_submodules.action_list_widgets import ActionListWidget, ActionListItem
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionParameterWidget import ActionParameterWidget, ActionParameterMainLayout
from ros_sequential_action_programmer.submodules.RsapApp_submodules.SequenceInfoWidget import SequenceInfoWidget
from ros_sequential_action_programmer.submodules.RsapApp_submodules.RecentFilesManager import RecentFilesManager
from ros_sequential_action_programmer.submodules.RsapApp_submodules.SeqParameterWindow import SeqParameterManagerDialog

# Change this later to be cleaner
try:
    # Import pm_co_pilot_programming
    pass
except ModuleNotFoundError as e:
    print(f"Error importing PmCoPilotAppProgramming: {e}")

from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI

class RsapApp(QMainWindow):
    def __init__(self, service_node:Node):
        super().__init__()
        self.service_node = service_node
        
        self.action_sequence_builder = RosSequentialActionProgrammer(service_node)
        self.recent_files_manager = RecentFilesManager(self.action_sequence_builder)
        self.thread_pool = QThreadPool()
        # load the recent file (last opened file)
        self.recent_files_manager.load_recent_file()
        self._init_signal()
        self.initUI()
        #self.init_actions_list()
        self.action_list_widget.populate_list()
        #self.initialize_active_service_list()
        self.execution_running = False
        self.sub_window_list = []
        

    def _init_signal(self):
        self.user_interaction_signals_list = [] # used to store the signals for the user interaction
        self.action_sequence_builder.signal_incoming_log.signal.connect(self.print_ros_log)
        self.action_sequence_builder.signal_execution_status.signal.connect(self.set_widget_activations)
        self.action_sequence_builder.signal_current_action.signal.connect(self.select_action_in_execution)

    def initUI(self):
        # Create the class for the action selection menu
        self.action_menu = ActionSelectionMenu(self, rsap=self.action_sequence_builder)
        # Overriting the callbackfunction for the menu
        self.action_menu.action_menu_clb = self.append_selected_action_from_menu
        
        self.setWindowTitle("Ros Sequential Action Programmer - RSAP")
        
        self.ros_run_menu = SelectionMenu(self)
        self.ros_run_menu.action_menu_clb = self.run_ros_executable

        self.ros_launch_menu = SelectionMenu(self)
        self.ros_launch_menu.action_menu_clb = self.run_ros_executable_launch

        self.text_output = AppTextOutput()

        self.action_list_widget = ActionSequenceListWidget(self.action_sequence_builder, 
                                                    text_output=self.text_output)
        self.action_list_widget.populate_list()
        self.action_list_widget.itemClicked.connect(self.action_selected)

        # Create main container widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Create layout for the main container widget
        layout = QGridLayout()
        self.rsap_seq_info_widget = SequenceInfoWidget(self.action_sequence_builder)

        layout.addWidget(self.rsap_seq_info_widget,0,1,1,2)
        
        status_layout = QVBoxLayout()
        #status_label = QLabel("Status")
        #status_label.setFont(font)
        #status_layout.addWidget(status_label,alignment=Qt.AlignmentFlag.AlignCenter)
        self.status_indicator = StatusIndicator()
        self.status_indicator.reset_clicked.connect(self.reset_action_list_status)
        status_layout.addWidget(self.status_indicator,alignment=Qt.AlignmentFlag.AlignCenter)
        layout.addLayout(status_layout,0,0,1,1,alignment=Qt.AlignmentFlag.AlignCenter)

        toolbar_layout = QVBoxLayout()
        self.open_action_menu_button = QPushButton('Add \n Action')
        self.open_action_menu_button.clicked.connect(self.action_menu.show_action_menu)
        self.open_action_menu_button.setFixedSize =(20,20)

        toolbar_layout.addWidget(self.open_action_menu_button,alignment=Qt.AlignmentFlag.AlignTop)

        # add button for deleting selected vision function
        self.delete_button = QPushButton("Delete \n Selected")
        self.delete_button.clicked.connect(self.action_list_widget.delete_actions_from_sequence)
        toolbar_layout.addWidget(self.delete_button,alignment=Qt.AlignmentFlag.AlignTop)

        self.copy_and_insert_button = QPushButton("Copy and \n Insert")
        self.copy_and_insert_button.clicked.connect(self.copy_and_insert_actions)
        toolbar_layout.addWidget(self.copy_and_insert_button,alignment=Qt.AlignmentFlag.AlignTop)

        layout.addLayout(toolbar_layout,1,0,alignment=Qt.AlignmentFlag.AlignTop)

        execute_layout = QHBoxLayout()
        stop_execution_layout = QHBoxLayout()
        # Add combobox for vision pipline building

        layout.addWidget(self.action_list_widget,1,1)

        self.execute_step_button = QPushButton("Step")
        self.execute_step_button.clicked.connect(self.execute_current_action)
        #self.execute_step_button.clicked.connect(self.initiate_current_action_execution)
        execute_layout.addWidget(self.execute_step_button)

        self.run_action_sequence_button = QPushButton("Run")
        self.run_action_sequence_button.clicked.connect(self.run_action_sequence)
        execute_layout.addWidget(self.run_action_sequence_button)
        layout.addLayout(execute_layout,2,1)
        layout.addLayout(stop_execution_layout,3,1)

        stop_execution_button = QPushButton("Stop Execution")
        stop_execution_button.clicked.connect(self.action_sequence_builder.set_stop_execution)
        stop_execution_layout.addWidget(stop_execution_button)

        interupt_execution_button = QPushButton("Interrupt Action")
        interupt_execution_button.clicked.connect(self.interrupt_action)
        stop_execution_layout.addWidget(interupt_execution_button)

        # add textbox for string output
        layout.addWidget(self.text_output,4,1,1,3)

        self.last_saved_label = QLabel('File not saved yet!')
        layout.addWidget(self.last_saved_label,5,1,1,1)

        self.clear_text_output_button = QPushButton("Clear")
        self.clear_text_output_button.clicked.connect(self.text_output.clear)

        layout.addWidget(self.clear_text_output_button,5,2,1,1)
        # # create sub layout for the function parameter widgets
        self.action_parameter_layout = ActionParameterMainLayout()
        
        # Create a menu bar
        menubar = self.menuBar()
        menubar.setStyleSheet("QMenuBar { font-size: 18px; }")
        file_menu = menubar.addMenu("File")

        # Create "Save" action
        save_action = QAction("Save", self)
        save_action.triggered.connect(self.save_process)
        file_menu.addAction(save_action)

        # Create 'save as' action
        save_as_action = QAction("Save process as", self)
        save_as_action.triggered.connect(self.save_process_as)
        file_menu.addAction(save_as_action)

        app_config_menu = menubar.addMenu("Settings")
        seq_param_menu = menubar.addMenu("Sequence Parameters File")

        #pm_robot_tools_menu = menubar.addMenu("PM Robot Tools")
        copilot_menu = menubar.addMenu("Co-Pilot")
        
        # Add Co-Pilot action
        open_copilot_action = QAction("Open Co-Pilot Programming", self)
        open_copilot_action.triggered.connect(self.openCoPilot)
        copilot_menu.addAction(open_copilot_action)

        open_copilot_planning_action = QAction("Open Co-Pilot Planning", self)
        open_copilot_planning_action.triggered.connect(self.openCoPilotPlanning)
        copilot_menu.addAction(open_copilot_planning_action)

        ros_menu = menubar.addMenu("ROS2 Functionalities")
        
        # Create "New" action
        new_action = QAction("New process", self)
        new_action.triggered.connect(self.create_new_file)
        file_menu.addAction(new_action)

        # Create "Open" action
        open_action = QAction("Open process", self)
        open_action.triggered.connect(self.open_process_file_dia)
        file_menu.addAction(open_action)

        # Recent files submenu
        self.recent_files_menu = file_menu.addMenu("Open Recent Process")
        self.update_recent_files_menu()  # Populate the submenu dynamically
        

        
        # try:
        #     open_pm_robot_config = QAction("PM Robot Config", self)
        #     open_pm_robot_config.triggered.connect(partial(self.open_sub_window, PmRobotConfigWidget))
        #     #pm_robot_tools_menu.addAction(open_pm_robot_config)
        # except:
        #     pass

        self.GuiParameterFileActions_instance = GuiParameterFileActions(self, 
                                                                        self.service_node, 
                                                                        self.action_sequence_builder,
                                                                        self.rsap_seq_info_widget)
        
        load_actionable_seq_params = QAction("Load Sequence Parameters File", self)
        load_actionable_seq_params.triggered.connect(self.GuiParameterFileActions_instance.load_sequence_parameters_file)
        load_actionable_seq_params.setToolTip("Load a .rsapp.json sequence parameters file to set parameters for the current action sequence.")
        new_actionable_seq_params = QAction("New Sequence Parameters File", self)
        new_actionable_seq_params.triggered.connect(self.GuiParameterFileActions_instance.create_new_sequence_parameters_file)
        new_actionable_seq_params.setToolTip("Create a new .rsapp.json sequence parameters file.")
        reset_actionable_seq_params = QAction("Reset Sequence Parameter Manager", self)
        reset_actionable_seq_params.triggered.connect(self.GuiParameterFileActions_instance.reset_sequence_parameter_manager)
        reset_actionable_seq_params.setToolTip("Reset the sequence parameter manager and clear all loaded sequence parameters.")
        save_actionable_seq_params = QAction("Save Sequence Parameter File", self)
        save_actionable_seq_params.triggered.connect(self.GuiParameterFileActions_instance.save_sequence_parameter_manager)
        save_actionable_seq_params.setToolTip("Save the current sequence parameters to the .rsapp.json file.")
        open_seq_param_widget = QAction("Open Parameter Manager", self)
        open_seq_param_widget.triggered.connect(self.open_sequence_parameter_window)
        open_seq_param_widget.setToolTip("Open the sequence parameter manager window.")

        seq_param_menu.addAction(open_seq_param_widget)
        seq_param_menu.addAction(load_actionable_seq_params)
        seq_param_menu.addAction(new_actionable_seq_params)
        seq_param_menu.addAction(save_actionable_seq_params)
        seq_param_menu.addAction(reset_actionable_seq_params)
        #seq_param_menu.addAction(open_seq_param_widget)

        executable_menu = QAction("ROS2 Run", self)
        executable_menu.triggered.connect(self.show_ros_executable_menu)
        ros_menu.addAction(executable_menu)
        
        executable_menu_launch = QAction("ROS2 Launch", self)
        executable_menu_launch.triggered.connect(self.show_ros_executable_launch_menu)
        ros_menu.addAction(executable_menu_launch)
                
        edit_configuration = QAction("Edit Configuration", self)
        edit_configuration.triggered.connect(self.open_config_editor)
        app_config_menu.addAction(edit_configuration)
        # open_pm_robot_tools = QAction("PM Robot Jog Panel", self)
        # open_pm_robot_tools.triggered.connect(partial(self.open_sub_window, PmDashboardApp))
        # pm_robot_tools_menu.addAction(open_pm_robot_tools)

        # Add vision manager to the menu, this will only be added if the package is found
        #append_vision_widget_to_menu(self, pm_robot_tools_menu, self.service_node)
        
        # Add jog panel to the menu, this will only be added if the package is found
        
        # try:
        #     append_jog_panel_to_menu(self, pm_robot_tools_menu, self.service_node)
        # except Exception as e:
        #     self.service_node.get_logger().warn(f"Error adding jog panel to menu: {e}")

        try:
            # open_pm_robot_co_pilot = QAction("PM Co-PilotProgramming", self)
            # open_pm_robot_co_pilot.triggered.connect(partial(self.open_sub_window, PmCoPilotAppProgramming))
            # copilot_menu.addAction(open_pm_robot_co_pilot)
            pass
        except (ModuleNotFoundError, NameError) as e:
            self.service_node.get_logger().error(f"Error adding PM Co-Pilot to menu: {e}")
              
        self.log_layout = QVBoxLayout()
        self.log_widget = QTreeWidget(self)
        self.log_widget.setHeaderLabel("Log Viewer")
        self.log_widget.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.log_widget.customContextMenuRequested.connect(self.show_log_context_menu)
        self.log_layout.addWidget(self.log_widget)

        self.export_log_button = QPushButton("Export Log")
        self.export_log_button.clicked.connect(self.export_log)
        
        self.clear_log_button = QPushButton("Clear Action Logs")
        self.clear_log_button.clicked.connect(self.clear_action_logs)
        
        self.log_layout.addWidget(self.export_log_button)
        self.log_layout.addWidget(self.clear_log_button)


        #add sublayout to app layout
        layout.addLayout(self.action_parameter_layout,1,2,Qt.AlignmentFlag.AlignTop)
        
        layout.addLayout(self.log_layout,1,3)
        #self.create_service_parameter_layout()
        central_widget.setLayout(layout)
        self.setGeometry(100, 100, 1800, 1200)
        self.execute_step_button.setEnabled(True)

    def openCoPilot(self):
        try:
            from pm_co_pilot_programming.submodules.PmCoPilotProgrammingApp import PmCoPilotProgrammingApp
            self.co_pilot_window = PmCoPilotProgrammingApp(self.service_node, rsap_instance=self.action_sequence_builder)
            # Connect signal to refresh GUI when sequence is modified
            self.co_pilot_window.sequence_modified.connect(self.refresh_action_list_from_copilot)
            self.co_pilot_window.show()
            self.service_node.get_logger().info("Co-Pilot Assistant opened with shared RSAP instance")
        except ModuleNotFoundError as e:
            self.service_node.get_logger().error(f"Failed to open Co-Pilot: {e}")
            QMessageBox.warning(self, "Module Not Found", "Co-Pilot module not found. Please ensure pm_co_pilot_programming is installed.")

    def openCoPilotPlanning(self):
        try:
            from pm_co_pilot_planning.submodules.PmCoPilotPlanningApp import PmCoPilotPlanningApp
            self.co_pilot_planning_window = PmCoPilotPlanningApp(self.service_node, rsap_instance=self.action_sequence_builder)
            self.co_pilot_planning_window.sequence_modified.connect(self.refresh_action_list_from_copilot)
            self.co_pilot_planning_window.show()
            self.service_node.get_logger().info("Co-Pilot Planning opened")
        except ModuleNotFoundError as e:
            self.service_node.get_logger().error(f"Failed to open Co-Pilot Planning: {e}")
            QMessageBox.warning(self, "Module Not Found", "Co-Pilot Planning module not found. Please ensure pm_co_pilot_planning is installed.")
    
    def refresh_action_list_from_copilot(self):
        """Refresh the action list display when Co-Pilot modifies the sequence"""
        current_row = self.action_list_widget.currentRow()
        self.action_list_widget.populate_list()
        # Try to maintain selection if possible
        if current_row >= 0 and current_row < len(self.action_sequence_builder.action_list):
            self.action_list_widget.setCurrentRow(current_row)
        self.service_node.get_logger().debug("Action list refreshed after Co-Pilot modification")
    
    def show_ros_executable_menu(self):
        self.ros_run_menu.menu_dictionary = self.action_sequence_builder.get_ros2_executables()
        self.ros_run_menu.init_action_menu()
        self.ros_run_menu.showMenu()

    def show_ros_executable_launch_menu(self):
        self.ros_launch_menu.menu_dictionary = self.action_sequence_builder.get_ros2_launch_executables()
        self.ros_launch_menu.init_action_menu()
        self.ros_launch_menu.showMenu()
            
    def run_ros_executable(self, command_list:list[str]):
        package = command_list[0]
        executable = command_list[1]
        command = self.action_sequence_builder.run_ros2_executable(package, executable, launch_mode=False)
        
        self.service_node.get_logger().info(f"Started command: '{command}' in a new terminal window.")

    def run_ros_executable_launch(self, command_list:list[str]):
        package = command_list[0]
        executable = command_list[1]
        command = self.action_sequence_builder.run_ros2_executable(package, executable, launch_mode=True)
        
        self.service_node.get_logger().info(f"Started command: '{command}' in a new terminal window.")
    
    def append_selected_action_from_menu(self, menu_output:list[str]):
        #print(menu_output)
        if menu_output[0] == 'Services':
            #print((menu_output[1]))
            if 'Clients' in menu_output[1] and 'Active' in menu_output[1]:
                self.append_service_dialog(service_client=menu_output[-1])
            elif 'Clients' in menu_output[1] and 'Memorised' in menu_output[1]:
                self.append_service_dialog(service_client = menu_output[-1],
                                           serivce_type=self.action_sequence_builder.get_srv_type_from_memorized_client(menu_output[-1]))
            elif 'Type' in menu_output[1]:
                self.append_service_dialog(serivce_type = f"{menu_output[-2]}/{menu_output[-1]}")
            elif menu_output[-1] == 'New':
                self.append_service_dialog()
        elif menu_output[0] == 'Actions':
            if 'Clients' in menu_output[1] and 'Active' in menu_output[1]:
                self.append_ros_action_service_dialog(type_action='action',client=menu_output[-1])
            elif 'Clients' in menu_output[1] and 'Memorised' in menu_output[1]:
                self.append_ros_action_service_dialog(type_action='action',client=menu_output[-1],
                                           type=self.action_sequence_builder.get_srv_type_from_memorized_client(menu_output[-1]))
            #elif 'Type' in menu_output[1]:
            #    self.append_ros_action_service_dialog(serivce_type = f"{menu_output[-2]}/{menu_output[-1]}")
            #elif menu_output[-1] == 'New':
            #    self.append_ros_action_service_dialog()

        elif menu_output[0]=='Other':
            if menu_output[-1] == 'User Interaction':
                self.append_user_interaction_dialog()
        else:
            self.text_output.append("This is not implemented yet!")
            self.service_node.get_logger().warn("This is not implemented yet!")


    def append_user_interaction_dialog(self):
        add_user_interaction_dialog = AddUserInteractionDialog()
        
        if add_user_interaction_dialog.exec():
            action_name, action_description, = add_user_interaction_dialog.get_values()
            self.add_user_interaction_to_action_list(action_name=action_name,
                                                     description=action_description)
        else:
            pass

    def add_user_interaction_to_action_list(self,action_name:str, description:str):
        pos_to_insert = self.action_list_widget.currentRow() + 1

        success = self.action_sequence_builder.append_user_interaction_to_action_list_at_index(index=pos_to_insert,
                                                                                                action_name=action_name,
                                                                                                action_description=description,
                                                                                                interaction_mode=GUI)
        # Get the name of the service from the currently acive action, which is the newly added one
        if success:
            service_name =  self.action_sequence_builder.get_current_action_name()
            self.action_list_widget.populate_list()
            self.text_output.append(f"Inserted action: {service_name}")
            self.action_selected()
        else:
            self.text_output.append_red_text(f"Invalid input arguments")
    
    def execute_current_action(self) -> bool:
        # connect the signals for the user interaction if not already connected
        self.pre_execution()
        index = self.action_list_widget.currentRow()
        # Return early if no function is selected
        if index == -1:
            return False
        
        success_set = self.action_sequence_builder.set_current_action(index)

        if success_set:
            _new_worker = RsapExecutionWorker(self.action_sequence_builder)
            _new_worker.signals.signal.connect(self.post_execute)
            self.thread_pool.start(_new_worker)

    def run_action_sequence(self) -> bool:
        # connect the signals for the user interaction if not already connected
        self.pre_execution()
        
        index = self.action_list_widget.currentRow()
        # Return early if no function is selected
        if index == -1:
            return False
        
        success_set = self.action_sequence_builder.set_current_action(index)

        if success_set:
            _new_worker = RsapExecutionRunWorker(self.action_sequence_builder, index)
            _new_worker.signals.signal.connect(self.post_execute)            
            self.thread_pool.start(_new_worker)
            
            
    def pre_execution(self):
        self.status_indicator.set_state_running()
        self._init_user_interaction_signals()
    
    def post_execute(self):
        self.action_list_widget.setCurrentRow(self.action_sequence_builder.current_action_index)
        self.action_selected()
        self.set_action_colors_from_execution_status(set_state=True)
        self.text_output.append_black_text("Execution finished!")
        
    def set_action_colors_from_execution_status(self, set_state=False):
        if set_state:
            self.status_indicator.set_state_success()
                
        for index, action in enumerate(self.action_sequence_builder.action_list):
            success = action.log_entry.get('success', None)
            if success == None:
                self.action_list_widget.item(index).setBackground(QColor("white"))
            elif success:
                light_green = QColor(144, 238, 144)
                self.action_list_widget.item(index).setBackground(light_green)
            else:
                self.action_list_widget.item(index).setBackground(QColor("red"))
                if set_state:
                    self.status_indicator.set_state_error()

    def reset_action_list_status(self):
        self.action_sequence_builder.clear_all_log_entries()
        self.set_action_colors_from_execution_status(set_state=False)

    
    def action_selected(self, active=True):
        row = self.action_list_widget.currentRow()
        self.action_parameter_layout.clear_action_parameter_layout()
        self.clear_log_viewer()

        action = self.action_sequence_builder.get_action_at_index(row)
        
        self.current_action_parameter_widget = ActionParameterWidget(action,
                                                                     self.action_sequence_builder.action_parameter_value_manager,
                                                                     logger=self.service_node.get_logger(),
                                                                     active=active)
        
        self.action_parameter_layout.addWidget(self.current_action_parameter_widget)
        self.current_action_parameter_widget.on_action_changed(self.action_parameter_changed)

        self.show_service_log(action.log_entry)

    def action_parameter_changed(self):
        row = self.action_list_widget.currentRow()
        #self.service_node.get_logger().warn("Action changed:")
        self.action_list_widget.populate_list()
        self.action_list_widget.setCurrentRow(row)
        self.action_selected()
        self.set_action_colors_from_execution_status(set_state=False)
            
    def add_ros_action_to_action_list(self, action_name: str, action_client:str, action_type:str = None) -> None:

        pos_to_insert = self.action_list_widget.currentRow() + 1

        if action_client:
            success = self.action_sequence_builder.append_ros_action_to_action_list_at_index(action_client = action_client, 
                                                                                    index = pos_to_insert, 
                                                                                    action_type = action_type,
                                                                                    action_name = action_name)
            # Get the name of the service from the currently acive action, which is the newly added one
            if success:
                action_name =  self.action_sequence_builder.get_current_action_name()
                self.action_list_widget.populate_list()
                self.action_list_widget.setCurrentRow(pos_to_insert)
                self.text_output.append(f"Inserted action: {action_name}")
                self.action_selected()
            else:
                self.text_output.append_red_text(f"Invalid input arguments")
                
                
    def open_process_file_dia(self):
        """
        This method opens a file dialog to select a process file to open.
        """
        
        file_filter = "JSON Files (*.json);;All Files (*)"
        file_path, _ = QFileDialog.getOpenFileName(self, "Open JSON File", 
                                                   "", 
                                                   file_filter)
        self.open_process_file(file_path)


    def open_process_file(self, file_path:str):
        """
        This method opens a file dialog to select a process file to open.
        """
        
        if file_path:
            self.text_output.clear()
            self.text_output.append(f"Opening: {file_path}")
            success = self.action_sequence_builder.rsap_file_manager.load_from_JSON(file_path)
            self.recent_files_manager.set_recent_file()
            if success:
                self.action_list_widget.populate_list()
                self.rsap_seq_info_widget.init_values()

                self.text_output.append("File loaded!")
            else:
                self.text_output.append("Error Opening File!")
                self.action_list_widget.populate_list()
                self.action_parameter_layout.clear_action_parameter_layout()

        # Set the first row as the current process
        self.action_list_widget.setCurrentRow(0)
        # Set the last saved timestap
        self.update_last_saved()

    def save_process_as(self):
        self.create_new_file(save_as=True)

    def save_process(self):
        """
        This method saves the current process to a file.
        """
        if self.action_sequence_builder.rsap_file_manager.get_sequence_name() is None:
            self.create_new_file()
        else:
            success = self.action_sequence_builder.rsap_file_manager.save_to_JSON()
            self.text_output.append(f"Saved file: {success}")
            self.update_last_saved()
            
    def create_new_file(self, save_as=False):
        """
        This method creates a new file and saves the current process to it.
        """
        # if self.current_vision_pipeline.vision_pipeline_json_dir == None:
        #     self.current_vision_pipeline.vision_pipeline_json_dir = self.default_process_libary_path

        # Set the file filters to show only JSON files
        file_filter = "JSON Files (*.json)"
        file_name, _ = QFileDialog.getSaveFileName(self, "Save JSON File", "", file_filter)

        # this is for the case the user entered .json to his filename 
        file_name = os.path.splitext(file_name)[0]

        # set action sequence file path
        self.action_sequence_builder.rsap_file_manager.set_folder_path(os.path.dirname(file_name))

        if not save_as and self.action_sequence_builder.rsap_file_manager.get_sequence_name() is not None:
            self.action_sequence_builder.action_list.clear()
            #self.init_actions_list()
            self.action_list_widget.populate_list()

        if file_name:
            # set action sequence name
            self.action_sequence_builder.rsap_file_manager.set_sequence_name(os.path.basename(os.path.splitext(file_name)[0]))
            success = self.action_sequence_builder.rsap_file_manager.save_to_JSON()
            self.text_output.append(f"Saved file: {success}")
            # set text in gui from action sequence name
            self.rsap_seq_info_widget.init_values()
            # update the last saved timestamp
            self.update_last_saved()
            self.recent_files_manager.set_recent_file()

        self.action_parameter_layout.clear_action_parameter_layout()

    def append_service_dialog(self, service_name: str = None, service_client:str= None, serivce_type:str = None)->None:
        add_service_dialog = AddServiceDialog(service_name, service_client, serivce_type)
        
        if add_service_dialog.exec():
            service_name, service_client, type = add_service_dialog.get_values()
            self.add_service_to_action_list(service_name=service_name,
                                            service_client=service_client,
                                            service_type=type)
        else:
            pass

    def append_ros_action_service_dialog(self, type_action: str, name: str = None, client:str= None, type:str = None)->None:
        """
        type should be either 'action' or 'service'
        """
        add_action_dialog = AddServiceDialog(name, client, type)
        
        if add_action_dialog.exec():
            name, client, type = add_action_dialog.get_values()
            self.add_ros_action_to_action_list(action_name = name,
                                            action_client = client,
                                            action_type = type)
        else:
            pass

    def interrupt_action(self) -> None:
        interupt_dialog = UserDialog(request_text="Do you want to interrupt the current action? This will interrupt the execution of the current action. However if the current action is a ros2 service call, the call might finish in the background!")
        
        if interupt_dialog.exec():
            self.text_output.append_red_text("Interruption requested! Interrupting action!")
            self.action_sequence_builder.set_interrupt_execution(True)
        else:
            pass

    def add_service_to_action_list(self, service_name: str, service_client:str, service_type:str = None) -> None:
        pos_to_insert = self.action_list_widget.currentRow() + 1

        if service_client:
            success = self.action_sequence_builder.append_service_to_action_list_at_index(service_client=service_client, 
                                                                                    index = pos_to_insert, 
                                                                                    service_type=service_type,
                                                                                    service_name = service_name)
            # Get the name of the service from the currently acive action, which is the newly added one
            if success:
                service_name =  self.action_sequence_builder.get_current_action_name()
                
                self.action_list_widget.populate_list()
                self.action_list_widget.setCurrentRow(pos_to_insert)
                self.text_output.append(f"Inserted action: {service_name}")
                self.action_selected()
            else:
                self.text_output.append_red_text(f"Invalid input arguments")
                    
    def update_last_saved(self):
        self.last_saved_label.setText("Saved at: " + datetime.now().strftime("%H:%M:%S"))

    def show_service_log(self, dictionary):
        if not dictionary:
            return     

        self.populate_log_widget(dictionary)

    def populate_log_widget(self, data, parent=None):
        if parent is None:
            parent = self.log_widget

        if isinstance(data, dict):
            for key, value in data.items():
                item = QTreeWidgetItem(parent, [str(key)])
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsSelectable)
                self.populate_log_widget(value, item)
        elif isinstance(data, list):
            for index, item_data in enumerate(data):
                item = QTreeWidgetItem(parent, [f"[{index}]"])
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsSelectable)
                self.populate_log_widget(item_data, item)
        else:
            item = QTreeWidgetItem(parent, [str(data)])
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsSelectable)   

    def clear_log_viewer(self):
        self.log_widget.clear()

    def show_log_context_menu(self, position):
        """Display a context menu for the log widget with copy option"""
        item = self.log_widget.itemAt(position)
        if item is None:
            return
        
        context_menu = QMenu(self)
        copy_action = QAction("Copy", self)
        copy_action.triggered.connect(lambda: self.copy_log_item(item))
        context_menu.addAction(copy_action)
        
        context_menu.exec(self.log_widget.mapToGlobal(position))

    def copy_log_item(self, item):
        """Copy the text of a log item to clipboard"""
        if item is None:
            return
        
        text = item.text(0)
        clipboard = QApplication.clipboard()
        clipboard.setText(text)
        self.text_output.append_green_text(f"Copied: {text}")

    def copy_and_insert_actions(self):

        selected_indexes = self.action_list_widget.selectedIndexes()
        selected_rows_indexes = [index.row() for index in selected_indexes]
        self.action_sequence_builder.copy_actions_from_index_list_and_insert(selected_rows_indexes)
        #self.action_sequence_builder.copy_action_at_index_and_insert(index)
        #self.init_actions_list()
        self.action_list_widget.populate_list()
        self.action_list_widget.setCurrentRow(max(selected_rows_indexes)+1)
        self.action_selected()

    def open_sub_window(self, window_class):
        # this cbk can only be used for classes inhereting from QWidget. 
        # The classes should have one optional argument, which is the a rclpy.node.Node
        try:
            self.new_window = window_class(self.service_node)
            self.new_window.show()
        except ModuleNotFoundError as e:
            self.service_node.get_logger().error(f"Error opening sub window: {e}")
        except PackageNotFoundError as e:
            self.service_node.get_logger().error(f"Error opening sub window: {e}")
        except Exception as e:
            self.service_node.get_logger().error(f"Error opening sub window: {e}")

    def update_recent_files_menu(self):
        """
        Clears and repopulates the 'Open Recent Process' submenu
        using the last 10 recent files from RecentFilesManager.
        """
        self.recent_files_menu.clear()

        recent_files = self.recent_files_manager.get_recent_files()  # List of last files
        if not recent_files:
            empty_action = QAction("No recent files", self)
            empty_action.setEnabled(False)
            self.recent_files_menu.addAction(empty_action)
            return

        for file_path in recent_files:
            action = QAction(file_path, self)
            # Directly call your open_process_file function
            action.triggered.connect(lambda checked, f=file_path: self.open_process_file(f))
            self.recent_files_menu.addAction(action)

    def open_sequence_parameter_window(self):
        if not self.action_sequence_builder.action_parameter_value_manager.seq_parameter_manager.get_is_initialized():
            self.text_output.append_red_text("No sequence parameter file loaded! Create or load a sequence parameter file first.")
            return
        
        dialog = SeqParameterManagerDialog(self.action_sequence_builder.action_parameter_value_manager.seq_parameter_manager,
                                           logger=self.service_node.get_logger(),
                                           parent=self)
        dialog.exec()

        self.action_sequence_builder.action_parameter_value_manager.seq_parameter_manager.save_to_file()

    def print_ros_log(self, msg, level):
        # INFO
        if level == 20: 
            self.text_output.append_black_text("[INFO]" + msg)
        # WARN
        if level == 30:
            self.text_output.append_orange_text("[WARN]" +msg)
        # ERROR
        if level == 40:
            self.text_output.append_red_text("[ERROR]" +msg)
        # DEBUG
        if level == 10:
            self.text_output.append_blue_text("[DEBUG]" +msg)

    def set_widget_activations(self, state):
        # This method activates/deactivates the buttons in the GUI when the action sequence is running
        #self.text_output.append_red_text(f"{state}")
        self.run_action_sequence_button.setEnabled(not state)
        self.execute_step_button.setEnabled(not state)
        self.copy_and_insert_button.setEnabled(not state)
        self.delete_button.setEnabled(not state)
        self.open_action_menu_button.setEnabled(not state)
        self.action_list_widget.setEnabled(not state)
        self.action_list_widget.setAcceptDrops(not state)
        self.action_list_widget.setDragEnabled(not state)
        self.action_list_widget.setDropIndicatorShown(not state)

    def select_action_in_execution(self, index):
        self.action_list_widget.setCurrentRow(index)
        self.set_action_colors_from_execution_status()
        self.action_selected(False)

    def create_user_interaction_dialog(self):
        add_user_interaction_dialog = AddUserInteractionDialog()
        
        if add_user_interaction_dialog.exec():
            action_name, action_description, = add_user_interaction_dialog.get_values()
            self.add_user_interaction_to_action_list(action_name=action_name,
                                                     description=action_description)
        else:
            pass
    
    def export_log(self):
        self.action_sequence_builder.save_action_sequence_log()
        self.text_output.append_green_text("Exported action log to file!")

    def clear_action_logs(self):
        # dialog to confirm clearing logs
        confirm_dialog = UserDialog(request_text="Are you sure you want to clear all action logs? This action cannot be undone.")
        if not confirm_dialog.exec():
            return
        self.action_sequence_builder.clear_all_log_entries()
        self.clear_log_viewer()
        self.text_output.append_green_text("Cleared all action logs!")
        
    @pyqtSlot(str, object)   
    def show_user_interaction_dialog(self, message: str, result_holder: dict):
        # Create and show the user interaction dialog
        dialog = UserInteractionActionDialog(message)
        
        dialog_result = dialog.exec() == QDialog.DialogCode.Accepted

        # Update the result holder so the thread can proceed
        result_holder["result"] = dialog_result
        
    def _init_user_interaction_signals(self):
        """
        Connects the signals for the user interaction actions if they are not already connected. This is needed because window must be shown in the main thread.
        """
        for ind, action in enumerate(self.action_sequence_builder.action_list):
            if isinstance(action, UserInteractionAction):
                action: UserInteractionAction
                if action.open_user_interaction_signal.signal not in self.user_interaction_signals_list:
                    self.user_interaction_signals_list.append(action.open_user_interaction_signal.signal)
                    action.open_user_interaction_signal.signal.connect(self.show_user_interaction_dialog)
    
    def open_config_editor(self):
        """Open the configuration editor and handle changes."""
        config_editor = NestedDictionaryEditor(self.action_sequence_builder.config.get_as_dict(), self)
        result = config_editor.exec()
        
        if result == QDialog.DialogCode.Accepted:
            result_config = config_editor.dictionary
            self.action_sequence_builder.config.set_from_dict(result_config)
            #self.apply_config_changes(result_config)
            self.text_output.append_green_text("Configuration updated!")

    def apply_config_changes(self, dictionary: dict):
        """Apply changes to the app based on the updated configuration."""
        QMessageBox.information(
            self,
            "Configuration Updated",
            f"New Configuration:\n{dictionary}",
            QMessageBox.StandardButton.Ok,
        ) 
                    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RsapApp()
    window.show()
    sys.exit(app.exec())
