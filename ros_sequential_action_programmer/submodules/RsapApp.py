from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtWidgets import QScrollArea, QDialog, QHBoxLayout, QInputDialog, QTreeWidget, QTreeWidgetItem, QApplication, QGridLayout, QFrame, QMainWindow, QListWidget, QListWidgetItem, QDoubleSpinBox, QWidget, QVBoxLayout, QPushButton, QCheckBox, QLineEdit, QComboBox, QTextEdit,QLabel,QSlider, QSpinBox, QFontDialog, QFileDialog
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction
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
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer, SET_IMPLICIT_SRV_DICT, LOG_AT_END, LOG_NEVER
from ros_sequential_action_programmer.submodules.RsapApp_submodules.PopupRecWindow import PopupRecWindow
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionSelectionMenu import ActionSelectionMenu
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AddServiceDialog import AddServiceDialog
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AddUserInteractionDialog import AddUserInteractionDialog
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from rosidl_runtime_py.get_interfaces import get_service_interfaces
import ast

from ros_sequential_action_programmer.submodules.pm_robot_modules.widget_pm_robot_config import PmRobotConfigWidget
from ros_sequential_action_programmer.submodules.pm_robot_modules.widget_pm_robot_dashboard import PmDashboardApp

# Change this later to be cleaner
try:
    from ros_sequential_action_programmer.submodules.co_pilot.PmCoPilotApp import PmCoPilotApp
except ModuleNotFoundError as e:
    print(f"Error importing PmCoPilotApp: {e}")

from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI

class RsapApp(QMainWindow):
    def __init__(self, service_node:Node):
        super().__init__()
        self.service_node = service_node
        
        self.action_sequence_builder = RosSequentialActionProgrammer(service_node)
        # self.action_sequence_builder.append_service_to_action_list('/pm_robot_gonio_left_controller/get_parameter_types')
        # self.action_sequence_builder.append_service_to_action_list('/get_planning_scene')
        # self.action_sequence_builder.append_service_to_action_list('/compute_fk')
        # self.action_sequence_builder.append_service_to_action_list(service_client='/object_manager/create_ref_frame', service_type='spawn_object_interfaces/srv/CreateRefFrame')
        # load the recent file (last opened file)
        self.load_recent_file()
        #print(self.action_sequence_builder.get_possible_srv_res_fields_at_index(index=3, target_key='frame_name'))
        self.initUI()
        self.init_actions_list()
        self.save_as = False 
        self.stop_execution = False
        #self.initialize_active_service_list()
        self.execution_running = False
        #self.action_exectuion_thread = QThread(parent=service_node.executor)
        #self.action_exectuion_thread.run = self.execute_current_action
        #self.action_exectuion_thread.finished.connect(self.end_current_action_execution)
        self.sub_window_list = []


    def initUI(self):
        # Create the class for the action selection menu
        self.action_menu = ActionSelectionMenu(self)
        # Overriting the callbackfunction for the menu
        self.action_menu.action_menu_clb = self.append_selected_action_from_menu
        
        self.setWindowTitle("Ros Sequential Action Programmer - RSAP")
        
        # Create main container widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Create layout for the main container widget
        layout = QGridLayout()

        self.action_sequence_name_widget = QLabel()
        font = QFont()
        font.setPointSize(14)
        self.action_sequence_name_widget.setFont(font)
        if self.action_sequence_builder.name is None:
            self.set_widget_action_sequence_name('- No name given - ')
        else:
            self.set_widget_action_sequence_name(self.action_sequence_builder.name)
        layout.addWidget(self.action_sequence_name_widget,0,1,1,2)

        toolbar_layout = QVBoxLayout()
        open_action_menu_button = QPushButton('Add \n Action')
        open_action_menu_button.clicked.connect(self.show_action_menu)
        open_action_menu_button.setFixedSize =(20,20)

        toolbar_layout.addWidget(open_action_menu_button,alignment=Qt.AlignmentFlag.AlignTop)

        # add button for deleting selected vision function
        delete_button = QPushButton("Delete \n Selected")
        delete_button.clicked.connect(self.delete_action_from_sequence)
        toolbar_layout.addWidget(delete_button,alignment=Qt.AlignmentFlag.AlignTop)

        copy_and_insert_button = QPushButton("Copy and \n Insert")
        copy_and_insert_button.clicked.connect(self.copy_and_insert_actions)
        toolbar_layout.addWidget(copy_and_insert_button,alignment=Qt.AlignmentFlag.AlignTop)

        layout.addLayout(toolbar_layout,1,0,alignment=Qt.AlignmentFlag.AlignTop)

        execute_layout = QHBoxLayout()
        # Add combobox for vision pipline building
        self.checkbox_list = ReorderableCheckBoxListWidget()
        self.checkbox_list.itemClicked.connect(self.action_selected)
        #self.checkbox_list.itemChanged.connect(self.set_function_states)
        self.checkbox_list.CustDragSig.connect(self.on_action_drag_drop)
        layout.addWidget(self.checkbox_list,1,1)

        self.execute_step_button = QPushButton("Step")
        self.execute_step_button.clicked.connect(self.execute_current_action)
        #self.execute_step_button.clicked.connect(self.initiate_current_action_execution)
        execute_layout.addWidget(self.execute_step_button)

        self.run_action_sequence_button = QPushButton("Run")
        self.run_action_sequence_button.clicked.connect(self.run_action_sequence)
        execute_layout.addWidget(self.run_action_sequence_button)
        layout.addLayout(execute_layout,2,1)

        stop_execution_button = QPushButton("Stop Execution")
        stop_execution_button.clicked.connect(self.stop_execution)
        layout.addWidget(stop_execution_button,3,1)

        # add textbox for string output
        self.text_output = AppTextOutput()
        layout.addWidget(self.text_output,4,1,1,3)

        self.last_saved_label = QLabel('File not saved yet!')
        layout.addWidget(self.last_saved_label,5,1,1,2)

        # # create sub layout for the function parameter widgetd
        self.sub_layout = QVBoxLayout()

        #add a textlabel to the sublayout
        action_parameters_label = QLabel('Action parameters:')
        action_parameters_label.setFont(font)
        self.sub_layout.addWidget(action_parameters_label)

        # Create a menu bar
        menubar = self.menuBar()
        menubar.setStyleSheet("QMenuBar { font-size: 18px; }")
        file_menu = menubar.addMenu("File")
        app_config_menu = menubar.addMenu("App Config")
        pm_robot_tools_menu = menubar.addMenu("PM Robot Tools")

        # Create "New" action
        new_action = QAction("New process", self)
        new_action.triggered.connect(self.create_new_file)
        file_menu.addAction(new_action)

        open_action = QAction("Open process", self)
        open_action.triggered.connect(self.open_process_file)
        file_menu.addAction(open_action)

        save_action = QAction("Save", self)
        save_action.triggered.connect(self.save_process)
        file_menu.addAction(save_action)

        # Create 'save as' action
        save_as_action = QAction("Save process as", self)
        save_as_action.triggered.connect(self.save_process_as)
        file_menu.addAction(save_as_action)
       
        open_pm_robot_config = QAction("PM Robot Config", self)
        open_pm_robot_config.triggered.connect(partial(self.open_sub_window, PmRobotConfigWidget))
        pm_robot_tools_menu.addAction(open_pm_robot_config)

        open_pm_robot_tools = QAction("PM Robot Jog Panel", self)
        open_pm_robot_tools.triggered.connect(partial(self.open_sub_window, PmDashboardApp))
        pm_robot_tools_menu.addAction(open_pm_robot_tools)

        try:
            open_pm_robot_co_pilot = QAction("PM Co-Pilot", self)
            open_pm_robot_co_pilot.triggered.connect(partial(self.open_sub_window, PmCoPilotApp))
            pm_robot_tools_menu.addAction(open_pm_robot_co_pilot)
        except ModuleNotFoundError as e:
            self.service_node.get_logger().error(f"Error adding PM Co-Pilot to menu: {e}")
        except NameError as e:
            self.service_node.get_logger().error(f"Error adding PM Co-Pilot to menu: {e}")

        self.log_layout = QVBoxLayout()
        self.log_widget = QTreeWidget(self)
        self.log_widget.setHeaderLabel("Log Viewer")
        self.log_layout.addWidget(self.log_widget)

        #add sublayout to app layout
        layout.addLayout(self.sub_layout,1,2,Qt.AlignmentFlag.AlignTop)
        
        layout.addLayout(self.log_layout,1,3)
        self.create_service_parameter_layout()
        central_widget.setLayout(layout)
        self.setGeometry(100, 100, 1800, 1200)
        self.execute_step_button.setEnabled(True)

    def openCoPilot(self):
        self.co_pilot_window = PmCoPilotApp(self.service_node, self.action_sequence_builder)
        self.co_pilot_window.show()

    def show_action_menu(self):
        self.action_sequence_builder.initialize_service_list()
        self.action_sequence_builder.save_all_service_req_res_to_JSON()
        self.action_menu.menu_dictionary= {
            'Services': {
                'Empty':  ['New'],
                'Active Clients blk':  self.action_sequence_builder.list_of_clients_to_dict(self.action_sequence_builder.get_active_client_blklist()),
                'Active Clients':  self.action_sequence_builder.list_of_clients_to_dict(self.action_sequence_builder.list_of_active_clients),
                'Active Clients wht':  self.action_sequence_builder.list_of_clients_to_dict(self.action_sequence_builder.get_active_client_whtlist()),
                'Memorised Clients blk':  self.action_sequence_builder.list_of_clients_to_dict(self.action_sequence_builder.get_memorized_client_blklist()),
                'Memorised Clients ':  self.action_sequence_builder.list_of_clients_to_dict(self.action_sequence_builder.get_list_memorized_service_clients()),
                'Memorised Clients wht':  self.action_sequence_builder.list_of_clients_to_dict(self.action_sequence_builder.get_memorized_client_whitelist()),
                'Available Service Types':  get_service_interfaces(),
            },
            'Skills': ['TBD1','TBD2','TBD3'],
            'Other': {
                'Conditions': {
                    'Options': ['Option7', 'Option8', 'Option9']
                },
                'Operation': ['User Interaction']
            }
        }
        self.action_menu.init_action_menu()
        self.action_menu.showMenu()

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
        pos_to_insert = self.checkbox_list.currentRow() + 1

        success = self.action_sequence_builder.append_user_interaction_to_action_list_at_index(index=pos_to_insert,
                                                                                                action_name=action_name,
                                                                                                action_description=description,
                                                                                                interaction_mode=GUI)
        # Get the name of the service from the currently acive action, which is the newly added one
        if success:
            service_name =  self.action_sequence_builder.get_current_action_name()
            function_checkbox = ReorderableCheckBoxListItem(f"{pos_to_insert}. {service_name}")
            self.checkbox_list.insertItem(pos_to_insert,function_checkbox)
            self.init_actions_list()
            self.checkbox_list.setCurrentRow(pos_to_insert)
            self.text_output.append(f"Inserted action: {service_name}")
            self.action_selected()
        else:
            self.text_output.append_red_text(f"Invalid input arguments")

    def init_actions_list(self):
        """
        Updates the action list widget with the current actions from the action sequence.
        """
        self.checkbox_list.clear()
        for index, action in enumerate(self.action_sequence_builder.action_list):
            function_checkbox = ReorderableCheckBoxListItem(f"{index}. {action.name}")
            self.checkbox_list.addItem(function_checkbox)

    # def initiate_current_action_execution(self):
    #     #self.set_gui_interactionable(False)
    #     self.action_exectuion_thread.start()
    
    # def end_current_action_execution(self):
    #     #self.set_gui_interactionable(True)
    #     pass
    
    def execute_current_action(self) -> bool:
        
        index = self.checkbox_list.currentRow()
        # Return early if no function is selected
        if index == -1:
            return False
        
        success_set = self.action_sequence_builder.set_current_action(index)
        success = False
        if success_set:
            success = self.action_sequence_builder.execute_current_action(log_mode=LOG_AT_END)
        if success:
            text_output = f"Action '{self.action_sequence_builder.get_current_action_name()}' executed successfully!"
            self.text_output.append_green_text(text_output)
            light_green = QColor(144, 238, 144)
            self.checkbox_list.currentItem().setBackground(light_green)
            if index < self.checkbox_list.count()-1:
                self.checkbox_list.setCurrentRow(index+1)
            else:
                self.checkbox_list.setCurrentRow(0)
            self.action_selected()
        else:
            self.text_output.append_red_text("An error occured in action execution!")
            self.checkbox_list.currentItem().setBackground(QColor("red"))
            self.action_selected()
        return success

    def run_action_sequence(self):
        error = False

        start_index = self.checkbox_list.currentRow()
        end_index = self.checkbox_list.count()

        # Return early if no function is selected
        if start_index == -1:
            return False

        for start_index in range(start_index, end_index):
            if not error or self.stop_execution:
                error = not self.execute_current_action()
            else:
                break

    def set_widget_action_sequence_name(self, text):
        self.action_sequence_name_widget.setText("Process name: " + text)
        self.action_sequence_name_widget.setToolTip(self.action_sequence_builder.action_file_path)

    def create_service_parameter_layout(self):
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.inner_widget = QWidget()
        self.inner_layout = QVBoxLayout()

        apply_botton = QPushButton("Apply Changes")
        apply_botton.clicked.connect(self.apply_changes_to_service)

        self.inner_widget.setLayout(self.inner_layout)
        self.scroll_area.setWidget(self.inner_widget)
        self.sub_layout.addWidget(apply_botton)
        self.sub_layout.addWidget(self.scroll_area)
    
    def set_service_meta_info_widget(self):
        row = self.checkbox_list.currentRow()

        label_action_name = QLabel("Action Name:")
        self.action_name_edit = QLineEdit(self.action_sequence_builder.get_action_at_index(row).name)
        self.inner_layout.addWidget(label_action_name)
        self.inner_layout.addWidget(self.action_name_edit)    

        # Set info for service client
        label_action_client_desc = QLabel(f"Service Client: ")
        label_action_client = QLineEdit(f"{self.action_sequence_builder.get_action_at_index(row).client}")
        label_action_client.setReadOnly(True)
        self.inner_layout.addWidget(label_action_client_desc)
        self.inner_layout.addWidget(label_action_client)

        # Set info for service type
        label_action_service_desc = QLabel(f"Service Type: ")
        label_action_service_type = QLineEdit(f"{self.action_sequence_builder.get_action_at_index(row).service_type}")
        label_action_service_type.setReadOnly(True)
        self.inner_layout.addWidget(label_action_service_desc)
        self.inner_layout.addWidget(label_action_service_type)

        # Set info for service type
        label_action_description_desc = QLabel(f"Description: ")
        self.label_action_description = QTextEdit(f"{self.action_sequence_builder.get_action_at_index(row).description}")
        self.inner_layout.addWidget(label_action_description_desc)
        self.inner_layout.addWidget(self.label_action_description)

        # Create a dropdown menu for selecting the error handling inputs
        label_error_handling_box = QLabel(f"Execution identifier: ")
        self.error_handling_box = QComboBox()

        box_values = ['None'] + self.action_sequence_builder.get_action_at_index(row).get_service_bool_fields()
        self.error_handling_box.addItems(box_values)
        # Set the default value when creating the qcombobox
        currently_set_error_handler = self.action_sequence_builder.get_action_at_index(row).get_service_bool_identifier()
        if currently_set_error_handler is None:
            currently_set_error_handler = "None"

        self.error_handling_box.setCurrentIndex(box_values.index(currently_set_error_handler))
        self.inner_layout.addWidget(label_error_handling_box)
        self.inner_layout.addWidget(self.error_handling_box)

    def on_action_drag_drop(self):
        self.action_sequence_builder.move_action_at_index_to_index(old_index=self.checkbox_list.drag_source_position,
                                                            new_index=self.checkbox_list.currentRow())
        self.init_actions_list()
        self.action_selected()

    def action_selected(self):
        row = self.checkbox_list.currentRow()
        self.clear_action_parameter_layout()
        self.clear_log_viewer()

        if isinstance(self.action_sequence_builder.get_action_at_index(row), ServiceAction):
            self.handle_dict = None
            self.handle_dict = self.action_sequence_builder.get_copy_impl_srv_dict_at_index(row)
            self.clear_action_parameter_layout()
            self.clear_log_viewer()
            self.set_service_meta_info_widget()
            self.populateActionParameterWidgets(self.handle_dict)

        self.show_service_log(self.action_sequence_builder.get_action_at_index(row).log_entry)

    def populateActionParameterWidgets(self, data, parent_key=None):
        """
        This method populates the action parameter layout with the parameters of the selected action.
        """
        # iterate through the dict
        for key, value in data.items():
            if parent_key is not None:
                full_key = parent_key + '.' + key
            else:
                full_key = key

            if isinstance(value, OrderedDict):
                self.populateActionParameterWidgets(value, full_key)
            else:
                widget_with_button = QLineButton(full_key=full_key, 
                                                 initial_value=str(value), 
                                                 on_text_changed=self.updateDictionary, 
                                                 on_button_clicked=self.get_recom_button_clicked)
                
                self.inner_layout.addWidget(widget_with_button)

    def get_recom_button_clicked(self, key, widget:QLineEdit):
        """
        This method is called when the user clicks the button to get recommendations for a specific parameter.
        """
        index = self.checkbox_list.currentRow()
        data = self.action_sequence_builder.get_recom_for_action_at_index_for_key(index=index, key=key)
        popup = PopupRecWindow(data)
        result = popup.exec()

        if result == QDialog.DialogCode.Accepted:
            selected_value = popup.get_selected_value()
            widget.setText(selected_value)
            print(f"Selected value: {selected_value}")

    def apply_changes_to_service(self):
        index = self.checkbox_list.currentRow()
        # if no line selected index will be -1
        if index != -1:
            # Apply error handler message
            error_identifier = self.error_handling_box.currentText()
            if error_identifier == 'None':
                error_identifier = None

            # set the error identifier
            set_error_identifier_success = self.action_sequence_builder.get_action_at_index(index).set_service_bool_identifier(error_identifier)
            
            # Set the action description text
            self.action_sequence_builder.get_action_at_index(index).description = self.label_action_description.toPlainText()

            # Apply values to service request dict
            self.action_sequence_builder.get_action_at_index(index).name = self.action_name_edit.text()
            set_success = self.action_sequence_builder.process_action_dict_at_index(index=index,mode=SET_IMPLICIT_SRV_DICT, input_impl_dict=self.handle_dict)

            if set_success and set_error_identifier_success:
                self.text_output.append("Success changing values!")
            else:
                self.text_output.append_red_text("Error occured changing changes!")

            self.init_actions_list()
            self.checkbox_list.setCurrentRow(index)
            self.action_selected()

    def clear_action_parameter_layout(self):
        """
        This method clears the action parameter layout.
        """
        if self.inner_layout is not None:
            while self.inner_layout.count():
                item = self.inner_layout.takeAt(0)
                widget = item.widget()
                if widget:
                    widget.deleteLater()
                else:
                    self.clear_action_parameter_layout(item.layout())

    def updateDictionary(self, key, edit):
        def handleTextChange(text):
            try:
                keys = key.split('.')
                current_dict = self.handle_dict
                for k in keys[:-1]:
                    current_dict = current_dict[k]

                # Convert the input text to the appropriate data type
                value = text
                if '.' in text and all(part.replace('.', '').lstrip('-').isdigit() for part in text.split('.', 1)):
                    # Modified condition to allow negative floats
                    value = float(text)
                elif text.lstrip('-').isdigit():
                    value = int(text)
                elif text == 'True':
                    value = True
                elif text == 'False':
                    value = False
                elif text[0] == '[' and text[-1] == ']':
                    value = ast.literal_eval(text)
                elif text == 'None':
                    value = None

                current_dict[keys[-1]] = value
            except ValueError:
                pass

        return handleTextChange

    def open_process_file(self):
        """
        This method opens a file dialog to select a process file to open.
        """

        # if self.current_vision_pipeline.vision_pipeline_json_dir == None:
        #     self.current_vision_pipeline.vision_pipeline_json_dir = self.default_process_libary_path
        
        file_filter = "JSON Files (*.json);;All Files (*)"
        file_path, _ = QFileDialog.getOpenFileName(self, "Open JSON File", 
                                                   "", 
                                                   file_filter)
        if file_path:
            self.text_output.clear()
            self.text_output.append(f"Opening: {file_path}")
            success = self.action_sequence_builder.load_from_JSON(file_path)
            self.set_recent_file()
            if success:
                self.init_actions_list()
                self.set_widget_action_sequence_name(self.action_sequence_builder.name)
                self.text_output.append("File loaded!")
            else:
                self.text_output.append("Error Opening File!")
                self.init_actions_list()
                self.clear_action_parameter_layout()

        # Set the first row as the current process
        self.checkbox_list.setCurrentRow(0)
        # Set the last saved timestap
        self.update_last_saved()

    def save_process_as(self):
        self.save_as = True
        self.create_new_file()

    def save_process(self):
        """
        This method saves the current process to a file.
        """
        if self.action_sequence_builder.name is None:
            self.create_new_file()
        else:
            success = self.action_sequence_builder.save_to_JSON()
            self.text_output.append(f"Saved file: {success}")
            self.update_last_saved()
            
    def create_new_file(self):
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
        self.action_sequence_builder.folder_path = os.path.dirname(file_name)

        if not self.save_as and self.action_sequence_builder.name is not None:
            self.action_sequence_builder.action_list.clear()
            self.init_actions_list()

        if file_name:
            # set action sequence name
            self.action_sequence_builder.name  = os.path.basename(os.path.splitext(file_name)[0])
            success = self.action_sequence_builder.save_to_JSON()
            self.text_output.append(f"Saved file: {success}")
            # set text in gui from action sequence name
            self.set_widget_action_sequence_name(self.action_sequence_builder.name)
            # update the last saved timestamp
            self.update_last_saved()
            self.set_recent_file()
            
        self.clear_action_parameter_layout()
        self.save_as = False                

    def append_service_dialog(self, service_name: str = None, service_client:str= None, serivce_type:str = None)->None:
        add_service_dialog = AddServiceDialog(service_name, service_client, serivce_type)
        
        if add_service_dialog.exec():
            service_name, service_client, service_type = add_service_dialog.get_values()
            self.add_service_to_action_list(service_name=service_name,
                                            service_client=service_client,
                                            service_type=service_type)
        else:
            pass

    def load_recent_file(self):
        """
        Loads the last opened file from the yaml file.
        """
        path = get_package_share_directory('ros_sequential_action_programmer')

        # Specify the path to your YAML file
        yaml_file_path = f"{path}/recent_file.yaml"
        try:
            # Read the content of the YAML file
            with open(yaml_file_path, 'r') as file:
                yaml_content = yaml.safe_load(file)

            process_file_path = yaml_content['recent_file'] 
            self.action_sequence_builder.load_from_JSON(process_file_path)
        except:
            self.service_node.get_logger().warn("No recent file found! Skipping loading of recent file!")

    def set_recent_file(self):
        """
        Saves the last opened file to the yaml file.
        """
        recent_file_dict = {}
        recent_file_dict['recent_file'] = self.action_sequence_builder.action_file_path
        path = get_package_share_directory('ros_sequential_action_programmer')
        # Specify the path to your YAML file
        yaml_file_path = f"{path}/recent_file.yaml"
        with open(yaml_file_path, 'w') as file:
            yaml.dump(recent_file_dict, file, default_flow_style=False)

    def stop_execution(self) -> None:
        self.stop_execution = True

    def add_service_to_action_list(self, service_name: str, service_client:str, service_type:str = None) -> None:
        #selected_client = self.service_combo_box.currentText()
        pos_to_insert = self.checkbox_list.currentRow() + 1

        if service_client:
            success = self.action_sequence_builder.append_service_to_action_list_at_index(service_client=service_client, 
                                                                                    index = pos_to_insert, 
                                                                                    service_type=service_type,
                                                                                    service_name = service_name)
            # Get the name of the service from the currently acive action, which is the newly added one
            if success:
                service_name =  self.action_sequence_builder.get_current_action_name()
                function_checkbox = ReorderableCheckBoxListItem(f"{pos_to_insert}. {service_name}")
                self.checkbox_list.insertItem(pos_to_insert,function_checkbox)
                self.init_actions_list()
                self.checkbox_list.setCurrentRow(pos_to_insert)
                self.text_output.append(f"Inserted action: {service_name}")
                self.action_selected()
            else:
                self.text_output.append_red_text(f"Invalid input arguments")
                    
    def set_gui_interactionable(self,set_to:bool)->None:
        """
        With this method gui interactions can be set to true/false. 
        This is used when e.a. a action is running to prevent that the user can start parallel processes.
        """
        self.execute_step_button.setEnabled(set_to)
        self.run_action_sequence_button.setEnabled(set_to)

    def delete_action_from_sequence(self):
        selected_function = self.checkbox_list.currentItem()
        current_row = self.checkbox_list.currentRow()
        if selected_function:
            # Remove the function from the action list and the list
            del_success = self.action_sequence_builder.delete_action_at_index(current_row)

            if del_success:
                self.text_output.append(f"Deleted action '{selected_function.text()}'")
                self.init_actions_list()
                self.checkbox_list.setCurrentRow(0)
            else:
                self.text_output.append(f"Error trying to delete action '{selected_function.text()}'")
        else:
            self.text_output.append("No action selected to delete!")

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
                self.populate_log_widget(value, item)
        elif isinstance(data, list):
            for index, item_data in enumerate(data):
                item = QTreeWidgetItem(parent, [f"[{index}]"])
                self.populate_log_widget(item_data, item)
        else:

            item = QTreeWidgetItem(parent, [str(data)])   

    def clear_log_viewer(self):
        self.log_widget.clear()

    def copy_and_insert_actions(self):
        #index = self.checkbox_list.currentRow()
        selected_indexes = self.checkbox_list.selectedIndexes()
        selected_rows_indexes = [index.row() for index in selected_indexes]
        self.action_sequence_builder.copy_actions_from_index_list_and_insert(selected_rows_indexes)
        #self.action_sequence_builder.copy_action_at_index_and_insert(index)
        self.init_actions_list()
        self.checkbox_list.setCurrentRow(max(selected_rows_indexes)+1)
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

class AppTextOutput(QTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)

    def append_red_text(self, text:str) -> None:
        self.setTextColor(QColor("red"))
        self.append(text)
        self.setTextColor(QColor("black"))

    def append_green_text(self, text:str) -> None:
        self.setTextColor(QColor("green"))
        self.append(text)
        self.setTextColor(QColor("black"))

class QLineButton(QWidget):
    def __init__(self, full_key, initial_value, on_text_changed, on_button_clicked):
        super().__init__()

        label = QLabel(full_key)
        edit = QLineEdit(initial_value)
        edit.textChanged.connect(on_text_changed(full_key, edit))

        button = QPushButton("+")
        button.clicked.connect(partial(on_button_clicked,full_key,edit))

        label_layout = QHBoxLayout()
        edit_button_layout = QHBoxLayout()

        edit_button_layout.addWidget(edit)
        edit_button_layout.addWidget(button)
        
        label_layout.addWidget(label)

        main_layout = QVBoxLayout()
        main_layout.addLayout(label_layout)
        main_layout.addLayout(edit_button_layout)
        self.setLayout(main_layout)

class ReorderableCheckBoxListWidget(QListWidget):
    CustDragSig = QtCore.pyqtSignal()
    def __init__(self):
        super(ReorderableCheckBoxListWidget,self).__init__()
        self.setAcceptDrops(True)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QListWidget.DragDropMode.InternalMove)
        self.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)

    def get_widget_list_names(self):
        name_list=[]
        for index in range(self.count()):
            item = self.item(index)
            if item:
                name_list.append(item.text())
        return name_list
    
    def dropEvent(self, event):
        super(ReorderableCheckBoxListWidget, self).dropEvent(event)
        event.accept()
        self.CustDragSig.emit()

    def dragEnterEvent(self, event):
        self.drag_source_position = self.currentRow()  # Capture the source position
        super(ReorderableCheckBoxListWidget, self).dragEnterEvent(event)

class ReorderableCheckBoxListItem(QListWidgetItem):
    def __init__(self, function_name):
        super().__init__(function_name)
        self.setFlags(self.flags() | Qt.ItemFlag.ItemIsUserCheckable)
        font = self.font()
        font.setPointSize(14)
        self.setFont(font)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RsapApp()
    window.show()
    sys.exit(app.exec())