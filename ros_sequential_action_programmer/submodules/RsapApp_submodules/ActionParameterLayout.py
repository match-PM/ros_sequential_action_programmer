from PyQt6.QtWidgets import QScrollArea, QMessageBox, QDialog, QHBoxLayout, QDialog, QInputDialog, QTreeWidget, QTreeWidgetItem, QApplication, QGridLayout, QFrame, QMainWindow, QListWidget, QListWidgetItem, QDoubleSpinBox, QWidget, QVBoxLayout, QPushButton, QCheckBox, QLineEdit, QComboBox, QTextEdit,QLabel,QSlider, QSpinBox, QFontDialog, QFileDialog, QGroupBox

from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction
from ros_sequential_action_programmer.submodules.RsapApp_submodules.NoScrollComboBox import NoScrollComboBox

from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI, TERMINAL
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from PyQt6.QtCore import pyqtSignal
from ros_sequential_action_programmer.submodules.RsapApp_submodules.RecomButton import RecomButton

from typing import Union
from collections import OrderedDict
import copy
import ast

class ActionParameterLayout(QWidget):
    actionChanged = pyqtSignal(object)

    def __init__(self, 
                 action: Union[ActionBaseClass, ServiceAction, UserInteractionAction, RosActionAction], 
                 logger = None):
        super().__init__()

        self.logger = logger
        self.action = action

        # ----- Main layout -----
        main_layout = QVBoxLayout(self)       # set as widget layout
        self.setLayout(main_layout)

        # ----- Title label -----
        title = QLabel("Action parameters:")
        title_font = QFont()
        title_font.setPointSize(14)
        title.setFont(title_font)
        main_layout.addWidget(title)

        # ----- Buttons -----
        toggle_btn = QPushButton("De-/Activate Action")
        toggle_btn.clicked.connect(self.toggle_action_activation)

        apply_btn = QPushButton("Apply Changes")
        apply_btn.clicked.connect(self.apply_changes_to_action)

        main_layout.addWidget(toggle_btn)
        main_layout.addWidget(apply_btn)

        # ----- Scrollable area -----
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        self.parameter_widget = QWidget()
        self.parameter_layout = QVBoxLayout(self.parameter_widget)
        self.parameter_widget.setLayout(self.parameter_layout)

        self.scroll_area.setWidget(self.parameter_widget)
        main_layout.addWidget(self.scroll_area)
        
        self.logger.error ("Test")
        req_types = self.action.get_request_type()

        self.add_meta_data_info()
        self.handle_dict = copy.deepcopy(self.action.get_request_dict())
        #self.populateActionParameterWidgets(self.handle_dict)
        
        disabled_keys = ["start.header.stamp", "start.pose.position.x", "tolerance"]
        
        param_editor_wid = ROS2DictEditor(types_dict=req_types,
                                          values_dict=self.handle_dict,
                                          disabled_keys=disabled_keys)
        
        self.parameter_layout.addWidget(param_editor_wid)


        self.logger.error(f"{self.handle_dict}")
        

    def toggle_action_activation(self):
        self.action.toggle_active()
        # Emit the signal with the current action as payload
        self.actionChanged.emit(self.action)

    def on_action_changed(self, slot_func):
        """Convenience to connect a callback."""
        self.actionChanged.connect(slot_func)
        
    def add_meta_data_info(self):
        
        label_action_name = QLabel("Action Name:")
        self.action_name_edit = QLineEdit(self.action.get_name())
        self.parameter_layout.addWidget(label_action_name)
        self.parameter_layout.addWidget(self.action_name_edit)    

        label_action_type_desc = QLabel(f"Action Type: ")
        label_action_type = QLineEdit(f"{self.action.get_type_indicator()}")
        label_action_type.setReadOnly(True)
        self.parameter_layout.addWidget(label_action_type_desc)
        self.parameter_layout.addWidget(label_action_type)
        
        # Set info for service type
        label_action_description_desc = QLabel(f"Description: ")
        self.label_action_description = QTextEdit(f"{self.action.get_description()}")
        self.parameter_layout.addWidget(label_action_description_desc)
        self.parameter_layout.addWidget(self.label_action_description)
        
        if isinstance(self.action, ServiceAction):
            _action:ServiceAction = self.action 
            # Set info for service client
            label_action_client_desc = QLabel(f"Service Client: ")
            label_action_client = QLineEdit(f"{_action.client}")
            label_action_client.setReadOnly(True)
            self.parameter_layout.addWidget(label_action_client_desc)
            self.parameter_layout.addWidget(label_action_client)
            
            # Set info for service type
            label_action_service_desc = QLabel(f"Service Type: ")
            label_action_service_type = QLineEdit(f"{_action.service_type}")
            label_action_service_type.setReadOnly(True)
            self.parameter_layout.addWidget(label_action_service_desc)
            self.parameter_layout.addWidget(label_action_service_type)
            
            # Create a dropdown menu for selecting the error handling inputs
            label_error_handling_box = QLabel(f"Execution identifier: ")
            self.error_handling_box = NoScrollComboBox()

            box_values = ['None'] + _action.get_bool_fields()
            self.error_handling_box.addItems(box_values)
            # Set the default value when creating the qcombobox
            currently_set_error_handler = _action.get_success_identifier()
            if currently_set_error_handler is None:
               currently_set_error_handler = "None"

            self.error_handling_box.setCurrentIndex(box_values.index(currently_set_error_handler))
            self.parameter_layout.addWidget(label_error_handling_box)
            self.parameter_layout.addWidget(self.error_handling_box)
            
            
        if isinstance(self.action, RosActionAction):
            _action:RosActionAction = self.action 
            # Set info for service client
            label_action_client_desc = QLabel(f"Action Client: ")
            label_action_client = QLineEdit(f"{_action.client}")
            label_action_client.setReadOnly(True)
            self.parameter_layout.addWidget(label_action_client_desc)
            self.parameter_layout.addWidget(label_action_client)
            
            # Set info for service type
            label_action_service_desc = QLabel(f"Action Type: ")
            label_action_service_type = QLineEdit(f"{_action.action_type}")
            label_action_service_type.setReadOnly(True)
            self.parameter_layout.addWidget(label_action_service_desc)
            self.parameter_layout.addWidget(label_action_service_type)
            
            # Create a dropdown menu for selecting the error handling inputs
            label_error_handling_box = QLabel(f"Execution identifier: ")
            self.error_handling_box = NoScrollComboBox()

            box_values = ['None'] #+ _action.get_bool_fields()
            self.error_handling_box.addItems(box_values)
            # Set the default value when creating the qcombobox
            currently_set_error_handler = None
            #currently_set_error_handler = _action.get_success_identifier()
            if currently_set_error_handler is None:
               currently_set_error_handler = "None"

            self.error_handling_box.setCurrentIndex(box_values.index(currently_set_error_handler))
            self.parameter_layout.addWidget(label_error_handling_box)
            self.parameter_layout.addWidget(self.error_handling_box)
        

    def apply_changes_to_action(self):

        if isinstance(self.action, ServiceAction) or isinstance(self.action, RosActionAction):
            # Apply error handler message
            error_identifier = self.error_handling_box.currentText()
            if error_identifier == 'None':
                error_identifier = None

            # set the error identifier
            set_error_identifier_success = self.action.set_success_identifier(error_identifier)
        
        # Set the action description text
        self.action.set_description(self.label_action_description.toPlainText())

        # Apply values to service request dict
        self.action.set_name(self.action_name_edit.text())
        
        _old_dict = copy.deepcopy(self.action.get_request_dict())
        
        # set the request_dict
        self.action.set_request_dict(self.handle_dict)

        # set the request obj from the request
        set_success = self.action.set_request_from_request_dict()
        
        if not set_success:
            #set back to old dict
            self.action.set_request_dict(_old_dict)
            self.logger.error(f"Setting failed")         
        
        
        # if isinstance(self.action, ServiceAction):
        #     set_success = self.action_sequence_builder.process_action_dict_at_index(index=index,mode=SET_IMPLICIT_SRV_DICT, input_impl_dict=self.handle_dict)

        #     if set_success and set_error_identifier_success:
        #         self.text_output.append("Success changing values!")
        #     else:
        #         self.text_output.append_red_text("Error occured changing changes!")
        
        self.actionChanged.emit(self.action)

    # def populateActionParameterWidgets(self, data, parent_key=None, active=True):
    #     """
    #     This method populates the action parameter layout with the parameters of the selected action.
    #     """
    #     # iterate through the dict
    #     for key, value in data.items():
    #         if parent_key is not None:
    #             full_key = parent_key + '.' + key
    #         else:
    #             full_key = key

    #         if isinstance(value, OrderedDict):
    #             self.populateActionParameterWidgets(data=value, 
    #                                                 parent_key=full_key, 
    #                                                 active = active)
    #         else:
    #             widget_with_button = RecomButton(full_key=full_key, 
    #                                              initial_value=str(value), 
    #                                              on_text_changed=self.updateDictionary, 
    #                                              on_button_clicked=self.get_recom_button_clicked,
    #                                              active=active)
                
    #             self.parameter_layout.addWidget(widget_with_button)

    # def updateDictionary(self, key, edit):
    #     def handleTextChange(text):
    #         try:
    #             keys = key.split('.')
    #             current_dict = self.handle_dict
    #             for k in keys[:-1]:
    #                 current_dict = current_dict[k]

    #             # Convert the input text to the appropriate data type
    #             value = text
    #             if '.' in text and all(part.replace('.', '').lstrip('-').isdigit() for part in text.split('.', 1)):
    #                 # Modified condition to allow negative floats
    #                 value = float(text)
    #             elif text.lstrip('-').isdigit():
    #                 value = int(text)
    #             elif text == 'True':
    #                 value = True
    #             elif text == 'False':
    #                 value = False
    #             elif text[0] == '[' and text[-1] == ']':
    #                 value = ast.literal_eval(text)
    #             elif text == 'None':
    #                 value = None

    #             current_dict[keys[-1]] = value
    #         except ValueError:
    #             pass

    #     return handleTextChange
    
    # def get_recom_button_clicked(self, key, widget:QLineEdit):
    #     """
    #     This method is called when the user clicks the button to get recommendations for a specific parameter.
    #     """
    #     index = self.action_list_widget.currentRow()
    #     data = self.action_sequence_builder.get_recom_for_action_at_index_for_key(index=index, key=key)
    #     popup = PopupRecWindow(data)
    #     result = popup.exec()

    #     if result == QDialog.DialogCode.Accepted:
    #         selected_value = popup.get_selected_value()
    #         widget.setText(selected_value)
    #         print(f"Selected value: {selected_value}")
            
            
class ROS2DictEditor(QWidget):
    def __init__(self, types_dict, values_dict, disabled_keys=None):
        super().__init__()
        self.types_dict = types_dict
        self.values_dict = values_dict
        self.disabled_keys = set(disabled_keys or [])

        layout = QVBoxLayout()
        self.build_layout(types_dict, values_dict, layout)
        self.setLayout(layout)
        self.setWindowTitle("ROS2 Message Editor")
        
    def build_layout(self, type_dict, value_dict, parent_layout, parent_path=""):
        if type_dict is None:
            return
        for key, val_type in type_dict.items():
            field_name = key.capitalize()
            full_path = f"{parent_path}.{key}" if parent_path else key

            is_disabled = self.check_disabled(full_path, self.disabled_keys)

            if 'fields' in val_type:
                # Nested message -> header with label + button
                header_layout = QHBoxLayout()
                label = QLabel(field_name)
                label.setToolTip(val_type['type'])
                header_layout.addWidget(label)

                if full_path in self.disabled_keys:
                    btn_text = "-"  # disabled top-level field
                elif any(full_path.startswith(k + ".") for k in self.disabled_keys):
                    btn_text = "×"  # disabled child
                else:
                    btn_text = "+"
                    
                btn = ReferenceButton(btn_text,
                                  val_type=val_type['type'])
                if btn_text == "×":
                    btn.setDisabled(True)
                else:
                    btn.setDisabled(False)
                    
                btn.clicked.connect(lambda k=full_path: print(f"Group button clicked: {k}"))
                header_layout.addWidget(btn)

                group = QGroupBox()
                group.setLayout(QVBoxLayout())
                parent_layout.addWidget(group)
                group.layout().addLayout(header_layout)

                self.build_layout(val_type['fields'], value_dict[key], group.layout(), full_path)

            else:
                # Primitive field
                hlayout = QHBoxLayout()
                label = QLabel(field_name)
                label.setToolTip(val_type['type'])
                hlayout.addWidget(label)

                typ = val_type['type']
                widget = None

                if typ == 'int':
                    widget = QSpinBox()
                    widget.setValue(value_dict[key])
                    widget.setDisabled(is_disabled)
                    widget.setMinimum(-1_000_000_000)   # allow negative numbers
                    widget.setMaximum(1_000_000_000)    # optional upper limit
                    widget.valueChanged.connect(lambda val, d=value_dict, k=key: d.__setitem__(k, val))
                elif typ == 'float':
                    widget = QDoubleSpinBox()
                    widget.setDecimals(6)
                    widget.setValue(value_dict[key])
                    widget.setDisabled(is_disabled)
                    widget.setMinimum(-1_000_000_000)   # allow negative numbers
                    widget.setMaximum(1_000_000_000)    # optional upper limit
                    widget.valueChanged.connect(lambda val, d=value_dict, k=key: d.__setitem__(k, val))
                elif typ == 'str':
                    widget = QLineEdit()
                    widget.setText(value_dict[key])
                    widget.setDisabled(is_disabled)
                    widget.textChanged.connect(lambda val, d=value_dict, k=key: d.__setitem__(k, val))
                elif typ == 'bool':
                    widget = QCheckBox()
                    widget.setChecked(value_dict[key])
                    widget.setDisabled(is_disabled)
                    widget.stateChanged.connect(lambda val, d=value_dict, k=key: d.__setitem__(k, bool(val)))
                else:
                    widget = QLabel(f"Unsupported type: {typ}")

                hlayout.addWidget(widget)

                btn_text = "×" if is_disabled else "…"
                btn = ReferenceButton(btn_text,val_type=typ)
                btn.setDisabled(is_disabled)
                btn.clicked.connect(lambda k=full_path: print(f"Field button clicked: {k}"))
                hlayout.addWidget(btn)

                parent_layout.addLayout(hlayout)

    def check_disabled(self, full_path, disabled_keys):
        for key in disabled_keys:
            if full_path == key or full_path.startswith(key + "."):
                return True
        return False

                
class ReferenceButton(QPushButton):
    def __init__(self, 
                 text='+', 
                 val_type="",
                 parent=None, 
                 width=20, 
                 height=20):
        
        super().__init__(text, parent)
        self.setFixedSize(width, height)  # small square button
        self.setToolTip("Click me!")      # optional default tooltip
        self.val_type = val_type
