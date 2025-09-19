from PyQt6.QtWidgets import QScrollArea, QMessageBox, QDialog, QHBoxLayout, QDialog, QInputDialog, QTreeWidget, QTreeWidgetItem, QApplication, QGridLayout, QFrame, QMainWindow, QListWidget, QListWidgetItem, QDoubleSpinBox, QWidget, QVBoxLayout, QPushButton, QCheckBox, QLineEdit, QComboBox, QTextEdit,QLabel,QSlider, QSpinBox, QFontDialog, QFileDialog
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

        self.add_meta_data_info()
        self.handle_dict = copy.deepcopy(self.action.request_dict_implicit)
        self.populateActionParameterWidgets(self.handle_dict)

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
            
        if isinstance(self.action, UserInteractionAction):
            _action:UserInteractionAction = self.action 
            
            # Set info for service type
            interaction_text_label = QLabel(f"Interaction Text: ")
            self.interaction_text_box = QLineEdit(f"{_action.get_interaction_text()}")
            self.parameter_layout.addWidget(interaction_text_label)
            self.parameter_layout.addWidget(self.interaction_text_box)

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
        
        # if isinstance(self.action, ServiceAction):
        #     set_success = self.action_sequence_builder.process_action_dict_at_index(index=index,mode=SET_IMPLICIT_SRV_DICT, input_impl_dict=self.handle_dict)

        #     if set_success and set_error_identifier_success:
        #         self.text_output.append("Success changing values!")
        #     else:
        #         self.text_output.append_red_text("Error occured changing changes!")

        self.actionChanged.emit(self.action)

    def populateActionParameterWidgets(self, data, parent_key=None, active=True):
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
                self.populateActionParameterWidgets(data=value, 
                                                    parent_key=full_key, 
                                                    active = active)
            else:
                widget_with_button = RecomButton(full_key=full_key, 
                                                 initial_value=str(value), 
                                                 on_text_changed=self.updateDictionary, 
                                                 on_button_clicked=self.get_recom_button_clicked,
                                                 active=active)
                
                self.parameter_layout.addWidget(widget_with_button)

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
    
    def get_recom_button_clicked(self, key, widget:QLineEdit):
        """
        This method is called when the user clicks the button to get recommendations for a specific parameter.
        """
        index = self.action_list_widget.currentRow()
        data = self.action_sequence_builder.get_recom_for_action_at_index_for_key(index=index, key=key)
        popup = PopupRecWindow(data)
        result = popup.exec()

        if result == QDialog.DialogCode.Accepted:
            selected_value = popup.get_selected_value()
            widget.setText(selected_value)
            print(f"Selected value: {selected_value}")