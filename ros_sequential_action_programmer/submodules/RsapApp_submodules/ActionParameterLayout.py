from PyQt6.QtWidgets import QScrollArea, QMessageBox, QDialog, QHBoxLayout, QDialog, QInputDialog, QTreeWidget, QTreeWidgetItem, QApplication, QGridLayout, QFrame, QMainWindow, QListWidget, QListWidgetItem, QDoubleSpinBox, QWidget, QVBoxLayout, QPushButton, QCheckBox, QLineEdit, QComboBox, QTextEdit,QLabel,QSlider, QSpinBox, QFontDialog, QFileDialog, QGroupBox, QSizePolicy

    
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction
from ros_sequential_action_programmer.submodules.RsapApp_submodules.NoScrollComboBox import NoScrollComboBox

from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI, TERMINAL
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from PyQt6.QtCore import pyqtSignal
from ros_sequential_action_programmer.submodules.RsapApp_submodules.RecomButton import RecomButton
from ros_sequential_action_programmer.submodules.rsap_modules.errors import SetActionRequestError

from typing import Union
from collections import OrderedDict
import copy
import ast
from functools import partial

from ros_sequential_action_programmer.submodules.action_classes.ros_messages_functions import resolve_ros_type
from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import ActionParameterValueManager
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionParameterValueManagerDialog import ActionParameterValueManagerDialog

class NoWheelSpinBox(QSpinBox):
    def wheelEvent(self, event):
        event.ignore()  # ignore the wheel event

class NoWheelDoubleSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()
        
class ActionParameterLayout(QWidget):
    actionChanged = pyqtSignal(object)

    def __init__(self, 
                 action: Union[ActionBaseClass, ServiceAction, UserInteractionAction, RosActionAction], 
                 action_parameter_value_manager: ActionParameterValueManager,
                 logger = None):
        super().__init__()

        self.logger = logger
        self.action = action
        self.action_parameter_value_manager = action_parameter_value_manager
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
        test_keys = self.action.get_request_keys_for_type("string")
        self.logger.error (f"{test_keys}")
        self.add_meta_data_info()
        self.handle_dict = copy.deepcopy(self.action.get_request_as_ordered_dict())
        #self.populateActionParameterWidgets(self.handle_dict)
        
        disabled_keys = ["start.header.stamp", "start.pose.position.x", "tolerance"]
        
        param_editor_wid = ROS2DictEditor(types_dict=req_types,
                                          values_dict=self.handle_dict,
                                          action=self.action,
                                          action_parameter_value_manager = self.action_parameter_value_manager,
                                          logger=self.logger,
                                          disabled_keys=disabled_keys)
        
        self.parameter_layout.addWidget(param_editor_wid)

        #self.logger.error(f"{self.handle_dict}")
        

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
        
        _old_dict = copy.deepcopy(self.action.get_request_as_ordered_dict())
        
        # set the request_dict
        try:
            self.action.set_request_from_dict(self.handle_dict)
        except SetActionRequestError as e:
            self.action.set_request_dict(_old_dict)
            self.logger.error(f"Setting failed")         
        
        self.actionChanged.emit(self.action)
    
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
            

MAX = 2_147_483_640
INT_TYPE_BOUNDS = {
    'int8': (-128, 127), 'uint8': (0, 255),
    'int16': (-32_768, 32_767), 'uint16': (0, 65_535),
    'int32': (-MAX, MAX), 'uint32': (0, MAX),
    'int64': (-MAX, MAX), 'uint64': (0, MAX)
}
FLOAT_TYPE_BOUNDS = {
    'float32': (-MAX, MAX), 'float64': (-MAX, MAX),
    'float': (-MAX, MAX), 'double': (-MAX, MAX),
}

class ROS2DictEditor(QWidget):
    def __init__(self, 
                 types_dict, 
                 values_dict, 
                 action: Union[ActionBaseClass, ServiceAction, UserInteractionAction, RosActionAction], 
                 action_parameter_value_manager: ActionParameterValueManager,
                 logger,
                 disabled_keys=None):
        
        super().__init__()
        self.types_dict = types_dict
        self.values_dict = values_dict
        self.disabled_keys = set(disabled_keys or [])
        self._action = action
        self._action_parameter_value_manager = action_parameter_value_manager
        self.logger = logger

        main_layout = QVBoxLayout()
        self.build_layout(types_dict, values_dict, main_layout)
        self.setLayout(main_layout)
        self.setWindowTitle("ROS2 Message Editor")

    def build_layout(self, type_dict, value_dict, parent_layout, parent_path=""):
        if type_dict is None:
            return

        for key, val_type in type_dict.items():
            field_name = key.capitalize()
            full_path = f"{parent_path}.{key}" if parent_path else key
            is_disabled = self.check_disabled(full_path, self.disabled_keys)

            # ---------- ARRAY HANDLING ----------
            if val_type.get('is_array', False):
                if key not in value_dict or not isinstance(value_dict[key], list):
                    value_dict[key] = []

                array_group = QGroupBox(field_name)
                array_layout = QVBoxLayout()
                array_group.setLayout(array_layout)
                parent_layout.addWidget(array_group)

                add_btn = QPushButton(f"Edit {val_type['type']}[]")
                add_btn.setDisabled(is_disabled)
                array_layout.addWidget(add_btn)

                def open_editor(k, t, _checked=False):
                    dlg = ArrayEditDialog(self, value_dict[k], t)
                    dlg.resize(500, 400)
                    dlg.show()
                    #dlg.exec()

                add_btn.clicked.connect(partial(open_editor, key, val_type))
                continue
            # ---------- END ARRAY HANDLING ----------

            # nested message (non-array)
            if 'fields' in val_type:
                header_layout = QHBoxLayout()
                label = QLabel(field_name)
                label.setToolTip(val_type['type'])
                header_layout.addWidget(label)

                # Add placeholder button
                btn_text = "×" if is_disabled else "+"
                btn = ReferenceButton(btn_text, val_type=val_type['type'])
                btn.clicked.connect(partial(self.recom_clicked, val_type['type']))

                btn.setDisabled(is_disabled)
                header_layout.addWidget(btn)

                group = QGroupBox()
                group.setLayout(QVBoxLayout())
                parent_layout.addWidget(group)
                group.layout().addLayout(header_layout)

                self.build_layout(
                    val_type['fields'],
                    value_dict.setdefault(key, {}),
                    group.layout(),
                    full_path
                )
            else:
                # Primitive field
                hlayout = QHBoxLayout()
                label = QLabel(field_name)
                label.setToolTip(val_type['type'])
                hlayout.addWidget(label)

                typ = val_type['type']
                if typ in INT_TYPE_BOUNDS:
                    widget = NoWheelSpinBox()
                    widget.setRange(*INT_TYPE_BOUNDS[typ])
                    widget.setValue(value_dict.get(key, 0))
                    widget.valueChanged.connect(lambda v, d=value_dict, k=key: d.__setitem__(k, v))
                elif typ in FLOAT_TYPE_BOUNDS:
                    widget = NoWheelDoubleSpinBox()
                    widget.setDecimals(6)
                    widget.setRange(*FLOAT_TYPE_BOUNDS[typ])
                    widget.setValue(value_dict.get(key, 0.0))
                    widget.valueChanged.connect(lambda v, d=value_dict, k=key: d.__setitem__(k, v))
                elif typ in ('string', 'str'):
                    widget = QLineEdit(value_dict.get(key, ""))
                    widget.textChanged.connect(lambda v, d=value_dict, k=key: d.__setitem__(k, v))
                elif typ in ('bool', 'boolean'):
                    widget = QCheckBox()
                    widget.setChecked(value_dict.get(key, False))
                    widget.stateChanged.connect(lambda v, d=value_dict, k=key: d.__setitem__(k, bool(v)))
                else:
                    widget = QLabel(f"Unsupported type: {typ}")

                widget.setDisabled(is_disabled)
                hlayout.addWidget(widget)

                # Add placeholder button
                btn_text = "×" if is_disabled else "+"
                btn = ReferenceButton(btn_text, val_type=typ)
                btn.clicked.connect(partial(self.recom_clicked, typ))
                btn.setDisabled(is_disabled)
                hlayout.addWidget(btn)

                parent_layout.addLayout(hlayout)

    def check_disabled(self, full_path, disabled_keys):
        return any(full_path == k or full_path.startswith(k + ".") for k in disabled_keys)
    
    def recom_clicked(self, type):
        self.logger.error(f"{type}")
        self._action_parameter_value_manager.parameter_values_set_generator.update()
        dlg = ActionParameterValueManagerDialog(current_action=self._action,
                                                current_type=type,
                                                action_parameter_value_manager=self._action_parameter_value_manager)
        dlg.exec()
        


class ArrayEditDialog(QDialog):
    """Now includes a scroll area to handle many elements."""
    def __init__(self, 
                 parent, 
                 arr, 
                 val_type):
        super().__init__(parent)
        self.setWindowTitle("Edit Array")
        self.arr = arr
        self.val_type = val_type

        # Outer layout for dialog
        self.main_layout = QVBoxLayout(self)

        # ---- Scroll area ----
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.rows_layout = QVBoxLayout(self.scroll_content)
        self.scroll.setWidget(self.scroll_content)
        self.main_layout.addWidget(self.scroll)

        self.refresh_rows()

        add_btn = QPushButton("Add element")
        add_btn.clicked.connect(self.add_element)
        self.main_layout.addWidget(add_btn)

        close_btn = QPushButton("Done")
        close_btn.clicked.connect(self.accept)
        self.main_layout.addWidget(close_btn)

    def refresh_rows(self):
        def clear_layout(layout):
            while layout.count():
                item = layout.takeAt(0)
                w = item.widget()
                l = item.layout()
                if w:
                    w.deleteLater()
                elif l:
                    clear_layout(l)

        self.scroll_content.setUpdatesEnabled(False)
        clear_layout(self.rows_layout)

        for i, val in enumerate(self.arr):
            row_box = QGroupBox(f"Element {i}")
            row_v = QVBoxLayout(row_box)

            rm_btn = QPushButton("Remove")
            rm_btn.clicked.connect(lambda _, idx=i: self.remove(idx))
            row_v.addWidget(rm_btn)

            if 'fields' in self.val_type:
                if not isinstance(self.arr[i], dict):
                    self.arr[i] = {}
                nested_editor = ROS2DictEditor(
                    self.val_type['fields'],
                    self.arr[i]
                )
                # allow the nested editor to shrink/grow nicely
                nested_editor.setSizePolicy(QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed))
                row_v.addWidget(nested_editor)
            else:
                row_v.addLayout(self._primitive_editor(i, val))

            self.rows_layout.addWidget(row_box)

        self.scroll_content.setUpdatesEnabled(True)

    def _primitive_editor(self, i, val):
        layout = QHBoxLayout()
        typ = self.val_type['type']
        if typ in INT_TYPE_BOUNDS:
            w = NoWheelSpinBox()
            w.setRange(*INT_TYPE_BOUNDS[typ])
            w.setValue(val)
            w.valueChanged.connect(lambda v, idx=i: self.arr.__setitem__(idx, v))
        elif typ in FLOAT_TYPE_BOUNDS:
            w = NoWheelDoubleSpinBox()
            w.setDecimals(6)
            w.setRange(*FLOAT_TYPE_BOUNDS[typ])
            w.setValue(val)
            w.valueChanged.connect(lambda v, idx=i: self.arr.__setitem__(idx, v))
        elif typ in ('string', 'str'):
            w = QLineEdit(val)
            w.textChanged.connect(lambda v, idx=i: self.arr.__setitem__(idx, v))
        elif typ in ('bool', 'boolean'):
            w = QCheckBox()
            w.setChecked(val)
            w.stateChanged.connect(lambda v, idx=i: self.arr.__setitem__(idx, bool(v)))
        else:
            w = QLabel(f"Unsupported: {typ}")
        layout.addWidget(w)
        return layout

    def add_element(self):
        if 'fields' in self.val_type:
            self.arr.append({})
        else:
            t = self.val_type['type']
            if t in INT_TYPE_BOUNDS or t in FLOAT_TYPE_BOUNDS:
                self.arr.append(0)
            elif t in ('string', 'str'):
                self.arr.append("")
            elif t in ('bool', 'boolean'):
                self.arr.append(False)
            else:
                self.arr.append(None)
        self.refresh_rows()

    def remove(self, idx):
        self.arr.pop(idx)
        self.refresh_rows()
        
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
        