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

class ActionParameterMainLayout(QVBoxLayout):
    def __init__(self):
        super().__init__()

    def clear_action_parameter_layout(self):
        """
        This method clears the action parameter layout.
        """
        while self.count():
            item = self.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
            else:
                self.clear_action_parameter_layout(item.layout())


class ActionParameterWidget(QWidget):
    actionChanged = pyqtSignal(object)

    def __init__(self, 
                 action: Union[ActionBaseClass, ServiceAction, UserInteractionAction, RosActionAction], 
                 action_parameter_value_manager: ActionParameterValueManager,
                 active : bool = True,
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

        if not active:
            apply_btn.setDisabled(True)
            toggle_btn.setDisabled(True)

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
        

        req_types = self.action.get_request_type()
        test_keys = self.action.get_request_keys_for_type("string")

        #self.logger.error (f"{test_keys}")

        self.add_meta_data_info()
        self.handle_dict = copy.deepcopy(self.action.get_request_as_ordered_dict())
        #self.populateActionParameterWidgets(self.handle_dict)
    

        if active:
            param_editor_wid = ROS2DictEditor(types_dict=req_types,
                                            values_dict=self.handle_dict,
                                            action=self.action,
                                            action_parameter_value_manager = self.action_parameter_value_manager,
                                            logger=self.logger,
                                            is_array_element=False)
            
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
            label_action_service_type = QLineEdit(f"{_action.get_type_indicator()}")
            label_action_service_type.setReadOnly(True)
            self.parameter_layout.addWidget(label_action_service_desc)
            self.parameter_layout.addWidget(label_action_service_type)
            
            # Create a dropdown menu for selecting the error handling inputs
            label_error_handling_box = QLabel(f"Execution identifier: ")
            self.error_handling_box = NoScrollComboBox()

            box_values = ['None'] + _action.get_res_bool_fields()
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

            box_values = ['None'] + _action.get_res_bool_fields()
            self.error_handling_box.addItems(box_values)
            # Set the default value when creating the qcombobox
            currently_set_error_handler = None
            currently_set_error_handler = _action.get_success_identifier()
            if currently_set_error_handler is None:
               currently_set_error_handler = "None"

            self.error_handling_box.setCurrentIndex(box_values.index(currently_set_error_handler))
            self.parameter_layout.addWidget(label_error_handling_box)
            self.parameter_layout.addWidget(self.error_handling_box)
            
    def on_error_handling_changed(self,new_value):
        if new_value == "None":
            self.action.set_success_identifier(None)
        else:
            self.action.set_success_identifier(new_value)
        # Optional: log the change for debugging
        self.logger.info(f"Execution identifier updated to: {new_value}")

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
        
        if isinstance(self.action, ServiceAction) or isinstance(self.action, RosActionAction):
            self.on_error_handling_changed(self.error_handling_box.currentText())
            
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
                 action: Union[ActionBaseClass, ServiceAction, UserInteractionAction, RosActionAction] = None, 
                 action_parameter_value_manager: ActionParameterValueManager = None,
                 logger = None,
                is_array_element: bool = False
                 ):
        
        super().__init__()
        self.types_dict = types_dict
        self.values_dict = values_dict

        self._action = action
        self.disabled_keys = []

        # log the disabled keys
        self._action_parameter_value_manager = action_parameter_value_manager
        self.logger = logger
        self.is_array_element = is_array_element

        self.main_layout = QVBoxLayout()
        self.init_layout()
        self.setLayout(self.main_layout)
        self.setWindowTitle("ROS2 Message Editor")

    def init_layout(self):
        self.clear_layout(self.main_layout)
        self.build_layout(self.types_dict, self.values_dict, self.main_layout)

    def build_layout(self, type_dict, value_dict, parent_layout, parent_path=""):
        self.disabled_keys = []

        # This is for disabling buttons globally if the action or parameter value manager is not given (needed for the seq parameter manager)
        if self._action_parameter_value_manager is None or self._action is None:
            global_button_disable = True
        else:
            global_button_disable = False
            
        if self._action is not None:
            self.disabled_keys.extend(self._action.get_references().get_all_value_keys_with_reference())
            self.disabled_keys.extend(["start.header.stamp", "start.pose.position.x", "tolerance"])

        #self.logger.error(f"Disabled keys for action '{self._action.get_name()}': {self.disabled_keys}")


        if type_dict is None:
            return

        for key, val_type in type_dict.items():
            field_name = key.capitalize()
            full_path = f"{parent_path}.{key}" if parent_path else key
            is_disabled = self.check_disabled(full_path, self.disabled_keys)
            has_parent_disabled = self.has_disabled_parent(full_path, self.disabled_keys)

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
                    dlg = ArrayEditDialog(parent=self, 
                                          arr=value_dict[k], 
                                          val_type=t,
                                          action_parameter_value_manager=self._action_parameter_value_manager,
                                          current_action=self._action,
                                          logger=self.logger)

                    dlg.resize(500, 400)
                    dlg.show()
                    #dlg.exec()

                add_btn.clicked.connect(partial(open_editor, key, val_type))
                continue

                    # ---------- END ARRAY HANDLING ----------

            # ---------- NESTED MESSAGE ----------
            if 'fields' in val_type:
                header_layout = QHBoxLayout()
                label = QLabel(field_name)
                label.setToolTip(val_type['type'])
                header_layout.addWidget(label)

                # Determine button for nested message
                if is_disabled:
                    btn_text = "-"  # exact disabled
                    btn_enabled = True
                    btn_clicked = partial(self.show_disabled_field_dialog, full_path, val_type['type'])
                elif has_parent_disabled:
                    btn_text = "×"  # child of disabled
                    btn_enabled = False
                    btn_clicked = None
                else:
                    btn_text = "+"
                    btn_enabled = True
                    btn_clicked = partial(self.recom_clicked, full_path, val_type['type'])

                btn = ReferenceButton(btn_text, val_type=val_type['type'])
                if btn_clicked:
                    btn.clicked.connect(btn_clicked)
                btn.setEnabled(btn_enabled)
                if btn_text in ("-", "×"):
                    btn.setStyleSheet("color: gray;")

                if not global_button_disable:
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
                continue

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


                # --- Apply disabled visuals ---
                if is_disabled or has_parent_disabled:
                    widget.setStyleSheet("color: gray; background-color: #f2f2f2;")
                    widget.setDisabled(True)

                hlayout.addWidget(widget)

                # --- Button logic ---
                if is_disabled:
                    # Exact disabled field → "-"
                    btn = ReferenceButton("-", val_type=typ)
                    btn.clicked.connect(partial(self.show_disabled_field_dialog, full_path, typ))
                    btn.setStyleSheet("color: gray;")
                elif has_parent_disabled:
                    # Child of disabled field → "×" (gray, disabled)
                    btn = ReferenceButton("×", val_type=typ)
                    btn.setEnabled(False)
                    btn.setStyleSheet("color: gray;")
                else:
                    # Normal editable field → "+"
                    btn = ReferenceButton("+", val_type=typ)
                    btn.clicked.connect(partial(self.recom_clicked, full_path, typ))

                # only add button if not globally disabled
                if not global_button_disable:
                    hlayout.addWidget(btn)
                    
                parent_layout.addLayout(hlayout)
    
    def show_disabled_field_dialog(self, field_path, field_type):
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Disabled Field")
        dlg.setText(f"'{field_path}' ({field_type}) has a reference.\nDo you want to delete the existing reference?")
        dlg.setIcon(QMessageBox.Icon.Warning)
        dlg.setStandardButtons(QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
        dlg.setDefaultButton(QMessageBox.StandardButton.Cancel)

        result = dlg.exec()

        if result == QMessageBox.StandardButton.Ok:
            # Delete the reference from values_dict
            #self.delete_value_by_path(self.values_dict, field_path)
            self._action.get_references().del_reference(field_path)
            if self.logger:
                self.logger.info(f"Deleted disabled reference '{field_path}'")
            self.init_layout()  # Rebuild the layout to reflect changes

    def clear_layout(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            l = item.layout()
            if w:
                w.deleteLater()
            elif l:
                self.clear_layout(l)

    # def check_disabled(self, full_path, disabled_keys):
    #     return any(full_path == k or full_path.startswith(k + ".") for k in disabled_keys)

    def check_disabled(self, full_path, disabled_keys):
        """True if this field is exactly disabled."""
        return full_path in disabled_keys

    def has_disabled_parent(self, full_path, disabled_keys):
        """True if this field is a descendant of a disabled key, excluding exact matches."""
        return any(full_path.startswith(k + ".") for k in disabled_keys if full_path != k)

    
    def recom_clicked(self, key: str, field_type: str):

        #self.logger.error(f"{field_type}")
        #self.logger.error(f"{key}")

        self._action_parameter_value_manager.parameter_values_set_generator.update()

        #self.logger.error(f"Getting dialog")
        
        dlg = ActionParameterValueManagerDialog(current_action=self._action,
                                                current_type=field_type,
                                                current_values_dict=self.values_dict,
                                                current_field_key=key,
                                                action_parameter_value_manager=self._action_parameter_value_manager,
                                                add_equations=not self.is_array_element,
                                                add_references=not self.is_array_element,
                                                )
        dlg.exec()

        #self.logger.error(f"{dlg.current_field_key}")
        #self.logger.error(f"{dlg.values_dict}")
        #self.logger.error(f"{dlg.current_type}")

        self.init_layout()  # Rebuild the layout to reflect any changes



class ArrayEditDialog(QDialog):
    """Now includes a scroll area to handle many elements."""
    def __init__(self, 
                 parent, 
                 arr, 
                 val_type,
                 action_parameter_value_manager: ActionParameterValueManager = None,
                 current_action: ActionBaseClass = None,
                 logger = None):
        
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
        self._action_parameter_value_manager = action_parameter_value_manager
        self._action = current_action
        self.logger = logger
        
        if self._action_parameter_value_manager is None or self._action is None:
            self.global_button_disable = True
        else:
            self.global_button_disable = False

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
                    types_dict=self.val_type['fields'],
                    values_dict=self.arr[i],
                    action=self._action,
                    action_parameter_value_manager=self._action_parameter_value_manager,
                    logger=self.logger,
                    is_array_element=True
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

        # --- Value widget ---
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

        # --- Recommendation button ---
        btn = ReferenceButton('+', val_type=typ)
        # only add button if not globally disabled
        if not self.global_button_disable:
            layout.addWidget(btn)

        # Connect to recommendation logic
        def on_recom_clicked():
            self._action_parameter_value_manager.parameter_values_set_generator.update()
            dlg = ActionParameterValueManagerDialog(
                current_action=self._action,
                current_type=typ,
                current_values_dict={str(i): self.arr[i]},
                current_field_key=i,
                action_parameter_value_manager=self._action_parameter_value_manager,
                add_references=False,
                add_equations=False,
            )
            dlg.exec()
            # Apply recommended value back to array

            if dlg.values_dict.get(str(i)) is not None:
                self.arr[i] = dlg.values_dict[str(i)]
                w.setValue(self.arr[i]) if hasattr(w, "setValue") else w.setText(str(self.arr[i]))

            #self.logger.error(f"Updated array element {i} to {self.arr[i]}")

        btn.clicked.connect(on_recom_clicked)

        return layout

    def add_element(self):
        def create_default_value(val_type):
            if 'fields' in val_type:
                # Nested message → recursively create dict with defaults
                return {k: create_default_value(v) for k, v in val_type['fields'].items()}
            else:
                t = val_type['type']
                if t in INT_TYPE_BOUNDS or t in FLOAT_TYPE_BOUNDS:
                    return 0
                elif t in ('string', 'str'):
                    return ""
                elif t in ('bool', 'boolean'):
                    return False
                else:
                    return None

        default_value = create_default_value(self.val_type)
        self.arr.append(default_value)
        #self.logger.error(f"Added new element with defaults: {default_value}")
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
        