from rosidl_runtime_py.utilities import get_message
from ament_index_python.packages import get_packages_with_prefixes
from rosidl_runtime_py import get_message_interfaces
from PyQt6.QtWidgets import QWidget, QPushButton, QVBoxLayout, QDialog, QListWidget, QListWidgetItem, QVBoxLayout,  QLabel, QHBoxLayout,QLineEdit, QTableWidget, QTableWidgetItem
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameterManager, SeqParameter
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionParameterWidget import ROS2DictEditor
from ros_sequential_action_programmer.submodules.action_classes.ros_messages_functions import field_type_map_recursive_with_msg_type
from PyQt6.QtWidgets import QHeaderView
import json

class SeqParameterManagerDialog(QDialog):
    def __init__(self,seq_parameter_manager:SeqParameterManager,
                 logger=None,
                  parent=None):
        
        super().__init__(parent)

        self.seq_parameter_manager = seq_parameter_manager
        self.setWindowTitle("Sequence Parameter Manager")
        self.resize(800, 600)
        
        widget = SeqParameterWidget(self.seq_parameter_manager, logger=logger, parent=self)

        layout = QVBoxLayout(self)
        layout.addWidget(widget)


class SeqParameterWidget(QWidget):
    def __init__(self, 
                seq_parameter_manager:SeqParameterManager,
                logger=None,
                parent=None):
        super().__init__(parent)

        self.seq_parameter_manager = seq_parameter_manager
        self.logger = logger    
        messages_interfaces  = self._get_message_interfaces()
        self.messages_interfaces_sorted  = sorted(messages_interfaces)
        self._custom_sort()
        
        self.main_layout = QVBoxLayout(self)
        add_parameter_button = QPushButton("Add Parameter")
        add_parameter_button.clicked.connect(self._open_add_parameter_dialog)
        self.main_layout.addWidget(add_parameter_button)
        
        self.init_parameter_layout()

    
    def _custom_sort(self):
        sorted_list = self.messages_interfaces_sorted
        sorted_list.sort(key=lambda s: not s.startswith("std_msgs"))

    def init_parameter_layout(self):
        self.seq_parameter_list_widget = SeqParameterListWidget(self.seq_parameter_manager, logger=self.logger)

        self.main_layout.addWidget(self.seq_parameter_list_widget)

        return

    @staticmethod
    def _get_message_interfaces()->list[str]:
        msgs_interfaces = get_message_interfaces()
        interfaces = []
        for pkg, types in msgs_interfaces.items():
            for t in types:
                interfaces.append(f"{pkg}/{t}")
        return interfaces

    def _open_add_parameter_dialog(self):
        dialog = StringListDialog(self.messages_interfaces_sorted)
        #exec_ok = dialog.exec()
        result = dialog.get_result()
        
        if result is None:
            self.logger.error("No parameter selected.")
            return

        param_name, value_type = result

        if param_name == "":
            self.logger.error(f"Parameter name is empty!")
            return

        if self.seq_parameter_manager.check_parameter_name_exists(param_name):
            self.logger.error(f"Parameter with name '{param_name}' already exists!")
            return
        
        msg_class = get_message(value_type)
        msg_dict_or = message_to_ordereddict(msg_class())
        msg_dict_j = json.loads(json.dumps(msg_dict_or))
        new_parameter = SeqParameter(name=param_name, val_type=value_type, value=msg_dict_j)

        self.logger.info(f"Adding parameter with name: {param_name} and type: {value_type}")

        self.seq_parameter_manager.add_parameter(new_parameter)
        
        self.seq_parameter_list_widget._populate_parameter_list()
        
# class SeqParameterRow(QWidget):
#     def __init__(self, parameter:SeqParameter, on_delete_callback, parent=None):
#         super().__init__(parent)
#         self.parameter = parameter
#         self.on_delete_callback = on_delete_callback

#         name_label = QLabel(parameter.get_name())
#         type_label = QLabel(parameter.get_type())

#         delete_button = QPushButton("Delete")
#         delete_button.clicked.connect(self._on_delete_clicked)

#         edit_button = QPushButton("Edit")
#         edit_button.clicked.connect(self._on_edit_clicked)

#         layout = QHBoxLayout(self)
#         layout.addWidget(name_label)
#         layout.addWidget(type_label)
#         layout.addStretch()
#         layout.addWidget(edit_button)
#         layout.addWidget(delete_button)

#     def _on_delete_clicked(self):
#         self.on_delete_callback(self.parameter)

#     def _on_edit_clicked(self):
#         # Handle edit action
#         dialog = EditParameterDialog(self.parameter, parent=self)
#         dialog.exec()


# class SeqParameterListWidget(QWidget):
#     def __init__(self, seq_parameter_manager: SeqParameterManager, logger=None, parent=None):
#         super().__init__(parent)

#         self.seq_parameter_manager = seq_parameter_manager
#         self.logger = logger

#         self.list_widget = QListWidget()

#         layout = QVBoxLayout(self)
#         layout.addWidget(self.list_widget)

#         self._populate_parameter_list()

#     def _populate_parameter_list(self):
#         self.list_widget.clear()

#         for param in self.seq_parameter_manager.get_parameter_list():
#             self._add_parameter_row(param)
           
#     def _add_parameter_row(self, parameter):
#         item = QListWidgetItem(self.list_widget)

#         row_widget = SeqParameterRow(
#             parameter=parameter,
#             on_delete_callback=self._delete_parameter
#         )

#         item.setSizeHint(row_widget.sizeHint())
#         self.list_widget.addItem(item)
#         self.list_widget.setItemWidget(item, row_widget) 
        
#     def _delete_parameter(self, parameter:SeqParameter):
#         if self.logger:
#             self.logger.info(f"Deleting parameter: {parameter.get_name()}")

#         self.seq_parameter_manager.delete_parameter_by_name(parameter.get_name())

#         # refresh list
#         self._populate_parameter_list()
        
        
        
        
class SeqParameterListWidget(QWidget):
    def __init__(self, seq_parameter_manager: SeqParameterManager, logger=None, parent=None):
        super().__init__(parent)

        self.seq_parameter_manager = seq_parameter_manager
        self.logger = logger

        # --- Table setup ---
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Name", "Type", "Edit", "Delete"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)

        layout = QVBoxLayout(self)
        layout.addWidget(self.table)

        self._populate_parameter_list()

    def _populate_parameter_list(self):
        self.table.setRowCount(0)  # clear table

        for row_index, param in enumerate(self.seq_parameter_manager.get_parameter_list()):
            self.table.insertRow(row_index)

            # --- Name ---
            name_item = QTableWidgetItem(param.get_name())
            self.table.setItem(row_index, 0, name_item)

            # --- Type ---
            type_item = QTableWidgetItem(param.get_type())
            self.table.setItem(row_index, 1, type_item)

            button_pix = 100
            # --- Edit button ---
            edit_button = QPushButton("Edit")
            edit_button.clicked.connect(lambda checked, p=param: self._edit_parameter(p))
            edit_button.setFixedWidth(button_pix)
            self.table.setCellWidget(row_index, 2, edit_button)

            # --- Delete button ---
            delete_button = QPushButton("Delete")
            delete_button.clicked.connect(lambda checked, p=param: self._delete_parameter(p))
            delete_button.setFixedWidth(button_pix)  # set width in pixels
            self.table.setCellWidget(row_index, 3, delete_button)
            #self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
            self.table.resizeColumnsToContents()
            self.table.setColumnWidth(3, button_pix)  # slightly larger than button width
            #self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.Fixed)

            
    def _delete_parameter(self, parameter: SeqParameter):
        if self.logger:
            self.logger.info(f"Deleting parameter: {parameter.get_name()}")

        self.seq_parameter_manager.delete_parameter_by_name(parameter.get_name())
        self._populate_parameter_list()

    def _edit_parameter(self, parameter: SeqParameter):
        dialog = EditParameterDialog(parameter, parent=self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            # you may refresh the table if values are editable
            self._populate_parameter_list()

class EditParameterDialog(QDialog):
    def __init__(self, parameter: SeqParameter, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Edit Sequence Parameter")
        self.resize(600, 400)

        # Buttons
        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Cancel")

        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)

        # Button layout
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)

        # Editor
        _msg = get_message(parameter.get_type())
        message_types = field_type_map_recursive_with_msg_type(_msg)

        editor = ROS2DictEditor(
            types_dict=message_types,
            values_dict=parameter.get_value()
        )

        layout = QVBoxLayout(self)
        layout.addWidget(editor)
        layout.addLayout(button_layout)

        
class StringListDialog(QDialog):
    def __init__(self, items: list[str], parent=None):
        super().__init__(parent)

        self.setWindowTitle("Select Type for new Sequence Parameter")
        self.resize(600, 400)

        self._selected_item = None

        # List widget
        self.list_widget = QListWidget()
        self.list_widget.addItems(items)
        self.list_widget.currentTextChanged.connect(self._on_item_selected)

        # Text field
        self.line_edit = QLineEdit()
        self.line_edit.setPlaceholderText("Enter a unique parameter name...")

        # Buttons
        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Cancel")

        self.ok_button.clicked.connect(self._on_ok)
        self.cancel_button.clicked.connect(self.reject)

        # Layout
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)

        layout = QVBoxLayout(self)
        layout.addWidget(self.list_widget)
        layout.addWidget(self.line_edit)
        layout.addLayout(button_layout)

    def _on_item_selected(self, text: str):
        self._selected_item = text

    def _on_ok(self):
        if self.line_edit.text().strip():
            self.accept()

    def get_result(self) -> tuple[str, str] | None:
        if self.exec() == QDialog.DialogCode.Accepted:
            return self.line_edit.text(), self._selected_item
        return None
    
if __name__ == "__main__":
    pass
