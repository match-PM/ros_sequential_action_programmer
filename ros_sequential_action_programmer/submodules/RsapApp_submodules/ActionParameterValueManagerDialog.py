from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QTabWidget, QWidget, QLabel, QPushButton,
    QListWidget, QListWidgetItem,QHBoxLayout,QComboBox,QTreeWidget, QTreeWidgetItem
)
from PyQt6.QtCore import Qt
from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import ActionParameterValueManager
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass  
from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import RefKeyList, RefKeyListElement
from ros_sequential_action_programmer.submodules.action_classes.ParameterValueSetGenerator import ValuesSet
from ros_sequential_action_programmer.submodules.obj_dict_modules.obj_functions import get_obj_value_from_key, set_obj_value_from_key, get_last_index_value
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameter
from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import SeqParameterReference, ActionResponseParameterReference

# class ReferenceListItem(QWidget):
#     def __init__(self, key: str, action_index: int, action_name: str):
#         super().__init__()
#         layout = QHBoxLayout(self)
#         layout.setContentsMargins(2, 2, 2, 2)
#         layout.setSpacing(10)

#         # Create labels in the desired order: index, name, key
#         self.index_label = QLabel(str(action_index))
#         self.action_name_label = QLabel(action_name)
#         self.key_label = QLabel(key)

#         # Optional: style for clarity
#         self.index_label.setStyleSheet("color: gray;")
#         self.action_name_label.setStyleSheet("color: darkblue;")
#         self.key_label.setStyleSheet("font-weight: bold;")

#         # Set fixed width for consistent alignment
#         self.index_label.setFixedWidth(40)
#         self.action_name_label.setFixedWidth(200)
#         #self.key_label.setFixedWidth(label_width)

#         # Add widgets in order
#         layout.addWidget(self.index_label)
#         layout.addWidget(self.action_name_label)
#         layout.addWidget(self.key_label)

#         layout.addStretch()  # push everything to the left

# class SeqParameterListItem(QWidget):
#     def __init__(self, parameter:SeqParameter):
#         super().__init__()
#         layout = QHBoxLayout(self)
#         layout.setContentsMargins(2, 2, 2, 2)
#         layout.setSpacing(10)

#         self.name_label = QLabel(parameter.get_name())
#         #self.type_label = QLabel(parameter.get_type())
#         self.value_label = QLabel(str(parameter.get_value()))

#         layout.addWidget(self.name_label)
#         #layout.addWidget(self.type_label)
#         layout.addWidget(self.value_label)

class ActionParameterValueManagerDialog(QDialog):

    RERFERENCE_TAB_SEQUENCE_PARAMETERS = "Sequence Parameters"
    RERFERENCE_TAB_ACTION_RESPONSES = "Action Responses"

    def __init__(self,
                 current_action: ActionBaseClass,
                 action_parameter_value_manager: ActionParameterValueManager,
                 current_type: str,
                 current_values_dict: dict,
                 current_field_key: str,
                 add_references = True,
                 add_equations = True,
                 parent=None):
        
        super().__init__(parent)
        self.setWindowTitle(f"Field Editor - type: '{current_type}'")
        self.resize(600, 400)
        
        self.current_action = current_action
        self.action_parameter_value_manager = action_parameter_value_manager
        self.current_type = current_type
        self.values_dict = current_values_dict
        self.current_field_key = current_field_key
        self.add_references = add_references
        self.add_equations = add_equations
        self._selected_value_object_return = None  # store selected value object
        
        main_layout = QVBoxLayout(self)
        self.setLayout(main_layout)

        # Create tabs
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # ------------------ Tab 1: Values ------------------
        self.values_tab = QWidget()
        self.values_layout = QVBoxLayout(self.values_tab)
        self.values_tab.setLayout(self.values_layout)
        self.tabs.addTab(self.values_tab, "Values")

        self.populate_values_tab()
        # ------------------ Tab 2: References ------------------
        if add_references:
            self.references_tab = QWidget()
            self.references_layout = QVBoxLayout(self.references_tab)
            self.references_tab.setLayout(self.references_layout)
            self.tabs.addTab(self.references_tab, "References")

            # Populate References tab
            self.init_references_tab()

        # ------------------ Tab 3: Equations ------------------
        if add_equations:
            self.equations_tab = QWidget()
            self.equations_layout = QVBoxLayout(self.equations_tab)
            self.equations_tab.setLayout(self.equations_layout)
            self.equations_layout.addWidget(QLabel("Equations content goes here"))
            self.tabs.addTab(self.equations_tab, "Equations")

        # ------------------ Buttons ------------------
        button_layout = QHBoxLayout()
        main_layout.addLayout(button_layout)

        self.ok_button = QPushButton("OK")
        self.ok_button.clicked.connect(self.on_ok)
        button_layout.addWidget(self.ok_button)

        self.close_button = QPushButton("Close")
        self.close_button.clicked.connect(self.reject)
        button_layout.addWidget(self.close_button)

    def init_references_tab(self):
        """Populate the References tab with a scrollable list of keys."""

        if not self.add_references:
            return
        
        # Create a dropdown combo box
        self.reference_dropdown = QComboBox()
        self.reference_dropdown.addItems([self.RERFERENCE_TAB_SEQUENCE_PARAMETERS, self.RERFERENCE_TAB_ACTION_RESPONSES])

        # when a value changes
        self.reference_dropdown.currentTextChanged.connect(self.populate_references_tab)
        self.reference_dropdown.setCurrentIndex(0)

        # Label and dropdown
        self.references_layout.addWidget(QLabel("Select the reference source:"))
        self.references_layout.addWidget(self.reference_dropdown)

        self.references_values_layout = QVBoxLayout()
        self.references_layout.addLayout(self.references_values_layout)
        self.references_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.populate_references_tab()


    def populate_references_tab(self):
        """Populate the References tab dynamically based on selected source."""
        self.clear_layout(self.references_values_layout)
        current_reference_type = self.reference_dropdown.currentText()

        # === Sequence Parameters ===
        if current_reference_type == self.RERFERENCE_TAB_SEQUENCE_PARAMETERS:
            manager = self.action_parameter_value_manager.seq_parameter_manager

            if not manager or not manager.get_is_initialized():
                self.references_values_layout.addWidget(QLabel("Sequence Parameter Manager File has not been loaded."))
                return

            current_file_name = manager.get_file_name()
            self.references_values_layout.addWidget(QLabel(f"Current Sequence Parameter File: {current_file_name}"))

            param_list = manager.get_parameters_for_type(self.current_type)
            if not param_list:
                self.references_values_layout.addWidget(QLabel("No parameters available for this type."))
                return

            # Create a QTreeWidget instead of a QListWidget
            self.reference_val_tree = QTreeWidget()
            self.reference_val_tree.setHeaderLabels(["Name", "Type", "Value"])
            self.reference_val_tree.header().setStretchLastSection(True)
            self.reference_val_tree.setAlternatingRowColors(True)
            self.reference_val_tree.setRootIsDecorated(False)
            self.reference_val_tree.setSelectionBehavior(QTreeWidget.SelectionBehavior.SelectRows)
            self.references_values_layout.addWidget(self.reference_val_tree)

            for param in param_list:
                param_name = param.get_name()
                param_type = param.get_type()
                param_value = str(param.get_value())
                item = QTreeWidgetItem(self.reference_val_tree, [param_name, param_type, param_value])
                item.setData(0, Qt.ItemDataRole.UserRole, param)

        # === Action Responses ===
        elif current_reference_type == self.RERFERENCE_TAB_ACTION_RESPONSES:
            ref_key_list = self.action_parameter_value_manager.get_comp_res_ref_keys(
                target_action=self.current_action,
                field_type=self.current_type
            )

            if not ref_key_list.get_list():
                self.references_values_layout.addWidget(QLabel("No action response references available."))
                return

            # Create a QTreeWidget with headers
            self.reference_val_tree = QTreeWidget()
            self.reference_val_tree.setHeaderLabels(["Key", "Action Index", "Action Name"])
            self.reference_val_tree.header().setStretchLastSection(True)
            self.reference_val_tree.setAlternatingRowColors(True)
            self.reference_val_tree.setRootIsDecorated(False)
            self.reference_val_tree.setSelectionBehavior(QTreeWidget.SelectionBehavior.SelectRows)
            self.references_values_layout.addWidget(self.reference_val_tree)

            for ref_key in ref_key_list.get_list():
                item = QTreeWidgetItem(self.reference_val_tree, [str(ref_key.key), str(ref_key.action_index), str(ref_key.action_name)])
                item.setData(0, Qt.ItemDataRole.UserRole, ref_key)

        else:
            return
    
    def populate_values_tab(self):
        """
        Populate the Values tab with:
          - a dropdown of value headers
          - a list widget of values for the currently selected header
        """
        # Get the list of headers
        values_header = self.action_parameter_value_manager.get_value_headers(self.current_type)

        # Create a dropdown combo box
        self.values_dropdown = QComboBox()
        self.values_dropdown.addItems(values_header)

        # Label and dropdown
        self.values_layout.addWidget(QLabel("Select a Value Header:"))
        self.values_layout.addWidget(self.values_dropdown)

        # Create the list widget to show the values of the selected header
        self.values_list_widget = QListWidget()
        self.values_layout.addWidget(QLabel("Values in Selected Set:"))
        self.values_layout.addWidget(self.values_list_widget)

        # Update the list for the initial selection (if any)
        if values_header:
            self.update_values_list(values_header[0])

        # Connect change signal to update the list dynamically
        self.values_dropdown.currentTextChanged.connect(self.update_values_list)

    def update_values_list(self, current_header: str):
        """
        Update the QListWidget with all values for the given header.
        """
        self.values_list_widget.clear()
        if not current_header:
            return

        # Retrieve the values for this header from the manager
        value_set: ValuesSet = self.action_parameter_value_manager.get_value_set_for_header(current_header)
        #

        # Add each value as a list item
        for val in value_set.get_values_list():
            item = WidgetValuesSetItem(str(val), val)
            self.values_list_widget.addItem(item)

    def on_ok(self):
        """
        Collect selected reference, selected value set header,
        and the actual selected value depending on which tab is active.
        """
        # Store which tab the user is on
        current_tab_index = self.tabs.currentIndex()
        current_tab_name = self.tabs.tabText(current_tab_index)
        self.selected_tab = current_tab_name

        # Initialize
        self.selected_value_header = None
        self.selected_value_item = None
        #self._selected_value_object_return = None

        # ---------------- Values tab ----------------
        if current_tab_name == "Values":
            # Which header is currently selected
            if hasattr(self, "values_dropdown"):
                self.selected_value_header = self.values_dropdown.currentText()

            # Which value (list item) is selected
            selected_items = self.values_list_widget.selectedItems()
            if selected_items:
                selected_item = selected_items[0]
                self.selected_value_item = selected_item.text()

                # Get the underlying Python value from your custom QListWidgetItem
                if isinstance(selected_item, WidgetValuesSetItem):
                    #self._selected_value_object_return = selected_item.get_value()
                    set_success = set_obj_value_from_key(self.values_dict, 
                                                        str(self.current_field_key),
                                                         selected_item.get_value())
                    #self.current_action.set_request_dict_value_from_key(self.current_field_key, self._selected_value_object_return)

        # ---------------- References tab ----------------
        elif current_tab_name == "References" and self.add_references:
            # Detect which sub-type of reference is being displayed
            if hasattr(self, "reference_dropdown"):
                ref_type = self.reference_dropdown.currentText()
            else:
                ref_type = None

            # === Sequence Parameters ===
            if ref_type == self.RERFERENCE_TAB_SEQUENCE_PARAMETERS:
                selected_items = self.reference_val_tree.selectedItems()
                if selected_items:
                    item = selected_items[0]
                    param: SeqParameter = item.data(0, Qt.ItemDataRole.UserRole)
                    self.current_action.get_references().add_reference(
                        SeqParameterReference(value_key=self.current_field_key, param=param)
                    )
                    self.current_action.node.get_logger().info(f"Selected SeqParameter: {param.get_name()} with value: {param.get_value()}")

            # === Action Responses ===
            elif ref_type == self.RERFERENCE_TAB_ACTION_RESPONSES:
                selected_items = self.reference_val_tree.selectedItems()
                if selected_items:
                    item = selected_items[0]
                    # columns: Key, Action Index, Action Name
                    ref_key: RefKeyListElement = item.data(0, Qt.ItemDataRole.UserRole)

                    self.current_action.get_references().add_reference(
                        ActionResponseParameterReference(
                            value_key=self.current_field_key,
                            reference_key=ref_key.key,
                            action_pointer=self.action_parameter_value_manager.get_action_at_index(ref_key.action_index)
                        )
                    )
        # ---------------- Equations tab ----------------
        elif current_tab_name == "Equations" and self.add_equations:
            # Future: collect equation data here
            pass
            
        # Close the dialog with success
        self.accept()

    # def get_selected_value_object(self):
    #     return self._selected_value_object_return

    def clear_layout(self, layout):
        """Helper: safely clear all widgets and sublayouts from a layout."""
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            elif item.layout():
                self.clear_layout(item.layout())

class WidgetValuesSetItem(QListWidgetItem):
    def __init__(self, disp_string, value):
        super().__init__(disp_string)
        self._value = value

    def get_value(self):
        return self._value