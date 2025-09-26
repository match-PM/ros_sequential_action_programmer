from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QTabWidget, QWidget, QLabel, QPushButton,
    QListWidget, QListWidgetItem,QHBoxLayout,QComboBox
)
from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import ActionParameterValueManager

from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import RefKeyList, RefKeyListElement


class ReferenceListItem(QWidget):
    def __init__(self, key: str, action_index: int, action_name: str):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(10)

        # Create labels in the desired order: index, name, key
        self.index_label = QLabel(str(action_index))
        self.action_name_label = QLabel(action_name)
        self.key_label = QLabel(key)

        # Optional: style for clarity
        self.index_label.setStyleSheet("color: gray;")
        self.action_name_label.setStyleSheet("color: darkblue;")
        self.key_label.setStyleSheet("font-weight: bold;")

        # Set fixed width for consistent alignment
        self.index_label.setFixedWidth(40)
        self.action_name_label.setFixedWidth(200)
        #self.key_label.setFixedWidth(label_width)

        # Add widgets in order
        layout.addWidget(self.index_label)
        layout.addWidget(self.action_name_label)
        layout.addWidget(self.key_label)

        layout.addStretch()  # push everything to the left
        
class ActionParameterValueManagerDialog(QDialog):
    def __init__(self,
                 current_action,
                 action_parameter_value_manager: ActionParameterValueManager,
                 current_type: str,
                 add_references = True,
                 add_equations = True,
                 parent=None):
        
        super().__init__(parent)
        self.setWindowTitle(f"Field Editor - type: '{current_type}'")
        self.resize(600, 400)
        
        self.action = current_action
        self.action_parameter_value_manager = action_parameter_value_manager
        self.current_type = current_type
        self.selected_reference = None  # store selected reference
        self.add_references = add_references
        self.add_equations = add_equations
        
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
            self.populate_references_tab()

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

    def populate_references_tab(self):
        """Populate the References tab with a scrollable list of keys."""
        ref_key_list = self.action_parameter_value_manager.get_comp_res_ref_keys(
            target_action=self.action,
            field_type=self.current_type
        )
        
        self.references_list_widget = QListWidget()
        self.references_layout.addWidget(self.references_list_widget)

        for ref_key in ref_key_list.get_list():
            ref_key: RefKeyListElement
            keys = ref_key.key
            action_index = ref_key.action_index
            action_name = ref_key.action_name
            for elem in keys:
                list_item = QListWidgetItem(self.references_list_widget)
                custom_widget = ReferenceListItem(elem, action_index, action_name)
                self.references_list_widget.setItemWidget(list_item, custom_widget)
                list_item.setSizeHint(custom_widget.sizeHint())

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
        values_for_header = self.action_parameter_value_manager.get_values_for_set_header(current_header)

        # Add each value as a list item
        for val in values_for_header:
            QListWidgetItem(str(val), self.values_list_widget)
        
                
    def on_ok(self):
        """
        Collect selected reference and selected value set header.
        """
        # Selected reference (if References tab is present)
        if self.add_references:
            selected_items = self.references_list_widget.selectedItems()
            if selected_items:
                self.selected_reference = selected_items[0].text()

        # Store the currently chosen value header
        if hasattr(self, "values_dropdown"):
            self.selected_value_header = self.values_dropdown.currentText()

        # Optionally also capture the chosen value inside the list widget
        self.selected_value_item = None
        if hasattr(self, "values_list_widget"):
            selected_values = self.values_list_widget.selectedItems()
            if selected_values:
                self.selected_value_item = selected_values[0].text()

        self.accept()