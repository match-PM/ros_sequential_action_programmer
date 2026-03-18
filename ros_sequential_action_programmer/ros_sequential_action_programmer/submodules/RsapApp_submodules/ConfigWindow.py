import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QPushButton, QMessageBox, QWidget, QDialog,
    QTableWidget, QTableWidgetItem, QHeaderView, QCheckBox, QSpinBox, QDoubleSpinBox, QLineEdit, QComboBox
)
from PyQt6.QtCore import Qt


class DictionaryValueEditor(QWidget):
    def __init__(self, dictionary=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Dictionary Editor")
        self.resize(600, 400)

        # Initialize dictionary
        self.dictionary = dictionary if dictionary is not None else {}

        # Main layout
        layout = QVBoxLayout(self)

        # Dictionary table
        self.table = QTableWidget(self)
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Key", "Value"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table)

        # Load the initial dictionary into the table
        self.update_table()

    def update_table(self):
        """Populate the table based on the current dictionary."""
        # Filter out keys starting with "_fields_" (case-insensitive)
        filtered_items = {
            key: value for key, value in self.dictionary.items()
            if not key.lower().startswith("_fields_")
        }

        self.table.setRowCount(len(filtered_items))
        for row, (key, value) in enumerate(filtered_items.items()):
            # Set key (read-only)
            key_item = QTableWidgetItem(str(key))
            key_item.setFlags(Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled)
            self.table.setItem(row, 0, key_item)

            # Create value widget or fallback item
            value_widget = self.create_widget_for_value(key, value)
            if value_widget:
                self.table.setCellWidget(row, 1, value_widget)
            else:
                # Fallback to QTableWidgetItem for unsupported types
                value_item = QTableWidgetItem(str(value))
                value_item.setFlags(Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled)
                self.table.setItem(row, 1, value_item)



    def create_widget_for_value(self, key, value):
        """Create a widget appropriate for the value's type."""
        # Handle dropdown fields if a `_fields_<key>` key exists
        field_key = f"_fields_{key.lower()}"
        if field_key in self.dictionary and isinstance(self.dictionary[field_key], list):
            return self.create_selector_widget(key, self.dictionary[field_key])

        if isinstance(value, bool):
            checkbox = QCheckBox()
            checkbox.setChecked(value)
            checkbox.toggled.connect(lambda checked: self.update_value(key, checked))
            return checkbox

        elif isinstance(value, int):
            spin_box = QSpinBox()
            spin_box.setValue(value)
            spin_box.setRange(-1_000_000, 1_000_000)
            spin_box.valueChanged.connect(lambda new_value: self.update_value(key, new_value))
            return spin_box

        elif isinstance(value, float):
            double_spin_box = QDoubleSpinBox()
            double_spin_box.setValue(value)
            double_spin_box.setRange(-1_000_000.0, 1_000_000.0)
            double_spin_box.setDecimals(4)
            double_spin_box.valueChanged.connect(lambda new_value: self.update_value(key, new_value))
            return double_spin_box

        elif isinstance(value, str):
            line_edit = QLineEdit(value)
            line_edit.editingFinished.connect(lambda: self.update_value(key, line_edit.text()))
            return line_edit

        elif isinstance(value, dict):
            button = QPushButton("--Edit Configuration--")
            button.clicked.connect(lambda: self.open_nested_editor(key, value))
            return button

        # For unsupported types, return None (to fallback to QTableWidgetItem)
        return None


    def create_selector_widget(self, key, options):
        """Create a dropdown widget for parameterized selection."""
        combo_box = QComboBox()
        combo_box.addItems(options)

        # Set current value if it matches one of the options
        current_value = self.dictionary.get(key, "")
        if current_value in options:
            combo_box.setCurrentText(current_value)

        combo_box.currentTextChanged.connect(lambda selected: self.update_value(key, selected))
        return combo_box

    def update_value(self, key, new_value):
        """Update the dictionary when a value changes."""
        self.dictionary[key] = new_value

    def open_nested_editor(self, key, value):
        """Open a nested dictionary editor."""
        if not isinstance(value, dict):
            QMessageBox.warning(self, "Error", "Value is not a dictionary.")
            return

        dialog = NestedDictionaryEditor(value, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.dictionary[key] = dialog.dictionary
            self.update_table()


class NestedDictionaryEditor(QDialog):
    def __init__(self, dictionary, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configuration Editor")
        self.resize(500, 300)

        self.dictionary = dictionary

        # Main layout
        layout = QVBoxLayout(self)

        # Dictionary editor
        self.editor = DictionaryValueEditor(self.dictionary, self)
        layout.addWidget(self.editor)

        # OK and Cancel buttons
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(self.accept)
        layout.addWidget(ok_button)

        #cancel_button = QPushButton("Cancel")
        #cancel_button.clicked.connect(self.reject)
        #layout.addWidget(cancel_button)


class ConfigurableApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("App with Configurable Settings")
        self.resize(800, 600)

        # Example configuration (initial dictionary)
        self.config = {
            "app_name": "MyApp",
            "version": 1.0,
            "debug": True,
            "window_size": {
                "width": 1024,
                "height": 768
            },
            "features": {
                "auto_update": False,
                "theme": "dark"
            }
        }

        # Main layout
        central_widget = QWidget(self)
        layout = QVBoxLayout(central_widget)

        # Button to open the config editor
        open_config_button = QPushButton("Edit Configuration")
        open_config_button.clicked.connect(self.open_config_editor)
        layout.addWidget(open_config_button)

        self.setCentralWidget(central_widget)

    def open_config_editor(self):
        """Open the configuration editor and handle changes."""
        config_editor = NestedDictionaryEditor(self.config, self)
        result = config_editor.exec()

        if result == QDialog.DialogCode.Accepted:
            self.config = config_editor.dictionary
            self.apply_config_changes()

    def apply_config_changes(self):
        """Apply changes to the app based on the updated configuration."""
        QMessageBox.information(
            self,
            "Configuration Updated",
            f"New Configuration:\n{self.config}",
            QMessageBox.StandardButton.Ok,
        )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = ConfigurableApp()
    main_window.show()
    sys.exit(app.exec())
