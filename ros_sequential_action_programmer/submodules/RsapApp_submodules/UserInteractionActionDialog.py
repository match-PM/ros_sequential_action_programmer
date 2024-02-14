import sys
from PyQt6.QtWidgets import QApplication, QWidget, QTextEdit, QVBoxLayout, QLabel, QLineEdit, QDialog, QDialogButtonBox, QPushButton, QFormLayout

class UserInteractionActionDialog(QDialog):
    def __init__(self, description_text:str, parent=None):
        super(UserInteractionActionDialog, self).__init__(parent)

        self.setWindowTitle("User interaction required")
        self.default_info_text = "Press 'OK' to proceed or press 'Cancel' to abort."
        description_text_widget = QTextEdit()
        description_text_widget.setPlainText(f"{description_text} \n {self.default_info_text}")
        description_text_widget.setReadOnly(True)

        layout = QVBoxLayout(self)

        service_type_label = QLabel("User Input:")
        layout.addWidget(description_text_widget)
        layout.addWidget(service_type_label)

        # add a line edit
        self.line_edit = QLineEdit()
        layout.addWidget(self.line_edit)
        
        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box)
        self.resize(600, 400)

    def get_values(self):

        return None