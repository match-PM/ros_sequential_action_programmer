import sys
from PyQt6.QtWidgets import QApplication, QWidget, QTextEdit, QVBoxLayout, QLabel, QLineEdit, QDialog, QDialogButtonBox, QPushButton, QFormLayout

class UserInteractionActionDialog(QDialog):
    def __init__(self, description_text:str, parent=None):
        super(UserInteractionActionDialog, self).__init__(parent)

        self.setWindowTitle("User interaction required")
        description_text_widget = QTextEdit(f"{description_text}")
        description_text_widget.setReadOnly(True)

        layout = QVBoxLayout(self)

        service_type_label = QLabel("Enter Something:")
        layout.addWidget(description_text_widget)
        layout.addWidget(service_type_label)

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box)
        self.resize(600, 400)

    def get_values(self):

        return None