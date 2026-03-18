import sys
from PyQt6.QtWidgets import QApplication, QWidget,QTextEdit, QVBoxLayout, QLabel, QLineEdit, QDialog, QDialogButtonBox, QPushButton, QFormLayout

class UserDialog(QDialog):
    def __init__(self, request_text:str, parent=None):
        super(UserDialog, self).__init__(parent)

        self.setWindowTitle("User Interaction Required...")

        layout = QVBoxLayout(self)

        description = QLabel(request_text)

        layout.addWidget(description)        

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box)
        self.resize(600, 400)
