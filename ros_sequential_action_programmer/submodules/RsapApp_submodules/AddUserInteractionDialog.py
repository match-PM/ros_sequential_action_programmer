import sys
from PyQt6.QtWidgets import QApplication, QWidget,QTextEdit, QVBoxLayout, QLabel, QLineEdit, QDialog, QDialogButtonBox, QPushButton, QFormLayout

class AddUserInteractionDialog(QDialog):
    def __init__(self, parent=None):
        super(AddUserInteractionDialog, self).__init__(parent)

        self.setWindowTitle("Configure Serivce Call Action")

        layout = QVBoxLayout(self)

        #if service_name == '':
        action_name = QLabel("Enter Name of the action:")
        self.action_name_widget = QLineEdit()
        self.description_box = QTextEdit()

        description = QLabel("Enter User Interaction Text:")
        
        layout.addWidget(action_name)
        layout.addWidget(self.action_name_widget)
        

        layout.addWidget(description)
        layout.addWidget(self.description_box)


        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box)
        self.resize(600, 400)

    def get_values(self):
        return self.action_name_widget.text(), self.description_box.toPlainText()