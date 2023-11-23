import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QDialog, QDialogButtonBox, QPushButton, QFormLayout

class AddServiceDialog(QDialog):
    def __init__(self, service_name='', service_client='', service_type='', parent=None):
        super(AddServiceDialog, self).__init__(parent)

        self.setWindowTitle("Configure Serivce Call Action")

        layout = QVBoxLayout(self)

        self.service_name_textbox = QLineEdit(service_name)
        self.service_client_textbox = QLineEdit(service_client)
        self.service_type_textbox = QLineEdit(service_type)

        #if service_name == '':
        service_name_label = QLabel("Enter Action Name:")
        layout.addWidget(service_name_label)
        layout.addWidget(self.service_name_textbox)

        #if service_client == '':
        service_client_label = QLabel("Enter Service Client:")
        layout.addWidget(service_client_label)
        layout.addWidget(self.service_client_textbox)

        #if service_type == '':
        service_type_label = QLabel("Enter Service Type:")
        layout.addWidget(service_type_label)
        layout.addWidget( self.service_type_textbox)

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box)
        self.resize(800, 200)

    def get_values(self):
        if self.service_name_textbox.text() == '':
            service_name = None
        else:
            service_name = self.service_name_textbox.text()
        if self.service_client_textbox.text() == '':
            service_client = None
        else:
            service_client = self.service_client_textbox.text()
        if self.service_type_textbox.text() == '':
            service_type = None
        else:
            service_type = self.service_type_textbox.text()
        return service_name, service_client, service_type