from PyQt6.QtWidgets import QWidget, QLabel, QGridLayout
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer

class SequenceInfoWidget(QWidget):
    def __init__(self, rsap: RosSequentialActionProgrammer):
        super().__init__()
        self.rsap = rsap
        # Create layout
        layout = QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setHorizontalSpacing(15)
        layout.setVerticalSpacing(10)

        # --- Labels ---
        name_label = QLabel("Sequence Name:")
        name_label.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        name_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        file_label = QLabel("Process Parameter File:")
        file_label.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        file_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        # --- Values ---
        self.name_value = QLabel()
        self.name_value.setFont(QFont("Arial", 11))
        self.name_value.setStyleSheet("color: navy;")

        self.parameter_file_value = QLabel()
        self.parameter_file_value.setFont(QFont("Arial", 11))
        self.parameter_file_value.setStyleSheet("color: darkgreen;")

        # --- Add to grid layout ---
        layout.addWidget(name_label, 0, 0)
        layout.addWidget(self.name_value, 0, 1)

        layout.addWidget(file_label, 1, 0)
        layout.addWidget(self.parameter_file_value, 1, 1)

        # Optional: make second column stretchable
        layout.setColumnStretch(1, 1)

        self.setLayout(layout)
        self.init_values()

    def init_values(self):
        if self.rsap.rsap_file_manager.get_sequence_name() is None:
            self.set_sequence_name('- No name given - ')
        else:
            self.set_sequence_name(self.rsap.rsap_file_manager.get_sequence_name())
            self.name_value.setToolTip(f"{self.rsap.rsap_file_manager.get_action_sequence_file_path()}")

        if self.rsap.rsap_file_manager.seq_parameter_manager.get_file_name() is None:
            self.set_parameter_file_name('No file selected')
        else:
            self.set_parameter_file_name(self.rsap.rsap_file_manager.seq_parameter_manager.get_file_name(True))
            self.parameter_file_value.setToolTip(f"{self.rsap.rsap_file_manager.seq_parameter_manager.get_file_dir()}/{self.rsap.rsap_file_manager.seq_parameter_manager.get_file_name()}")

    def set_sequence_name(self, name: str):
        """Update the sequence name display."""
        self.name_value.setText(name)
    
    def set_parameter_file_name(self, file_name: str):
        """Update the process parameter file display."""
        self.parameter_file_value.setText(file_name)
