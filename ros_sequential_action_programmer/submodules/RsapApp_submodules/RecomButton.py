from PyQt6.QtWidgets import QHBoxLayout, QWidget, QVBoxLayout, QPushButton, QLineEdit,  QLabel
from functools import partial


class RecomButton(QWidget):
    def __init__(self, full_key, initial_value, on_text_changed, on_button_clicked):
        super().__init__()

        label = QLabel(full_key)
        edit = QLineEdit(initial_value)
        edit.textChanged.connect(on_text_changed(full_key, edit))

        button = QPushButton("+")
        button.clicked.connect(partial(on_button_clicked,full_key,edit))

        label_layout = QHBoxLayout()
        edit_button_layout = QHBoxLayout()

        edit_button_layout.addWidget(edit)
        edit_button_layout.addWidget(button)
        
        label_layout.addWidget(label)

        main_layout = QVBoxLayout()
        main_layout.addLayout(label_layout)
        main_layout.addLayout(edit_button_layout)
        self.setLayout(main_layout)
