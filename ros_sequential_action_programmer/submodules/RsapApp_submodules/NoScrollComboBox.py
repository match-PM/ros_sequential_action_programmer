from PyQt6.QtWidgets import QComboBox
from PyQt6.QtCore import Qt

class NoScrollComboBox(QComboBox):
    def wheelEvent(self, event):
        # Prevent the wheel event from being processed further
        event.ignore()