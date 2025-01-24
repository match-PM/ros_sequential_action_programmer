import sys
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QPushButton, QMainWindow, QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QColor, QFont

class StatusIndicator(QWidget):
    def __init__(self, initial_color="green", size=50, text=""):
        super().__init__()
        self.color = initial_color  # Default color of the circle
        self.size = size            # Diameter of the circle
        self.text = text            # Text inside the circle
        self.setFixedSize(size, size)
        self.set_state_idle()

    def set_color(self, color: str):
        """Set the color of the status indicator."""
        self.color = color
        self.update()  # Trigger a repaint

    def set_text(self, text: str):
        """Set the text displayed inside the indicator."""
        self.text = text
        self.update()  # Trigger a repaint

    def paintEvent(self, event):
        """Custom paint event to draw the circle and the text."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw the circle
        painter.setBrush(QColor(self.color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(0, 0, self.size, self.size)

        # Draw the text
        if self.text:
            painter.setPen(Qt.GlobalColor.white)  # Text color
            font = QFont("Arial", int(self.size * 0.2), QFont.Weight.Bold)
            painter.setFont(font)

            # Center the text in the circle
            rect = self.rect()
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, self.text)
    
    def set_state_running(self):
        self.set_color("blue")
        self.set_text("Exec")
        
    def set_state_error(self):
        self.set_color("red")
        self.set_text("Error")
    
    def set_state_idle(self):
        self.set_color("orange")
        self.set_text("Idle")
        
    def set_state_success(self):
        self.set_color("green")
        self.set_text("Done")