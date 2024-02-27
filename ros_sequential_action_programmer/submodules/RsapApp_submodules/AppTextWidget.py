from PyQt6.QtWidgets import QTextEdit
from PyQt6.QtGui import QColor

class AppTextOutput(QTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)

    def append_red_text(self, text:str) -> None:
        self.setTextColor(QColor("red"))
        self.append(text)
        self.setTextColor(QColor("black"))

    def append_green_text(self, text:str) -> None:
        self.setTextColor(QColor("green"))
        self.append(text)
        self.setTextColor(QColor("black"))