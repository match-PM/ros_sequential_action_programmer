from PyQt6.QtWidgets import (
    QApplication, QWidget, QListWidget, QListWidgetItem, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel
)
import sys

class CheckableNumberLabel(QCheckBox):
    def __init__(self, number):
        super().__init__(str(number))
        self.setStyleSheet(
            "QCheckBox { border: none; font-size: 14px; }"
            "QCheckBox::indicator { width: 0px; height: 0px; }"
        )
        self.stateChanged.connect(self.updateStyle)
        self.updateStyle()
    
    def updateStyle(self):
        if self.isChecked():
            self.setStyleSheet(
                "QCheckBox { border: 2px solid black; border-radius: 12px; padding: 2px; font-size: 14px; }"
                "QCheckBox::indicator { width: 0px; height: 0px; }"
            )
        else:
            self.setStyleSheet(
                "QCheckBox { border: none; font-size: 14px; }"
                "QCheckBox::indicator { width: 0px; height: 0px; }"
            )

class ListItemWidget(QWidget):
    def __init__(self, number):
        super().__init__()
        layout = QHBoxLayout()
        
        self.number_label = CheckableNumberLabel(number)
        self.active_checkbox = QCheckBox("Active")
        
        layout.addWidget(self.number_label)
        layout.addWidget(self.active_checkbox)
        
        self.setLayout(layout)

class CustomListWidget(QListWidget):
    def __init__(self):
        super().__init__()
        self.populate_list()
    
    def populate_list(self, count=10):
        for i in range(1, count + 1):
            item = QListWidgetItem(self)
            item_widget = ListItemWidget(i)
            
            item.setSizeHint(item_widget.sizeHint())
            self.addItem(item)
            self.setItemWidget(item, item_widget)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        
        self.list_widget = CustomListWidget()
        layout.addWidget(self.list_widget)
        
        self.setLayout(layout)
        self.setWindowTitle("Custom List Widget")
        self.resize(300, 400)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())