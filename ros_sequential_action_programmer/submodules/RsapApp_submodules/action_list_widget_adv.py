from PyQt6.QtGui import QFont, QPainter, QPen
from PyQt6.QtCore import Qt
from PyQt6 import QtCore

from PyQt6.QtWidgets import (
    QApplication, QListWidget, QListWidgetItem, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QFrame
)

from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction

# import node
from rclpy.node import Node
import rclpy

class NumberLabel(QLabel):
    def __init__(self, number: int, action:ServiceAction):
        super().__init__(str(number))
        self.action = action
        self.number = number
        self.is_selected = action.has_breakpoint()
        self.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(30, 30)

    def mousePressEvent(self, event):
        self.is_selected = not self.is_selected
        self.action.toggle_breakpoint()
        print(f"Action {self.number} selected: {self.action.has_breakpoint()}")
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if self.is_selected:
            painter = QPainter(self)
            pen = QPen(Qt.GlobalColor.blue, 2)
            painter.setPen(pen)
            painter.drawEllipse(1, 1, self.width() - 2, self.height() - 2)

class ActionSequenceListItem(QWidget):
    def __init__(self, 
                 number: int, 
                 text: str,
                 action: ServiceAction):
        
        super().__init__()
        
        main_layout = QVBoxLayout(self)  # Vertical layout to hold the item and separator

        # Row Layout for Number and Text
        self.row_layout = QHBoxLayout()
        self.number_label = NumberLabel(number,
                                action)
        self.text_label = QLabel(text)
        self.text_label.setFont(QFont("Arial", 12))


        self.row_layout.addWidget(self.number_label)
        self.row_layout.addWidget(self.text_label)
        self.row_layout.addStretch()  # Ensures text stays aligned properly
        self.row_layout.setContentsMargins(2, 2, 2, 2)

        # Add separator line
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        separator.setStyleSheet("color: grey;")  # Line color

        main_layout.addLayout(self.row_layout)
        main_layout.addWidget(separator)  # Add the line below the item
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        
        # layout = QHBoxLayout(self)
        # self.number_label = NumberLabel(number,
        #                                 action)
        # self.text_label = QLabel(text)
        # self.text_label.setFont(QFont("Arial", 12))
        # layout.addWidget(self.number_label)
        # layout.addWidget(self.text_label)
        # layout.alignment(
        # layout.setContentsMargins(5, 5, 5, 5)
        

class ActionSequenceListWidget(QListWidget):
    def __init__(self, rsap_sequence: RosSequentialActionProgrammer):
        super().__init__()
        self.rsap_sequence = rsap_sequence
        self.setAcceptDrops(True)
        self.setDragEnabled(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QListWidget.DragDropMode.InternalMove)
        self.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)

    
    def populate_list(self):
        self.clear()
        for i, action in enumerate(self.rsap_sequence.action_list):
            item = QListWidgetItem(self)
            widget = ActionSequenceListItem(number = i + 1, 
                                            text = action.name,
                                            action=action)
            
            # Set style for inactive actions
            if not action.is_active():
                widget.text_label.setStyleSheet("color: grey;")
                #widget.setStyleSheet("background-color: lightgrey; border-radius: 5px; padding: 5px;")
                widget.number_label.setStyleSheet("background-color: lightgrey; border-radius: 5px; padding: 5px;")
                widget.text_label.setStyleSheet("background-color: lightgrey; border-radius: 5px; padding: 5px;")
                
            item.setSizeHint(widget.sizeHint())
            self.addItem(item)
            self.setItemWidget(item, widget)

    def get_widget_list_names(self):
        name_list=[]
        for index in range(self.count()):
            item = self.item(index)
            if item:
                name_list.append(item.text())
        return name_list
    
    def dropEvent(self, event):
        super(ActionSequenceListWidget, self).dropEvent(event)
        event.accept()
        self.on_action_drag_drop() 
        self.clear()     
        self.populate_list()
    
    def on_action_drag_drop(self):
        self.rsap_sequence.move_action_at_index_to_index(old_index=self.drag_source_position,
                                                            new_index=self.currentRow())

    def dragEnterEvent(self, event):
        self.drag_source_position = self.currentRow()  # Capture the source position
        super(ActionSequenceListWidget, self).dragEnterEvent(event)


if __name__ == "__main__":
    import sys
    rclpy.init()
    test_node = Node("test_node")
    test_sequence = RosSequentialActionProgrammer(test_node)
    test_sequence.load_from_JSON("/home/niklas/ros2_ws/src/ros_sequential_action_programmer/documentation/examples/turtle_sim_example.json")
    
    for ind, action in enumerate(test_sequence.action_list):
        if ind == 1 or ind == 3:
            action.toggle_active()
        print(action.is_active())
        
    app = QApplication(sys.argv)
    window = ActionSequenceListWidget(test_sequence)
    window.populate_list()
    window.show()
    sys.exit(app.exec())
