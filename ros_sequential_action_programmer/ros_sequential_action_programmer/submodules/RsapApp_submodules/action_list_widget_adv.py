from PyQt6.QtGui import QFont, QPainter, QPen, QAction
from PyQt6.QtCore import Qt
from PyQt6 import QtCore

from PyQt6.QtWidgets import (
    QApplication, QListWidget, QListWidgetItem, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QMessageBox, QAbstractItemView, QMenu
)
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction, ActionBaseClass
from ros_sequential_action_programmer.submodules.RsapApp_submodules.AppTextWidget import AppTextOutput

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
    def __init__(self, rsap_sequence: RosSequentialActionProgrammer, 
                 text_output: AppTextOutput):
        
        super().__init__()
        self.rsap_sequence = rsap_sequence
        self.text_output = text_output
        self.setAcceptDrops(True)
        self.setDragEnabled(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QListWidget.DragDropMode.InternalMove)
        self.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)


    # --- ✅ Context Menu for Right-Click ---
    def contextMenuEvent(self, event):
        # Get the item under the cursor
        item = self.itemAt(event.pos())
        if not item:
            return  # Clicked on empty area — ignore

        # Find its index and associated action
        row = self.row(item)
        action = self.rsap_sequence.action_list[row]

        # Build the context menu
        menu = QMenu(self)
        clear_log_action = QAction("Clear Log", self)
        deactivate_action = QAction(
            "Deactivate Action" if action.is_active() else "Activate Action", self
        )

        menu.addAction(clear_log_action)
        menu.addAction(deactivate_action)

        # Connect menu actions
        clear_log_action.triggered.connect(lambda: self.clear_action_log(action))
        deactivate_action.triggered.connect(lambda: self.toggle_action_state(action, row))

        # Show menu at cursor
        menu.exec(event.globalPos())

    def clear_action_log(self, action: ActionBaseClass):
        """Clears the log of the selected action."""
        action.clear_log_entry()
        self.text_output.append(f"Cleared log for action '{action.get_name()}'")

    def toggle_action_state(self, action: ActionBaseClass, row: int):
        """Toggles active/inactive state of an action."""
        try:
            if action.is_active():
                action.set_active(False)
                self.text_output.append(f"Deactivated '{action.get_name()}'")
            else:
                action.set_active(True)
                self.text_output.append(f"Activated '{action.get_name()}'")
            self.populate_list()
            self.setCurrentRow(row)
        except Exception as e:
            self.text_output.append(f"Error toggling action state: {str(e)}")


    def populate_list(self):
        self.clear()
        for i, action in enumerate(self.rsap_sequence.action_list):
            
            #self.rsap_sequence.node.get_logger().error(f"{str(action)}")
            
            item = QListWidgetItem(self)
            widget = ActionSequenceListItem(number = i + 1, 
                                            text = action.get_name(),
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
    
    # def dropEvent(self, event):
    #     super(ActionSequenceListWidget, self).dropEvent(event)
    #     event.accept()
    #     self.on_action_drag_drop() 
    #     self.clear()     
    #     self.populate_list()

    def dropEvent(self, event):
        drop_row = self.indexAt(event.position().toPoint()).row()
        super().dropEvent(event)
        event.accept()

        self.on_action_drag_drop()
        scroll_value = self.verticalScrollBar().value()

        self.clear()
        self.populate_list()

        # Scroll to where the drop happened
        if 0 <= drop_row < self.count():
            self.setCurrentRow(drop_row)
            self.scrollToItem(self.item(drop_row), QAbstractItemView.ScrollHint.PositionAtCenter)
        else:
            self.verticalScrollBar().setValue(scroll_value)
    
    def on_action_drag_drop(self):
        self.rsap_sequence.move_action_at_index_to_index(old_index=self.drag_source_position,
                                                            new_index=self.currentRow())

    def dragEnterEvent(self, event):
        self.drag_source_position = self.currentRow()  # Capture the source position
        super(ActionSequenceListWidget, self).dragEnterEvent(event)

    def delete_actions_from_sequence(self):
        selected_items = self.selectedItems()
        if not selected_items:
            self.text_output.append("No actions selected to delete!")
            return

        # Get the indexes of the selected items
        indexes_to_delete = [self.row(item) for item in selected_items]

        # Show confirmation dialog
        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            f"Are you sure you want to delete the selected actions?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel
        )
        if reply != QMessageBox.StandardButton.Yes:
            self.text_output.append("Deletion canceled.")
            return

        # Proceed with deletion
        del_success = self.rsap_sequence.delete_actions_at_indexes(indexes_to_delete)

        if del_success:
            self.text_output.append(f"Actions deleted!")
            self.populate_list()
        else:
            self.text_output.append(f"Error trying to delete actions: {', '.join(item.text() for item in selected_items)}")
            return
        

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
