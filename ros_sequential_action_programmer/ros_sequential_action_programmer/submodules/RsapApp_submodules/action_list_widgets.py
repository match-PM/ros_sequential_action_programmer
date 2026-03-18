from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QListWidget, QListWidgetItem
from PyQt6 import QtCore


class ActionListWidget(QListWidget):
    CustDragSig = QtCore.pyqtSignal()
    def __init__(self):
        super(ActionListWidget,self).__init__()
        self.setAcceptDrops(True)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QListWidget.DragDropMode.InternalMove)
        self.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)

    def get_widget_list_names(self):
        name_list=[]
        for index in range(self.count()):
            item = self.item(index)
            if item:
                name_list.append(item.text())
        return name_list
    
    def dropEvent(self, event):
        super(ActionListWidget, self).dropEvent(event)
        event.accept()
        self.CustDragSig.emit()

    def dragEnterEvent(self, event):
        self.drag_source_position = self.currentRow()  # Capture the source position
        super(ActionListWidget, self).dragEnterEvent(event)

class ActionListItem(QListWidgetItem):
    def __init__(self, function_name):
        super().__init__(function_name)
        self.setFlags(self.flags() | Qt.ItemFlag.ItemIsUserCheckable)
        font = self.font()
        font.setPointSize(14)
        self.setFont(font)