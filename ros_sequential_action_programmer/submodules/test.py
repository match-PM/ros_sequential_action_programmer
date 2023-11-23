import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QMenu,  QPushButton, QVBoxLayout, QWidget
from PyQt6.QtGui import QAction
from functools import partial
from copy import copy

class NestedDictMenu(QMainWindow):
    def __init__(self):
        super(NestedDictMenu, self).__init__()

        self.initUI()

    def initUI(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Sample nested dictionary with three levels of depth and a list at the last level
        nested_dict = {
            'Servicesa': {
                'Services_plain':  ['Option1', 'Option2', 'Option3']
            },
            'Servicesb': {
                'Services_plain': {
                    'Options': ['Option4', 'Option5', 'Option6']
                }
            },
            'Servicesc': {
                'Services_plain': {
                    'Options': ['Option7', 'Option8', 'Option9']
                }
            }
        }

        self.createMenu(central_widget, nested_dict)

        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('Nested Dict Menu')
        self.show()

    def createMenu(self, central_widget, nested_dict):
        vbox = QVBoxLayout(central_widget)

        button = QPushButton('Show Menu', self)
        button.clicked.connect(self.showMenu)
        vbox.addWidget(button)

        self.contextMenu = QMenu(self)
        self.addActionMenu(self.contextMenu, nested_dict)

    def addActionMenu(self, menu, menu_dict, parents = None):
        if not parents:
            parents = []
        for menu_title, menu_content in menu_dict.items():
            if isinstance(menu_content, dict):  # Submenu
                submenu = menu.addMenu(menu_title)
                parents.append(menu_title)
                self.addActionMenu(submenu, menu_content, parents)
            elif isinstance(menu_content, list):  # List of options
                submenu = menu.addMenu(menu_title)
                for option in menu_content:
                    action = QAction(option, self)
                    local = copy(parents)
                    local.extend([menu_title, option])
                    action.triggered.connect(partial(self.optionSelected, local))
                    submenu.addAction(action)
                parents.clear()
            else:  # Action
                action = QAction(menu_title, self)
                action.triggered.connect(menu_content)
                menu.addAction(action)

    def showMenu(self):
        # Display the menu at the cursor position
        self.contextMenu.exec(self.mapToGlobal(self.sender().pos()))

    def optionSelected(self, tree_list):
        print(tree_list)

class ActionSelectionMenu():
    def __init__(self):
        self.menu_dictionary = {}

        # Sample nested dictionary with three levels of depth and a list at the last level
        nested_dict = {
            'Servicesa': {
                'Services_plain':  ['Option1', 'Option2', 'Option3']
            },
            'Servicesb': {
                'Services_plain': {
                    'Options': ['Option4', 'Option5', 'Option6']
                }
            },
            'Servicesc': {
                'Services_plain': {
                    'Options': ['Option7', 'Option8', 'Option9']
                }
            }
        }
        self.menu_dictionary = nested_dict
        
        self.open_action_menu_button = QPushButton('Add Action')
        self.open_action_menu_button.clicked.connect(self.showMenu)

        self.contextMenu = QMenu(self)
        self.addActionMenu(self.contextMenu, self.menu_dictionary)

    def addActionMenu(self, menu, menu_dict, parents = None):
        if not parents:
            parents = []
        for menu_title, menu_content in menu_dict.items():
            if isinstance(menu_content, dict):  # Submenu
                submenu = menu.addMenu(menu_title)
                parents.append(menu_title)
                self.addActionMenu(submenu, menu_content, parents)
            elif isinstance(menu_content, list):  # List of options
                submenu = menu.addMenu(menu_title)
                for option in menu_content:
                    action = QAction(option, self)
                    local = copy(parents)
                    local.extend([menu_title, option])
                    action.triggered.connect(partial(self.optionSelected, local))
                    submenu.addAction(action)
                parents.clear()
            else:  # Action
                action = QAction(menu_title, self)
                action.triggered.connect(menu_content)
                menu.addAction(action)

    def showMenu(self):
        # Display the menu at the cursor position
        self.contextMenu.exec(self.mapToGlobal(self.sender().pos()))

    def optionSelected(self, tree_list):
        print(tree_list)

    def add_dict_to_menu(self, new_entry:dict) -> None:
        self.menu_dictionary.update(new_entry)

def main():
    app = QApplication(sys.argv)
    ex = NestedDictMenu()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
