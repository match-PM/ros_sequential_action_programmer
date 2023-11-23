import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QMenu,  QPushButton, QVBoxLayout, QWidget
from PyQt6.QtGui import QAction
from functools import partial
from copy import copy

class ActionSelectionMenu():
    def __init__(self, mainwindow:QMainWindow):
        self.menu_dictionary = {}
        self.mainwindow = mainwindow

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
        #self.menu_dictionary = nested_dict
        

    def addActionMenu(self, menu:QMenu, menu_dict, parents = None):
        if not parents:
            parents = []
        for menu_title, menu_content in menu_dict.items():
            if isinstance(menu_content, dict):  # Submenu
                submenu = menu.addMenu(menu_title)
                parents.append(menu_title)
                self.addActionMenu(submenu, menu_content, parents)
                parents.pop()
            elif isinstance(menu_content, list):  # List of options
                submenu = menu.addMenu(menu_title)
                for option in menu_content:
                    action = QAction(option, self.mainwindow)
                    local = copy(parents)
                    local = parents +[menu_title, option]
                    action.triggered.connect(partial(self.action_menu_clb, local))
                    submenu.addAction(action)
                #parents.clear()
            else:  # Action
                action = QAction(menu_title, self.mainwindow)
                action.triggered.connect(menu_content)
                menu.addAction(action)

    def showMenu(self):
        # Display the menu at the cursor position
        self.contextMenu.exec(self.mainwindow.mapToGlobal(self.mainwindow.sender().pos()))

    def action_menu_clb(self, tree_list):
        print(tree_list)
        print("Overwrite this function for your own use!")

    def init_action_menu(self):
        self.contextMenu = QMenu(self.mainwindow)
        self.addActionMenu(self.contextMenu, self.menu_dictionary)

    def add_dict_to_menu(self, new_entry:dict) -> None:
        self.menu_dictionary = self.append_dict(self.menu_dictionary, new_entry)
        #self.menu_dictionary.update(new_entry)

    def append_dict(self, dict1, dict2):
        for key, value in dict2.items():
            if key not in dict1:
                dict1[key] = value
            else:
                # Append values to existing key (assuming both values are lists)
                if isinstance(dict1[key], list) and isinstance(value, list):
                    dict1[key].extend(value)
                else:
                    # Handle other types or raise an exception if needed
                    pass
        return dict1
    

if __name__ == '__main__':
    pass