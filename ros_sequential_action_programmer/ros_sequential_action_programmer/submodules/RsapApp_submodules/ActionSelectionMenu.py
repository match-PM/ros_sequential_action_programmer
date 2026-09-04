import sys
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QDialog,
    QDialogButtonBox,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
)
from PyQt6.QtGui import QAction
from functools import partial
from copy import copy
from PyQt6.QtGui import QCursor

from ros_sequential_action_programmer.submodules.saving_loading_functions import RosSequentialActionProgrammer
from rosidl_runtime_py.get_interfaces import get_service_interfaces


class SearchableSelectionDialog(QDialog):
    """Scrollable tree picker for large nested action/service collections."""

    PATH_ROLE = Qt.ItemDataRole.UserRole
    SEARCH_ROLE = int(Qt.ItemDataRole.UserRole) + 1

    def __init__(self, menu_dictionary, parent=None):
        super().__init__(parent)
        self.selected_path = None
        self.setWindowTitle("Add Action")
        screen = parent.screen() if parent is not None else QApplication.primaryScreen()
        if screen is not None:
            available_size = screen.availableGeometry().size()
            self.resize(
                min(750, int(available_size.width() * 0.85)),
                min(700, int(available_size.height() * 0.85)),
            )
        else:
            self.resize(750, 700)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Select a service or action:"))

        self.search_input = QLineEdit(self)
        self.search_input.setPlaceholderText("Search by name, type, or namespace...")
        self.search_input.setClearButtonEnabled(True)
        layout.addWidget(self.search_input)

        self.tree = QTreeWidget(self)
        self.tree.setHeaderHidden(True)
        self.tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.tree.setUniformRowHeights(True)
        layout.addWidget(self.tree)

        self.buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel,
            parent=self,
        )
        self.buttons.button(QDialogButtonBox.StandardButton.Ok).setText("Add")
        self.buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        layout.addWidget(self.buttons)

        self._populate_tree(menu_dictionary)
        self.search_input.textChanged.connect(self._filter_tree)
        self.tree.currentItemChanged.connect(self._update_add_button)
        self.tree.itemDoubleClicked.connect(self._accept_item)
        self.buttons.button(QDialogButtonBox.StandardButton.Ok).clicked.connect(
            self._accept_current_item
        )
        self.buttons.rejected.connect(self.reject)
        self.search_input.setFocus()

    def _populate_tree(self, menu_dictionary):
        self._add_dictionary_items(self.tree.invisibleRootItem(), menu_dictionary, [])

    def _add_dictionary_items(self, parent_item, menu_dictionary, parents):
        for title, content in menu_dictionary.items():
            category_item = QTreeWidgetItem(parent_item, [str(title)])
            category_path = parents + [str(title)]
            category_item.setData(
                0, self.SEARCH_ROLE, " ".join(category_path).lower()
            )

            if isinstance(content, dict):
                self._add_dictionary_items(category_item, content, category_path)
            elif isinstance(content, list):
                for option in content:
                    option_path = category_path + [str(option)]
                    option_item = QTreeWidgetItem(category_item, [str(option)])
                    option_item.setData(0, self.PATH_ROLE, option_path)
                    option_item.setData(
                        0, self.SEARCH_ROLE, " ".join(option_path).lower()
                    )

    def _filter_tree(self, text):
        search_text = text.strip().lower()

        def update_visibility(item, ancestor_matches=False):
            item_matches = search_text in item.data(0, self.SEARCH_ROLE)
            show_descendants = ancestor_matches or item_matches
            child_visible = False
            for index in range(item.childCount()):
                child_visible |= update_visibility(
                    item.child(index), show_descendants
                )

            visible = not search_text or show_descendants or child_visible
            item.setHidden(not visible)
            if search_text and child_visible:
                item.setExpanded(True)
            return visible

        root = self.tree.invisibleRootItem()
        for index in range(root.childCount()):
            update_visibility(root.child(index))

    def _update_add_button(self, current, previous=None):
        can_add = current is not None and current.data(0, self.PATH_ROLE) is not None
        self.buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(can_add)

    def _accept_item(self, item, column=0):
        path = item.data(0, self.PATH_ROLE)
        if path is not None:
            self.selected_path = path
            self.accept()

    def _accept_current_item(self):
        current_item = self.tree.currentItem()
        if current_item is not None:
            self._accept_item(current_item)


class SelectionMenu():
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

    def showMenu(self, use_button_pos=False):
        """
        Display the context menu.
        - If use_button_pos=True, show at the button's position.
        - Otherwise, show at the current mouse cursor.
        """
        if use_button_pos and self.mainwindow.sender():
            # Use the widget's position that triggered the menu
            self.contextMenu.exec(self.mainwindow.mapToGlobal(self.mainwindow.sender().pos()))
        else:
            # Show at the current cursor position
            self.contextMenu.exec(QCursor.pos())

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

class ActionSelectionMenu(SelectionMenu):
    def __init__(self, mainwindow, rsap:RosSequentialActionProgrammer):
        super().__init__(mainwindow)
        self.rsap = rsap
    
    def show_action_menu(self):
        self.rsap.initialize_service_list()
        self.rsap.initialize_ros_action_list()

        self.rsap.save_all_service_req_res_to_JSON()
        self.menu_dictionary= {
            'Services': {
                'Empty':  ['New'],
                'Active Clients blk':  self.rsap.list_of_clients_to_dict(self.rsap.get_active_client_blklist()),
                'Active Clients':  self.rsap.list_of_clients_to_dict(self.rsap.list_of_active_clients),
                'Active Clients wht':  self.rsap.list_of_clients_to_dict(self.rsap.get_active_client_whtlist()),
                'Memorised Clients blk':  self.rsap.list_of_clients_to_dict(self.rsap.get_memorized_client_blklist()),
                'Memorised Clients ':  self.rsap.list_of_clients_to_dict(self.rsap.get_list_memorized_service_clients()),
                'Memorised Clients wht':  self.rsap.list_of_clients_to_dict(self.rsap.get_memorized_client_whitelist()),
                'Available Service Types':  get_service_interfaces(),
            },
            'Actions':{
                'Empty':  ['New'],
                'Active Clients blk':  ['New'],
                'Active Clients':  self.rsap.list_of_clients_to_dict(self.rsap.list_of_active_ros_action_clients),
                'Active Clients wht':  ['New'],
                'Memorised Clients blk':  ['New'],
                'Memorised Clients ':  ['New'],
                'Memorised Clients wht':  ['New'],
                'Available Service Types':  ['New'],
            },
            'Skills': ['TBD1','TBD2','TBD3'],
            'Other': {
                'Conditions': {
                    'Options': ['Option7', 'Option8', 'Option9']
                },
                'Operation': ['User Interaction']
            }
        }
        dialog = SearchableSelectionDialog(self.menu_dictionary, self.mainwindow)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.action_menu_clb(dialog.selected_path)
    

if __name__ == '__main__':
    pass
