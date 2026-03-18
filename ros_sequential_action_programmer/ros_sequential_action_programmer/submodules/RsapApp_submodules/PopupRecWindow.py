from PyQt6.QtWidgets import QApplication, QDialog, QVBoxLayout, QComboBox, QListWidget, QPushButton, QHBoxLayout

class PopupRecWindow(QDialog):
    def __init__(self, recommendation_list):
        super().__init__()

        self.recommendation_list = recommendation_list

        # Create combo box to display keys
        self.key_combobox = QComboBox()
        self.key_combobox.addItems([list(item.keys())[0] for item in self.recommendation_list])
        self.key_combobox.currentIndexChanged.connect(self.show_list)

        # Create list widget to display items
        self.list_widget = QListWidget()

        # Create Choose and Cancel buttons
        choose_button = QPushButton("Choose")
        choose_button.clicked.connect(self.accept)

        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)

        # Layout setup
        layout = QVBoxLayout(self)
        button_layout = QHBoxLayout()
        button_layout.addWidget(cancel_button)
        button_layout.addWidget(choose_button)
        layout.addWidget(self.key_combobox)
        layout.addWidget(self.list_widget)
        layout.addLayout(button_layout)
        self.show_list()
        self.resize(800, 500)

    def show_list(self):
        # Get the selected key
        selected_key = self.key_combobox.currentText()
        selected_item = next((item for item in self.recommendation_list if selected_key in item), {})
        selected_list = selected_item.get(selected_key, [])
        self.list_widget.clear()
        self.list_widget.addItems(selected_list)
    
    def accept(self):
        # Get the selected item from the QListWidget
        selected_items = self.list_widget.selectedItems()
        if selected_items:
            self.selected_value = selected_items[0].text()
            super().accept()
        else:
            # Close the dialog
            super().close()

    def get_selected_value(self):
        return self.selected_value

if __name__ == "__main__":
    app = QApplication([])

    # Sample data
    data = [{'Vision-Processes': ['test_13.json', 'my_test.json', 'my_first_app_process.json', 'process_demo.json', 'process_demo_2.json', 'simple_process_example.json', 'test/test_13.json']}, {'Vision-Cameras': ['webcam_config.yaml']}, {'service_response': ['service_response.1-/get_planning_scene.scene.name', 'service_response.1-/get_planning_scene.scene.robot_state.joint_state.header.frame_id', 'service_response.1-/get_planning_scene.scene.robot_state.multi_dof_joint_state.header.frame_id', 'service_response.1-/get_planning_scene.scene.robot_model_name', 'service_response.1-/get_planning_scene.scene.world.octomap.header.frame_id', 'service_response.1-/get_planning_scene.scene.world.octomap.octomap.header.frame_id', 'service_response.1-/get_planning_scene.scene.world.octomap.octomap.id']}]
    print(type(data[1]))
    popup = PopupRecWindow(data)
    popup.exec()
