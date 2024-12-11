import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QFormLayout, QHBoxLayout, QCheckBox, QComboBox
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from functools import partial
from typing import Union
from rclpy.node import Node

class ParallelGripperConfig():
    def __init__(self, config_key, config_value) -> None:
        self.config_key = config_key
        self.config_value = config_value
        self.current_tool: str = self.config_value['use_tool']
        self.current_tool_jaw: str = self.config_value['use_jaw_type']
        self.tool_active: bool = self.config_value['use_paralell_gripper']

        self.available_tools = []
        _available_tools = self.config_value['available_tools']
        for tool in _available_tools:
            _available_tool_jaws = []
            
            for jaw in tool['availabe_jaws']:
                _available_tool_jaws.append(jaw)
            self.available_tools.append((tool['tool_name'], _available_tool_jaws)) 

    def set_current_tool(self, tool:str, jaw:str):
        self.current_tool = tool
        self.current_tool_jaw = jaw
        self.config_value['use_tool'] = tool
        self.config_value['use_jaw_type'] = jaw
    
    def activate(self):
        self.tool_active = True
        self.config_value['use_paralell_gripper'] = True

    def deactivate(self):
        self.tool_active = False
        self.config_value['use_paralell_gripper'] = False

    def get_activate_status(self):
        return self.tool_active
    
    def get_config(self):
        return self.config_value
    
    def get_first_extension_for_current_tool(self)->str:
        return self.available_tools[0][1][0]

    def get_current_extension_list(self)->list[str]:
        for tool in self.available_tools:
            if tool[0] == self.current_tool:
                return tool[1]
        return []
    
class DispenserTipConfig():
    def __init__(self, config_key: str, config_value:dict) -> None:
        self.config_key = config_key
        self.config_value = config_value
        self.current_dispenser_tip = config_value['use_dispenser_tip']
        self.available_dispenser_tips:list[str] = config_value['availabe_dispenser_tips']
    
    def set_currrent_dispenser_tip(self, tip:str):
        self.current_dispenser_tip = tip
        self.config_value['use_dispenser_tip'] = tip

    def get_config(self):
        return self.config_value
    
class GonioConfig():
    def __init__(self, config_key: str, config_2_key:str, config_value:dict) -> None:
        self.config_key = config_key
        self.config_value = config_value
        self.config_2_key = config_2_key
        self.gonio_active = config_value[config_2_key]
        self.current_chuck = config_value['use_chuck']
        self.available_chucks:list[str] = config_value['availabe_chucks']
    
    def set_current_chuck(self, chuck:str):
        self.current_chuck = chuck
        self.config_value['use_chuck'] = chuck

    def activate(self):
        self.gonio_active = True
        self.config_value[self.config_2_key] = True

    def deactivate(self):
        self.gonio_active = False
        self.config_value[self.config_2_key] = False

    def get_config(self):
        return self.config_value
    
class VacuumGripperConfig():
    def __init__(self, config_key, config_value) -> None:
        self.config_key = config_key
        self.config_value = config_value
        self.current_tool = self.config_value['use_tool']
        self.current_tool_tip = self.config_value['use_tip']
        self.tool_active = self.config_value['use_vacuum_tool']

        self.available_tools = []
        _available_tools = self.config_value['availabe_tools']
        for tool in _available_tools:
            _available_tool_tips = []
            for jaw in tool['availabe_tips']:
                _available_tool_tips.append(jaw)
            self.available_tools.append((tool['tool_name'], _available_tool_tips)) 

    def set_current_tool(self, tool:str = None, tip:str = None):
        if tool is not None:
            self.current_tool = tool
            self.config_value['use_tool'] = tool
        if tip is not None:
            self.current_tool_tip = tip
            self.config_value['use_tip'] = tip
    
    def activate(self):
        self.tool_active = True
        self.config_value['use_vacuum_tool'] = True

    def deactivate(self):
        self.tool_active = False
        self.config_value['use_vacuum_tool'] = False

    def get_activate_status(self):
        return self.tool_active
    def get_config(self):
        return self.config_value
    
    def get_first_extension_for_current_tool(self)->str:
        return self.available_tools[0][1][0]
    
    def get_current_extension_list(self)->list[str]:
        for tool in self.available_tools:
            if tool[0] == self.current_tool:
                return tool[1]
        return []
    
#class ConfigApp():
class PmRobotConfigWidget(QWidget):
    def __init__(self, ros_node:Node = None):
        super().__init__()

        try:
            self.path = get_package_share_directory('pm_robot_bringup')
            self.init_success = True
        except PackageNotFoundError:
            self.init_success = False
            return

        if ros_node is not None:
            self.logger = ros_node.get_logger()
        else:
            self.logger = None

        self.file = f'{self.path}/config/pm_robot_bringup_config.yaml'

        with open(self.file, 'r') as file:
            self.config_data = yaml.safe_load(file)

        self.gripper_vacuum = VacuumGripperConfig('pm_robot_vacuum_tools', self.config_data['pm_robot_tools']['pm_robot_vacuum_tools'])
        self.gripper_1_jaw = ParallelGripperConfig('pm_robot_tool_parallel_gripper_1_jaw', self.config_data['pm_robot_tools']['pm_robot_tool_parallel_gripper_1_jaw'])
        self.gripper_2_jaw = ParallelGripperConfig('pm_robot_tool_parallel_gripper_2_jaws', self.config_data['pm_robot_tools']['pm_robot_tool_parallel_gripper_2_jaws'])
        self.dispenser_tip = DispenserTipConfig('pm_robot_1K_dispenser_tip', self.config_data['pm_robot_1K_dispenser_tip'])
        self.gonio_left = GonioConfig('pm_robot_gonio_left', 'with_Gonio_Left', self.config_data['pm_robot_gonio_left'])
        self.gonio_right = GonioConfig('pm_robot_gonio_right', 'with_Gonio_Right', self.config_data['pm_robot_gonio_right'])
        self.smarpod = GonioConfig('pm_smparpod_station', 'with_smarpod_station', self.config_data['pm_smparpod_station'])

        self.set_config()
        self.init_ui()

    def init_ui(self):
        central_widget = QWidget(self)

        vertical_layout = QVBoxLayout()

        horizontal_layout_vacuum = QHBoxLayout()
        horizontal_layout_gripper_1_jaw = QHBoxLayout()
        horizontal_layout_gripper_2_jaw = QHBoxLayout()
        horizontal_layout_gonio_left = QHBoxLayout()
        horizontal_layout_gonio_right = QHBoxLayout()
        horizontal_layout_smarpod = QHBoxLayout()

        # vacuum gripper checkbox
        self.box_activate_vacuum = QCheckBox()
        self.box_activate_vacuum.setChecked(self.gripper_vacuum.tool_active)
        self.box_activate_vacuum.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_vacuum, self.gripper_vacuum))

        # parallel gripper 1 jaw checkbox
        self.box_activate_gripper_1_jaw = QCheckBox()
        self.box_activate_gripper_1_jaw.setChecked(self.gripper_1_jaw.tool_active)
        self.box_activate_gripper_1_jaw.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_gripper_1_jaw, self.gripper_1_jaw))

        # parallel gripper 2 jaw checkbox
        self.box_activate_gripper_2_jaw = QCheckBox()
        self.box_activate_gripper_2_jaw.setChecked(self.gripper_2_jaw.tool_active)
        self.box_activate_gripper_2_jaw.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_gripper_2_jaw, self.gripper_2_jaw))

        # gonio left checkbox
        self.box_activate_gonio_left = QCheckBox()
        self.box_activate_gonio_left.setChecked(self.gonio_left.gonio_active)
        self.box_activate_gonio_left.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_gonio_left, self.gonio_left))

        # gonio right checkbox  
        self.box_activate_gonio_right = QCheckBox()
        self.box_activate_gonio_right.setChecked(self.gonio_right.gonio_active)
        self.box_activate_gonio_right.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_gonio_right, self.gonio_right))

       # gonio right checkbox  
        self.box_activate_smarpod = QCheckBox()
        self.box_activate_smarpod.setChecked(self.smarpod.gonio_active)
        self.box_activate_smarpod.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_smarpod, self.smarpod))

        # vacuum gripper combobox
        self.vacuum_tool_combobox = QComboBox()
        self.vacuum_tool_combobox.addItems([t[0] for t in self.gripper_vacuum.available_tools])
        self.vacuum_tool_combobox.setCurrentText(self.gripper_vacuum.current_tool)

        self.vacuum_tip_combobox = QComboBox()
        self.vacuum_tip_combobox.addItems(self.gripper_vacuum.get_current_extension_list())
        self.vacuum_tip_combobox.setCurrentText(self.gripper_vacuum.current_tool_tip)

        self.vacuum_tool_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.vacuum_tool_combobox, self.vacuum_tip_combobox, self.gripper_vacuum))
        self.vacuum_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.vacuum_tip_combobox, self.gripper_vacuum))

        # parallel gripper 1 jaw combobox
        self.gripper_1_jaw_combobox = QComboBox()
        self.gripper_1_jaw_combobox.addItems([t[0] for t in self.gripper_1_jaw.available_tools])
        self.gripper_1_jaw_combobox.setCurrentText(self.gripper_1_jaw.current_tool)

        self.gripper_1_jaw_tip_combobox = QComboBox()
        self.gripper_1_jaw_tip_combobox.addItems(self.gripper_1_jaw.get_current_extension_list())
        self.gripper_1_jaw_tip_combobox.setCurrentText(self.gripper_1_jaw.current_tool_jaw)

        self.gripper_1_jaw_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.gripper_1_jaw_combobox, self.gripper_1_jaw_tip_combobox, self.gripper_1_jaw))
        self.gripper_1_jaw_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.gripper_1_jaw_tip_combobox, self.gripper_1_jaw))

        # parallel gripper 2 jaw combobox
        self.gripper_2_jaw_combobox = QComboBox()
        self.gripper_2_jaw_combobox.addItems([t[0] for t in self.gripper_2_jaw.available_tools])
        self.gripper_2_jaw_combobox.setCurrentText(self.gripper_2_jaw.current_tool)

        self.gripper_2_jaw_tip_combobox = QComboBox()
        self.gripper_2_jaw_tip_combobox.addItems(self.gripper_2_jaw.get_current_extension_list())
        self.gripper_2_jaw_tip_combobox.setCurrentText(self.gripper_2_jaw.current_tool_jaw)

        self.gripper_2_jaw_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.gripper_2_jaw_combobox, self.gripper_2_jaw_tip_combobox, self.gripper_2_jaw))
        self.gripper_2_jaw_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.gripper_2_jaw_tip_combobox, self.gripper_2_jaw))

        # dispenser tip combobox
        self.dispenser_tip_combobox = QComboBox()
        self.dispenser_tip_combobox.addItems(self.dispenser_tip.available_dispenser_tips)
        self.dispenser_tip_combobox.setCurrentText(self.dispenser_tip.current_dispenser_tip)
        self.dispenser_tip_combobox.currentTextChanged.connect(partial(self.clb_set_dispenser_tip, self.dispenser_tip_combobox))

        # gonio left combobox
        self.gonio_left_combobox = QComboBox()
        self.gonio_left_combobox.addItems(self.gonio_left.available_chucks)
        self.gonio_left_combobox.setCurrentText(self.gonio_left.current_chuck)
        self.gonio_left_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, self.gonio_left_combobox, self.gonio_left))

        # gonio right combobox
        self.gonio_right_combobox = QComboBox()
        self.gonio_right_combobox.addItems(self.gonio_right.available_chucks)
        self.gonio_right_combobox.setCurrentText(self.gonio_right.current_chuck)
        self.gonio_right_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, self.gonio_right_combobox, self.gonio_right))

        # smarpod combobox
        self.smarpod_combobox = QComboBox()
        self.smarpod_combobox.addItems(self.smarpod.available_chucks)
        self.smarpod_combobox.setCurrentText(self.smarpod.current_chuck)
        self.smarpod_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, self.smarpod_combobox, self.smarpod))

        # layout for vacuum gripper
        vertical_layout.addWidget(QLabel('Vacuum Gripper Settings'))
        horizontal_layout_vacuum.addWidget(self.box_activate_vacuum)
        horizontal_layout_vacuum.addWidget(self.vacuum_tool_combobox)
        horizontal_layout_vacuum.addWidget(self.vacuum_tip_combobox)
        vertical_layout.addLayout(horizontal_layout_vacuum)

        # layout for parallel gripper 1 jaw
        vertical_layout.addWidget(QLabel('Gripper 1 Jaw Settings'))
        horizontal_layout_gripper_1_jaw.addWidget(self.box_activate_gripper_1_jaw)
        horizontal_layout_gripper_1_jaw.addWidget(self.gripper_1_jaw_combobox)
        horizontal_layout_gripper_1_jaw.addWidget(self.gripper_1_jaw_tip_combobox)
        vertical_layout.addLayout(horizontal_layout_gripper_1_jaw)

        # layout for parallel gripper 2 jaw
        vertical_layout.addWidget(QLabel('Gripper 2 Jaw Settings'))
        horizontal_layout_gripper_2_jaw.addWidget(self.box_activate_gripper_2_jaw)
        horizontal_layout_gripper_2_jaw.addWidget(self.gripper_2_jaw_combobox)
        horizontal_layout_gripper_2_jaw.addWidget(self.gripper_2_jaw_tip_combobox)
        vertical_layout.addLayout(horizontal_layout_gripper_2_jaw)

        # layout for dispenser tip
        vertical_layout.addWidget(QLabel('Dispenser Tip Settings'))
        vertical_layout.addWidget(self.dispenser_tip_combobox)

        # layout for gonio left
        vertical_layout.addWidget(QLabel('Gonio Left Settings'))
        horizontal_layout_gonio_left.addWidget(self.box_activate_gonio_left)
        horizontal_layout_gonio_left.addWidget(self.gonio_left_combobox)
        vertical_layout.addLayout(horizontal_layout_gonio_left)

        #layout for gonio right
        vertical_layout.addWidget(QLabel('Gonio Right Settings'))
        horizontal_layout_gonio_right.addWidget(self.box_activate_gonio_right)
        horizontal_layout_gonio_right.addWidget(self.gonio_right_combobox)

        vertical_layout.addLayout(horizontal_layout_gonio_right)

        #layout for smarpod
        vertical_layout.addWidget(QLabel('Smarpod Settings'))
        horizontal_layout_smarpod.addWidget(self.box_activate_smarpod)
        horizontal_layout_smarpod.addWidget(self.smarpod_combobox)

        vertical_layout.addLayout(horizontal_layout_smarpod)        

        # add save button
        save_button = QPushButton("Save Config")
        save_button.clicked.connect(self.save_config)
        vertical_layout.addWidget(save_button)

        central_widget.setLayout(vertical_layout)
        
        self.setWindowTitle('PM Robot Configurator')
        self.setGeometry(100, 100, 600, 400)

    def clb_gripper_checkbox_change(self, box_widget:QCheckBox, obj:Union[VacuumGripperConfig, ParallelGripperConfig]):
        if not box_widget.isChecked():
            box_widget.setChecked(True)
            return
        
        self.deactivate_grippers()
        obj.activate()
        self.refresh_gripper_checkboxes()
        self.set_config()

    def deactivate_grippers(self):
        self.gripper_vacuum.deactivate()
        self.gripper_1_jaw.deactivate()
        self.gripper_2_jaw.deactivate()

    def clb_gonio_checkbox_change(self, box_widget:QCheckBox, gonio:GonioConfig):
        if not box_widget.isChecked():
            gonio.deactivate()
        else:
            gonio.activate()
        print(gonio.gonio_active)
        self.set_config()

    def clb_tool_gripper_combobox_change(self, combobox:QComboBox, combobox_tip: QComboBox ,obj:Union[VacuumGripperConfig, ParallelGripperConfig]):
        obj.set_current_tool(tool=combobox.currentText())
        obj.set_current_tool(tip=obj.get_first_extension_for_current_tool())
        combobox_tip.clear()
        combobox_tip.addItems(obj.get_current_extension_list())
        combobox_tip.setCurrentText(obj.get_first_extension_for_current_tool())
        self.set_config()

    def clb_tip_gripper_combobox_change(self, combobox_tip: QComboBox ,obj:Union[VacuumGripperConfig, ParallelGripperConfig]):
        obj.set_current_tool(tip=combobox_tip.currentText())
        self.set_config()

    def clb_gonio_combobox_change(self, combobox:QComboBox, gonio:GonioConfig):
        gonio.set_current_chuck(combobox.currentText())
        self.set_config()

    def clb_set_dispenser_tip(self, combobox: QComboBox):
        self.dispenser_tip.set_currrent_dispenser_tip(combobox.currentText())
        self.set_config()
    
    def refresh_gripper_checkboxes(self):
        self.box_activate_vacuum.setChecked(self.gripper_vacuum.tool_active)
        self.box_activate_gripper_1_jaw.setChecked(self.gripper_1_jaw.tool_active)
        self.box_activate_gripper_2_jaw.setChecked(self.gripper_2_jaw.tool_active)

    def set_config(self):
        self.config_data['pm_robot_tools'][self.gripper_vacuum.config_key]= self.gripper_vacuum.get_config()
        self.config_data['pm_robot_tools'][self.gripper_1_jaw.config_key]= self.gripper_1_jaw.get_config()
        self.config_data['pm_robot_tools'][self.gripper_2_jaw.config_key]= self.gripper_2_jaw.get_config()
        self.config_data[self.dispenser_tip.config_key]= self.dispenser_tip.get_config()
        self.config_data[self.gonio_left.config_key]= self.gonio_left.get_config()
        self.config_data[self.gonio_right.config_key]= self.gonio_right.get_config()
        self.config_data[self.smarpod.config_key]= self.smarpod.get_config()
        print(self.config_data)
        #self.save_config()

    def save_config(self):
        self.set_config()
        with open(self.file, 'w') as file:
            yaml.dump(self.config_data, file, default_flow_style=False)

        if self.logger is not None:
            self.logger.info("Config saved.")


class DummyMain(QMainWindow):

    def __init__(self):
        super().__init__()
        self.w = None  # No external window yet.
        self.button = QPushButton("Push for Window")
        self.button.clicked.connect(self.show_new_window)
        self.setCentralWidget(self.button)

    def show_new_window(self, checked):
        if self.w is None:
            self.w = PmRobotConfigWidget()
        self.w.show()


def main():
    app = QApplication(sys.argv)
    ex = DummyMain()
    ex.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
