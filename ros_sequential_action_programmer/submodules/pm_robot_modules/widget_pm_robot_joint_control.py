from PyQt6.QtWidgets import QApplication, QGroupBox, QGridLayout, QMenu, QMainWindow, QTableWidget, QTableWidgetItem, QWidget, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QPushButton, QFormLayout, QHBoxLayout, QCheckBox, QComboBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QAction, QDoubleValidator
from ament_index_python.packages import get_package_share_directory
from functools import partial, reduce
from rclpy.node import Node
from copy import deepcopy, copy
from rosidl_runtime_py.convert import message_to_ordereddict
import math

from ros_sequential_action_programmer.submodules.pm_robot_modules.joint_controller_class import JointJogControl


def degrees_to_radians(degrees)->float:
    return degrees * (math.pi / 180.0)

def radians_to_degrees(radians)->float:
    return radians * (180.0 / math.pi)

class PmRobotJointControlWidget(QWidget):
    def __init__(self, ros_node:Node):
        super().__init__()
        self.ros_node = ros_node
        joint_topic = "/joint_states"
        gonio_right_controller_name = "/pm_robot_gonio_right_controller/follow_joint_trajectory"
        joints_gonio_right = ['Gonio_Right_Stage_1_Joint','Gonio_Right_Stage_2_Joint']

        self.gonio_right_controller = JointJogControl(self.ros_node, joints_gonio_right, gonio_right_controller_name, joint_topic, "robot_state_publisher")
        self.gonio_right_controller.set_current_joint_values([0.0, 0.0])

        gonio_left_controller_name = "/pm_robot_gonio_left_controller/follow_joint_trajectory"
        joint_gonio_left = ['Gonio_Left_Stage_1_Joint','Gonio_Left_Stage_2_Joint']

        self.gonio_left_controller = JointJogControl(self.ros_node, joint_gonio_left, gonio_left_controller_name, joint_topic, "robot_state_publisher")


        t_axis_controller_name = "/pm_robot_t_axis_controller/follow_joint_trajectory"
        joints_t = ['T_Axis_Joint']

        self.t_axis_controller = JointJogControl(self.ros_node, joints_t, t_axis_controller_name, joint_topic, "robot_state_publisher")

        xyz_controller_name = "/pm_robot_xyz_axis_controller/follow_joint_trajectory"
        joints_xyz = ['X_Axis_Joint','Y_Axis_Joint','Z_Axis_Joint']

        self.xyz_controller = JointJogControl(self.ros_node, joints_xyz, xyz_controller_name, joint_topic, "robot_state_publisher")

        self.init_ui()  
        self.gonio_right_controller.set_target_from_current()
        self.gonio_left_controller.set_target_from_current()
        self.t_axis_controller.set_target_from_current()
        self.xyz_controller.set_target_from_current()
    
    def init_ui(self):
        central_widget = QWidget(self)
        #self.setCentralWidget(central_widget)
        self.font_label = QFont()  # Create a QFont object
        self.font_label.setPointSize(12)  # Set the font size to 16 points
        self.font_label.setBold(True)
        self.font_label_2 = QFont()  # Create a QFont object
        self.font_label_2.setPointSize(14)  # Set the font size to 16 points
        self.font_label_2.setBold(True)
        main_layout = QGridLayout()

        self.vertical_layout = QVBoxLayout()
        #main_layout.addLayout(self.vertical_layout,2,0,1,2)

        central_widget.setLayout(self.vertical_layout)

        self.add_jog_widgets_for_instance(self.gonio_right_controller, [-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0], "deg")
        self.add_jog_widgets_for_instance(self.gonio_left_controller, [-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0], "deg")
        self.add_jog_widgets_for_instance(self.t_axis_controller, [-10.0,-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0,10.0], "deg")
        self.add_jog_widgets_for_instance(self.xyz_controller, [-10.0,-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0,10.0], "mm")


    def add_jog_widgets_for_instance(self, joint_control_instance:JointJogControl, incement_list:list, unit:str):

        joint_instance_layout = QVBoxLayout()
        controller_label = QLabel(joint_control_instance._action_name)
        controller_label.setFont(self.font_label_2)
        controller_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controller_group_box = QGroupBox()
        controller_group_box.setLayout(joint_instance_layout)
        controller_group_box.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vertical_layout.addWidget(controller_label)
        self.vertical_layout.addWidget(controller_group_box)

        for index, joint in enumerate(joint_control_instance.joint_names):

            joint_jog_layout_v = QHBoxLayout()
            #joint_jog_layout_v.setAlignment(Qt.AlignmentFlag.AlignCenter)

            # Three widget in the middle
            joint_layout = QVBoxLayout()
            joint_label_widget = QLabel(f'{joint} [{unit}]')
            joint_label_widget.setFont(self.font_label)
            joint_label_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
            display_widget_current_value = QLineEdit()
            display_widget_current_value.setReadOnly(True)
            display_widget_current_value.setFixedWidth(100)
            display_widget_current_value.setToolTip(f"Current value of '{joint}'")
            display_widget_current_value.setAlignment(Qt.AlignmentFlag.AlignCenter)

            joint_control_instance.gui_update_values.gui_update_current_values_signal.connect(partial(self.set_value, display_widget_current_value, index, unit))

            if unit == "deg":
                current_value = radians_to_degrees(joint_control_instance.get_current_joint_value_for_joint(joint))
            elif unit == "mm":
                current_value = joint_control_instance.get_current_joint_value_for_joint(joint)*1000
            else:
                current_value = joint_control_instance.get_current_joint_value_for_joint(joint)

            display_widget_current_value.setText(str(round(current_value,6)))
            display_widget_target_value = QLineEdit()
            display_widget_target_value.setFixedWidth(100)
            display_widget_target_value.setToolTip(f"Target value of '{joint}'")
            display_widget_target_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            double_validator = QDoubleValidator()
            display_widget_target_value.setValidator(double_validator)
            display_widget_target_value.textChanged.connect(partial(self.cbl_set_target_joint_valaue_from_text, joint_control_instance, joint, unit))
            joint_control_instance.gui_update_values.gui_update_target_values_signal.connect(partial(self.set_value, display_widget_target_value, index, unit))

            for index, inc_value in enumerate(incement_list):
                    if index == int(len(incement_list)/2):
                        #joint_layout.addWidget(joint_label_widget)
                        joint_layout.addWidget(display_widget_current_value)
                        joint_layout.addWidget(display_widget_target_value)
                        joint_jog_layout_v.addLayout(joint_layout)

                    button = QPushButton(str(inc_value))
                    button.setFixedWidth(60)
                    button.clicked.connect(partial(self.clb_change_joint_target_value, joint_control_instance, joint, inc_value, unit))
                    joint_jog_layout_v.addWidget(button)

            joint_instance_layout.addWidget(joint_label_widget)
            joint_instance_layout.addLayout(joint_jog_layout_v)
        
        send_button = QPushButton("Send Target Values")
        send_button.clicked.connect(joint_control_instance.send_target_joint_values)
        joint_instance_layout.addWidget(send_button)
    
    def cbl_set_target_joint_valaue_from_text(self, joint_control_instance:JointJogControl, joint_name:str, unit:str, changed_value):
        if unit == "deg":
            changed_value = degrees_to_radians(float(changed_value))
        elif unit == "mm":
            changed_value = float(changed_value)/1000
        bool_t = joint_control_instance.set_target_joint_value(joint_name, float(changed_value))

    def clb_change_joint_target_value(self,  joint_control_instance:JointJogControl, joint_name:str, inc_value:float, unit:str):
        current_value = joint_control_instance.get_target_joint_value_for_joint(joint_name)
        self.ros_node.get_logger().debug(f"A: Current value for joint {joint_name} is {current_value}")
        if unit == "deg":
            inc_value = degrees_to_radians(inc_value)
            new_value = current_value + inc_value
        elif unit == "mm":
            inc_value = inc_value/ 1000
            new_value = current_value + inc_value
        else:
            new_value = current_value + inc_value
        joint_control_instance.set_target_joint_value(joint_name, new_value)
        self.ros_node.get_logger().debug(f"B: New target value for joint {joint_name} is {new_value}")
        
    def set_value(self, display_widget:QLineEdit, ind:int, unit:str, values:list[float]):
        if unit == "deg":
            display_widget.setText(str(round(radians_to_degrees(values[ind]),6)))
        elif unit == "mm":
            display_widget.setText(str(round(values[ind]*1000,6)))
        else:
            display_widget.setText(str(round(values[ind],6)))