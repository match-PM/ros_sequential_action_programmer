import sys
from PyQt6.QtWidgets import QApplication, QGridLayout, QMainWindow, QTableWidget, QTableWidgetItem, QWidget, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QPushButton, QFormLayout, QHBoxLayout, QCheckBox, QComboBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
import yaml
from ament_index_python.packages import get_package_share_directory
from functools import partial, reduce
from typing import Union
from geometry_msgs.msg import Pose
from operator import attrgetter
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread 
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from copy import deepcopy, copy
import yaml
import tf2_py as tf2
from geometry_msgs.msg import Vector3, Quaternion
from importlib import import_module
from sensor_msgs.msg._joint_state import JointState
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from rosidl_runtime_py.convert import message_to_ordereddict

def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return reduce(_getattr, [obj] + attr.split('.'))


class PmRobotAxisControl():
    
    BLACKLIST = ['1K_Dispenser_Flap', 'Z_Axis', '1K_Dispenser', '2K_Dispenser_Cartridge', 'Camera_Station', 'Camera_Calibration_Platelet', 'Gonio_Left_Stage_1_Top', 'Gonio_Left_Base', 'Gonio_Left_Stage_2_Bottom', 'Gonio_Right_Stage_1_Top', 'Gonio_Right_Stage_1_Bottom', 'Gonio_Right_Stage_2_Bottom', 'Gripper_Rot_Plate', 'UV_LED_Back', 'UV_Slider_X_Back', 'UV_LED_Front', 'UV_Slider_X_Front', 'X_Axis', 'axis_base', 'Y_Axis', '1K_Dispenser_TCP', '1K_Dispenser_Tip', '2K_Dispenser_TCP', 'Cam1_Toolhead_TCP', 'Camera_Bottom_View_Link', 'Camera_Bottom_View_Link_Optical', 'pm_robot_base_link', 'Camera_Top_View_Link', 'Camera_Top_View_Link_Optical', 'Gonio_Base_Right', 'Laser_Toolhead_TCP', 'PM_Robot_Tool_TCP', 'PM_Robot_Vacuum_Tool', 'PM_Robot_Vacuum_Tool_Tip',  'housing_hl', 'base_link_empthy', 'housing_hr', 'housing', 'housing_vl', 'housing_vr', 'laser_top_link', 'left_match_logo_font', 'left_match_logo_background', 'match_logo_link', 't_axis_toolchanger']
    
    def __init__(self, ros_node:Node):
        
        self.PM_Interfaces_Module = import_module('pm_moveit_interfaces.srv')
        self.target_pose = Pose()
        self.tools = ['PM_Robot_Tool_TCP', '1K_Dispenser_TCP', 'Cam1_Toolhead_TCP','Laser_Toolhead_TCP']
        self._active_tool = 'PM_Robot_Tool_TCP'
        self.node = ros_node
        self.clb_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()
        self.current_tool_pose = Pose()
        self.rel_movement_position = Vector3()
        # self.tf_subscription = self.node.create_subscription(TFMessage,
        #                                                     '/tf',
        #                                                     self.tf_callback,
        #                                                     10)
        # self.tf_subscription_2 = self.node.create_subscription(TFMessage,
        #                                         '/tf_static',
        #                                         self.tf_static_callback,
        #                                         10)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)
        self.available_frames = []
        self.update_timer = self.node.create_timer(0.1, self.update_target,callback_group=self.clb_group)
        self.joint_state_topic_watchdog = self.node.create_timer(10, self.check_js_subscription, callback_group=self.clb_group)
        self.frame_added = False    
        self.joint_state_msg_received = False
        # subscribe to joint states
        self.joint_state_subscription = self.node.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10, callback_group=self.clb_group)
        self.joint_state_list = []
        
    def set_active_tool(self, tool:str):
        if tool in self.tools:
            self._active_tool = tool
    
    def get_active_tool(self):
        return self._active_tool
    
    def check_js_subscription(self):
        #self.logger.warn("Checking joint state subscription")
        if not self.joint_state_msg_received:
            self.logger.error("No joint state message received within 10 s. Destroying subscription.")
            self.joint_state_subscription.destroy()
            self.joint_state_topic_watchdog.cancel()
            self.update_timer.cancel()
        
    # def tf_callback(self, msg:TFMessage):
    #     for transform in msg.transforms:
    #         transform:TransformStamped
    #         #print(transform.child_frame_id)
    #         if transform.child_frame_id not in self.available_frames:
    #             pass
    
    # def tf_static_callback(self, msg:TFMessage):
    #     for transform in msg.transforms:
    #         transform:TransformStamped
    #         if transform.child_frame_id not in self.available_frames:
    #             pass
    
    def set_target_pose_from_frame_world_transform(self, frame:str):
        """
        Set target pose from frame to world transform
        Args:
            frame (str): frame name
        """
        try:
            tool_world_pose = self.tf_buffer.lookup_transform("world", frame, rclpy.time.Time())
            self.target_pose.position.x = tool_world_pose.transform.translation.x * 1000
            self.target_pose.position.y = tool_world_pose.transform.translation.y * 1000
            self.target_pose.position.z = tool_world_pose.transform.translation.z * 1000
            self.target_pose.orientation.w = tool_world_pose.transform.rotation.w
            self.target_pose.orientation.x = tool_world_pose.transform.rotation.x
            self.target_pose.orientation.y = tool_world_pose.transform.rotation.y
            self.target_pose.orientation.z = tool_world_pose.transform.rotation.z
        except Exception as e:
            print(e)

    def update_current_tool_pose(self):
        try:
            tool_world_pose = self.tf_buffer.lookup_transform("world", self.get_active_tool(), rclpy.time.Time())
            self.current_tool_pose.position.x = tool_world_pose.transform.translation.x * 1000
            self.current_tool_pose.position.y = tool_world_pose.transform.translation.y * 1000
            self.current_tool_pose.position.z = tool_world_pose.transform.translation.z * 1000
            self.current_tool_pose.orientation.w = tool_world_pose.transform.rotation.w
            self.current_tool_pose.orientation.x = tool_world_pose.transform.rotation.x
            self.current_tool_pose.orientation.y = tool_world_pose.transform.rotation.y
            self.current_tool_pose.orientation.z = tool_world_pose.transform.rotation.z
        except Exception as e:
            print(e)

    def update_target_pose_with_current_pose(self):
        self.target_pose.position.x = self.current_tool_pose.position.x
        self.target_pose.position.y = self.current_tool_pose.position.y
        self.target_pose.position.z = self.current_tool_pose.position.z
        self.target_pose.orientation.w = self.current_tool_pose.orientation.w
        self.target_pose.orientation.x = self.current_tool_pose.orientation.x
        self.target_pose.orientation.y = self.current_tool_pose.orientation.y
        self.target_pose.orientation.z = self.current_tool_pose.orientation.z

    def update_target(self):
        self.update_current_tool_pose()
        self.target_pose.position.x += float(self.rel_movement_position.x)
        self.target_pose.position.y += float(self.rel_movement_position.y)
        self.target_pose.position.z += float(self.rel_movement_position.z)
        self.rel_movement_position.x = 0.0
        self.rel_movement_position.y = 0.0
        self.rel_movement_position.z = 0.0
        self.update_frame_list()

    def update_frame_list(self):
        self.frame_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        # extract frames from dict
        if self.frame_dict == []:
            self._frame_list = []
        else:
            self._frame_list = list(self.frame_dict.keys())

        #print(self._frame_list)
        for frame in self._frame_list:
            if frame not in self.available_frames and frame not in self.BLACKLIST:
                self.available_frames.append(frame)
                self.frame_added = True

    def move_to_target(self):
        # move to target pose
        if self.get_active_tool() == 'PM_Robot_Tool_TCP':
            service_action = ServiceAction(self.node, client='/pm_moveit_server/move_tool_to_pose', service_type='pm_moveit_interfaces/srv/MoveToPose')
            #client = self.node.create_client(self.PM_Interfaces_Module.MoveToolTcpTo, '/pm_moveit_server/move_tool_to_frame', callback_group=self.clb_group)
        elif self.get_active_tool() == 'Cam1_Toolhead_TCP':
            service_action = ServiceAction(self.node, client='/pm_moveit_server/move_cam1_to_pose', service_type='pm_moveit_interfaces/srv/MoveToPose')
            #client = self.node.create_client(self.PM_Interfaces_Module.MoveCam1TcpTo, '/pm_moveit_server/move_cam1_to_frame', callback_group=self.clb_group)
        elif self.get_active_tool() == 'Laser_Toolhead_TCP':
            service_action = ServiceAction(self.node, client='/pm_moveit_server/move_laser_to_pose', service_type='pm_moveit_interfaces/srv/MoveToPose')
            #client = self.node.create_client(self.PM_Interfaces_Module.MoveLaserTcpTo, '/pm_moveit_server/move_laser_to_frame', callback_group=self.clb_group)
        else:
            self.node.get_logger().error(f"Service for moving '{self.get_active_tool()}' not yet implemented!")
            return
        
        request = self.PM_Interfaces_Module.MoveToPose.Request()
        request.move_to_pose.position.x = self.target_pose.position.x / 1000
        request.move_to_pose.position.y = self.target_pose.position.y / 1000
        request.move_to_pose.position.z = self.target_pose.position.z / 1000
        request.execute_movement = True
        request_dict = message_to_ordereddict(request)
        service_action.set_req_message_from_dict(request_dict)
        success = service_action.execute()

        #result = client.call(request)
        
        self.node.get_logger().info(f'Result of {self.get_active_tool()}: {success}')
        return
        
    def joint_state_callback(self, msg:JointState):
        self.joint_state_msg_received = True
        # Create a list of tuples with first the name and second the position
        self.joint_state_list = list(zip(msg.name, msg.position, msg.velocity, msg.effort))











class PmDashboardApp(QWidget):
    def __init__(self, ros_node:Node=None):
        super().__init__()
        self.path = get_package_share_directory('pm_robot_bringup')
        self.ros_node = ros_node
        self.pm_robot_control = PmRobotAxisControl(ros_node)
        self.clb_group = ReentrantCallbackGroup()
        self.init_ui()
        # Set active tool and update target poses
        self.set_active_tool()

        self.timer = self.ros_node.create_timer(0.1, self.cbk_timer_update_widget_current_tool_pose, callback_group=self.clb_group)


    def init_ui(self):
        central_widget = QWidget(self)
        #self.setCentralWidget(central_widget)
        self.font_label = QFont()  # Create a QFont object
        self.font_label.setPointSize(14)  # Set the font size to 16 points
        main_layout = QGridLayout()

        self.vertical_layout = QVBoxLayout()

        main_layout.addLayout(self.vertical_layout,2,0,1,2)

        # active tool combobox
        self.active_tool_combobox = QComboBox()
        active_tool_label = QLabel('Active Tool')
        active_tool_label.setFont(self.font_label)
        main_layout.addWidget(active_tool_label, 0, 0)
        self.active_tool_combobox.addItems(self.pm_robot_control.tools)
        self.active_tool_combobox.setCurrentText(self.pm_robot_control.get_active_tool())
        self.active_tool_combobox.currentTextChanged.connect(self.set_active_tool)
        main_layout.addWidget(self.active_tool_combobox, 1, 0)
        #self.vertical_layout.addWidget(self.active_tool_combobox)        

        # target frame combobox
        self.target_frame_combobox = QComboBox()
        target_frame_label = QLabel('Target Frame')
        target_frame_label.setFont(self.font_label)
        main_layout.addWidget(target_frame_label, 0, 1)
        self.target_frame_combobox.currentTextChanged.connect(self.cbk_target_frame_changed)
        #self.vertical_layout.addWidget(self.target_frame_combobox) 
        main_layout.addWidget(self.target_frame_combobox, 1, 1)

        # target pose jog widgets
        self.create_target_pose_jog_widgets()

        # add automove checkbox
        self.automove_checkbox = QCheckBox('Auto Move')
        self.automove_active = self.automove_checkbox.isChecked
        # Add move button
        self.move_button = QPushButton('Move to Target')
        self.move_button.clicked.connect(self.pm_robot_control.move_to_target)
        self.vertical_layout.addWidget(self.automove_checkbox)
        self.vertical_layout.addWidget(self.move_button)

        # add text output
        self.text_output_terminal = QTextEdit()
        self.text_output_terminal.setReadOnly(True)
        self.vertical_layout.addWidget(self.text_output_terminal)

        self.table_widget = QTableWidget()
        self.table_widget.setMinimumWidth(450)
        self.table_widget.setMinimumHeight(500)
        main_layout.addWidget(self.table_widget,2,2)

        central_widget.setLayout(main_layout)

        self.setWindowTitle('PM Robot Dashboard')
        self.setGeometry(100, 100, 1400, 800)

    def update_target_pose_widget(self):
        self.display_widget_target_pose_position_x.setText(str(round(self.pm_robot_control.target_pose.position.x,6)))
        self.display_widget_target_pose_position_y.setText(str(round(self.pm_robot_control.target_pose.position.y,6)))
        self.display_widget_target_pose_position_z.setText(str(round(self.pm_robot_control.target_pose.position.z,6)))
        self.display_widget_target_pose_orientation_w.setText(str(round(self.pm_robot_control.target_pose.orientation.w,9)))
        self.display_widget_target_pose_orientation_x.setText(str(round(self.pm_robot_control.target_pose.orientation.x,9)))
        self.display_widget_target_pose_orientation_y.setText(str(round(self.pm_robot_control.target_pose.orientation.y,9)))
        self.display_widget_target_pose_orientation_z.setText(str(round(self.pm_robot_control.target_pose.orientation.z,9)))

    def clb_change_target_pose(self, obj: PmRobotAxisControl, path: str, value: float):   
        new_value = value + rgetattr(obj, path)
        rsetattr(obj, path, new_value)
        self.pm_robot_control.update_target()
        self.update_target_pose_widget()

        if self.automove_active():
            self.pm_robot_control.move_to_target()
            print("Auto move active")

    def create_target_pose_jog_widgets(self):
        target_pose_jog_layout = QVBoxLayout()
        translation_list = [-10.0,-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0,10.0]
        for i in range(3):
            target_pose_jog_layout_h = QHBoxLayout()
            target_pose_jog_layout_h.setAlignment(Qt.AlignmentFlag.AlignCenter)
            if i == 0:
                key ='rel_movement_position.x'
                xlabel_widget = QLabel('X-Direction [mm]')
                xlabel_widget.setFont(self.font_label)
                xlabel_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
                target_pose_jog_layout.addWidget(xlabel_widget)
                self.display_widget_current_pose_position_x = QLineEdit()
                self.display_widget_current_pose_position_x.setReadOnly(True)
                self.display_widget_current_pose_position_x.setFixedWidth(100)
                self.display_widget_current_pose_position_x.setToolTip('Current value x-direction')
                target_pose_jog_layout.addWidget(self.display_widget_current_pose_position_x, alignment=Qt.AlignmentFlag.AlignCenter)
            elif i == 1:
                key ='rel_movement_position.y'
                ylabel_widget = QLabel('Y-Direction [mm]')
                ylabel_widget.setFont(self.font_label)
                ylabel_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
                target_pose_jog_layout.addWidget(ylabel_widget)
                self.display_widget_current_pose_position_y = QLineEdit()
                self.display_widget_current_pose_position_y.setReadOnly(True)
                self.display_widget_current_pose_position_y.setFixedWidth(100)
                self.display_widget_current_pose_position_y.setToolTip('Current value y-direction')
                target_pose_jog_layout.addWidget(self.display_widget_current_pose_position_y, alignment=Qt.AlignmentFlag.AlignCenter)
            elif i == 2:
                key ='rel_movement_position.z'
                zlabel_widget = QLabel('Z-Direction [mm]')
                zlabel_widget.setFont(self.font_label)
                zlabel_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
                target_pose_jog_layout.addWidget(zlabel_widget)
                self.display_widget_current_pose_position_z = QLineEdit()
                self.display_widget_current_pose_position_z.setReadOnly(True)
                self.display_widget_current_pose_position_z.setFixedWidth(100)
                self.display_widget_current_pose_position_z.setToolTip('Current value z-direction')
                target_pose_jog_layout.addWidget(self.display_widget_current_pose_position_z, alignment=Qt.AlignmentFlag.AlignCenter)

            for index, element in enumerate(translation_list):
                if index == len(translation_list)/2 and i == 0:
                    self.display_widget_target_pose_position_x = QLineEdit()
                    self.display_widget_target_pose_position_x.setFixedWidth(100)
                    self.display_widget_target_pose_position_x.setReadOnly(True)
                    self.display_widget_target_pose_position_x.setToolTip('Target value x-direction')
                    target_pose_jog_layout_h.addWidget(self.display_widget_target_pose_position_x)
                elif index == len(translation_list)/2 and i == 1:
                    self.display_widget_target_pose_position_y = QLineEdit()
                    self.display_widget_target_pose_position_y.setFixedWidth(100)
                    self.display_widget_target_pose_position_y.setReadOnly(True)
                    self.display_widget_target_pose_position_y.setToolTip('Target value y-direction')
                    target_pose_jog_layout_h.addWidget(self.display_widget_target_pose_position_y)
                elif index == len(translation_list)/2 and i == 2:
                    self.display_widget_target_pose_position_z = QLineEdit()
                    self.display_widget_target_pose_position_z.setFixedWidth(100)
                    self.display_widget_target_pose_position_z.setReadOnly(True)
                    self.display_widget_target_pose_position_z.setToolTip('Target value z-direction')
                    target_pose_jog_layout_h.addWidget(self.display_widget_target_pose_position_z)
                button = QPushButton(str(element))
                button.setFixedWidth(60)
                button.clicked.connect(partial(self.clb_change_target_pose, self.pm_robot_control, key, element))
                target_pose_jog_layout_h.addWidget(button)
            target_pose_jog_layout.addLayout(target_pose_jog_layout_h)

        orientation_label = QLabel('Orientation [quaternion]')
        orientation_label.setFont(self.font_label)
        orientation_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.display_widget_target_pose_orientation_w = QLineEdit()
        self.display_widget_target_pose_orientation_w.setFixedWidth(100)
        self.display_widget_target_pose_orientation_x = QLineEdit()
        self.display_widget_target_pose_orientation_x.setFixedWidth(100)
        self.display_widget_target_pose_orientation_y = QLineEdit()
        self.display_widget_target_pose_orientation_y.setFixedWidth(100)
        self.display_widget_target_pose_orientation_z = QLineEdit()
        self.display_widget_target_pose_orientation_z.setFixedWidth(100)
        target_pose_jog_layout.addWidget(orientation_label)
        target_pose_jog_layout.addWidget(self.display_widget_target_pose_orientation_w, alignment=Qt.AlignmentFlag.AlignCenter)
        target_pose_jog_layout.addWidget(self.display_widget_target_pose_orientation_x, alignment=Qt.AlignmentFlag.AlignCenter)
        target_pose_jog_layout.addWidget(self.display_widget_target_pose_orientation_y, alignment=Qt.AlignmentFlag.AlignCenter)
        target_pose_jog_layout.addWidget(self.display_widget_target_pose_orientation_z, alignment=Qt.AlignmentFlag.AlignCenter)
        self.vertical_layout.addLayout(target_pose_jog_layout)

    def set_active_tool(self):
        # for changing active tool
        self.pm_robot_control.set_active_tool(self.active_tool_combobox.currentText())
        self.pm_robot_control.update_current_tool_pose()
        self.pm_robot_control.update_target_pose_with_current_pose()
        self.update_target_pose_widget()

    def cbk_timer_update_widget_current_tool_pose(self):
        # for timer to update current tool pose
        self.pm_robot_control.update_current_tool_pose()
        self.display_widget_current_pose_position_x.setText(str(round(self.pm_robot_control.current_tool_pose.position.x,6)))
        self.display_widget_current_pose_position_y.setText(str(round(self.pm_robot_control.current_tool_pose.position.y,6)))
        self.display_widget_current_pose_position_z.setText(str(round(self.pm_robot_control.current_tool_pose.position.z,6)))
        if self.pm_robot_control.frame_added:
            self.target_frame_combobox.clear()
            self.target_frame_combobox.addItems(self.pm_robot_control.available_frames)
            self.pm_robot_control.frame_added = False
        self.populateTable()

    def cbk_target_frame_changed(self):
        self.pm_robot_control.set_target_pose_from_frame_world_transform(self.target_frame_combobox.currentText())
        self.update_target_pose_widget()

    def automove_active(self):
        # this method will be overritten in the init
        pass

    def populateTable(self):
        #column_names = ['Name', 'Position', 'Velocity', 'Effort']
        if len(self.pm_robot_control.joint_state_list) == 0:
            return
        column_names = ['Name', 'Position [m]']
        self.table_widget.setHorizontalHeaderLabels(column_names)
        self.table_widget.setRowCount(len(self.pm_robot_control.joint_state_list))
        self.table_widget.setColumnCount(len(self.pm_robot_control.joint_state_list[0]))
        self.table_widget.setColumnCount(len(column_names))
        self.table_widget.setColumnWidth(0, 200)
        self.table_widget.setColumnWidth(1, 200)

        for row, joint_state in enumerate(self.pm_robot_control.joint_state_list):
            for col, data in enumerate(joint_state):
                if isinstance(data, float):
                    data = round(data, 9)
                item = QTableWidgetItem(str(data))
                self.table_widget.setItem(row, col, item)

class DummyMain(QMainWindow):

    def __init__(self, node:Node):
        super().__init__()
        self.node = node
        self.w = None  # No external window yet.
        self.button = QPushButton("Push for Window")
        self.button.clicked.connect(self.show_new_window)
        self.setCentralWidget(self.button)

    def show_new_window(self, checked):
        if self.w is None:
            self.w = PmDashboardApp(self.node)
        self.w.show()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=6) 

    app = QApplication(sys.argv)
    just_a_node = Node('my_Node')
    executor.add_node(just_a_node)

    thread = Thread(target=executor.spin)
    thread.start()
    ex = DummyMain(just_a_node)
    try:
        ex.show()
        sys.exit(app.exec())

    finally:
        just_a_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()

