from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from PyQt6.QtCore import pyqtSignal, QObject
import time
from copy import deepcopy,copy
from urdf_parser_py.urdf import URDF
from urdf_parser_py.urdf import Joint as URDFJoint
from urdf_parser_py.urdf import JointLimit as URDFJointLimit
import rclpy
# import GetParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class UpdateValuesSignal(QObject):
    gui_update_current_values_signal = pyqtSignal(object)
    gui_update_target_values_signal = pyqtSignal(object)

class JointJogControl():
    EXEC_DURATION = 1
    def __init__(self, ros_node: Node, joint_names: list, action_name:str, joint_topic:str, robot_state_publisher:str) -> None:
        self.node = ros_node
        self.joint_names = joint_names
        self._action_name = action_name
        self.target_joint_values = [0.0] * len(joint_names)
        self.joint_state_list = [0.0] * len(joint_names)
        self.lower_limits = [-10.0] * len(joint_names)
        self.upper_limits = [10.0] * len(joint_names)

        self.get_robot_description_srv = self.node.create_client(GetParameters, f'{robot_state_publisher}/get_parameters')
        self.get_robot_description()

        callback_group = MutuallyExclusiveCallbackGroup()
        callback_group_2 = ReentrantCallbackGroup()
        self.joint_topic = joint_topic

        self.gui_update_values = UpdateValuesSignal()
        self.topic_sub = self.node.create_subscription(JointState, self.joint_topic, self.joint_state_callback, 10, callback_group = callback_group)
        
        self._action_client = ActionClient(self.node, FollowJointTrajectory, self._action_name, callback_group=callback_group)
        self._action_client.wait_for_server()
        self.subscription_running = False

        while not self.subscription_running:
            self.node.get_logger().info("Waiting for subscription to start")
            time.sleep(0.2)

        self.set_target_from_current()

    def get_joint_limits(self):
        for joint in self.urdf.joints:
            joint: URDFJoint
            if joint.name in self.joint_names:
                limit : URDFJointLimit = joint.limit
                self.lower_limits[self.get_index_of_joint(joint.name)]=limit.lower
                self.upper_limits[self.get_index_of_joint(joint.name)]=limit.upper

    def get_robot_description(self):
        while not self.get_robot_description_srv.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info('Service not available, waiting again...')

        req = GetParameters.Request()
        req.names = ['robot_description']
        # future = self.get_robot_description_srv.call_async(req)
        # while not future.done():
        #     rclpy.spin_once(self.node)
        # response:GetParameters.Response = future.result()
        # value:ParameterValue = response.values[0]
        # robot_description = value.string_value
        response :GetParameters.Response= self.get_robot_description_srv.call(req)
        value:ParameterValue = response.values[0]
        robot_description = value.string_value
        #self.node.get_logger().info(f"Response: {str(robot_description)}")
        self.urdf = URDF.from_xml_string(robot_description)
        self.get_joint_limits()

    def set_current_joint_values(self, joint_state_list: list[float])->bool:
        if len(joint_state_list) != len(self.joint_names):
            self.node.get_logger().error("Length of joint state list does not match the length of joint names")
            return False
        else:
            self.joint_state_list = copy(joint_state_list)
            return True
    
    def joint_state_callback(self, msg:JointState):
        self.subscription_running = True
        for i in range(len(self.joint_names)):
            if self.joint_names[i] in msg.name:
                index = msg.name.index(self.joint_names[i])
                self.joint_state_list[i] = msg.position[index]
        self.gui_update_values.gui_update_current_values_signal.emit(self.joint_state_list)
        self.node.get_logger().debug(f"Current joint values: {self.joint_state_list}")
        time.sleep(0.4)

    def set_target_joint_value(self, joint_name:str, value:float)->bool:
        if joint_name not in self.joint_names:
            self.node.get_logger().error(f"Joint name {joint_name} not in list of joint names")
            return False
        index = self.joint_names.index(joint_name)

        # Check if value is within limits
        if value < self.lower_limits[index]:
            self.node.get_logger().warn(f"Value {value} is out of bounds for joint {joint_name}. Setting to lower limit.")
            value = self.lower_limits[index] 
        elif value > self.upper_limits[index]:
            self.node.get_logger().warn(f"Value {value} is out of bounds for joint {joint_name}. Setting to upper limit.")
            value = self.upper_limits[index]
        
        self.target_joint_values[index] = value
        self.gui_update_values.gui_update_target_values_signal.emit(self.target_joint_values)
        return True

    def get_index_of_joint(self, joint_name:str)->int:
        if joint_name not in self.joint_names:
            self.node.get_logger().error(f"Joint name {joint_name} not in list of joint names")
            return None
        return self.joint_names.index(joint_name)
    
    def set_target_joint_values(self, target_values:list[float])->bool:
        if len(target_values) != len(self.joint_names):
            self.node.get_logger().error("Length of target values does not match the length of joint names")
            return False
        else:
            # Check if values are within limits
            for i in range(len(target_values)):
                if target_values[i] < self.lower_limits[i] or target_values[i] > self.upper_limits[i]:
                    self.node.get_logger().error(f"Value {target_values[i]} is out of bounds for joint {self.joint_names[i]}")
                    return False
            self.target_joint_values = copy(target_values)
            self.gui_update_values.gui_update_target_values_signal.emit(self.target_joint_values)
            return True
    
    def get_target_joint_value_for_joint(self, joint_name:str)->float:
        if joint_name not in self.joint_names:
            self.node.get_logger().error(f"Joint name {joint_name} not in list of joint names")
            return None
        index = self.joint_names.index(joint_name)
        return self.target_joint_values[index]
        
    def get_current_joint_value_for_joint(self, joint_name:str)->float:
        if joint_name not in self.joint_names:
            self.node.get_logger().error(f"Joint name {joint_name} not in list of joint names")
            return None
        index = self.joint_names.index(joint_name)
        return self.joint_state_list[index]
    
    def get_current_joint_values(self):
        return self.joint_state_list
    
    def send_target_joint_values(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.target_joint_values
        self.node.get_logger().debug(f"Sending target values {self.target_joint_values}")
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=self.EXEC_DURATION, nanosec=0)  # Time from start for this point
        trajectory_msg.points.append(point)
        # Publish the trajectory
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = trajectory_msg     
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
    
    def set_target_from_current(self)->bool:
        self.target_joint_values = copy(self.joint_state_list)
        self.gui_update_values.gui_update_target_values_signal.emit(self.target_joint_values)
        return True
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected was rejected by the controller")
            return

        self.node.get_logger().debug("Goal accepted!")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.get_logger().info("Goal succeeded!")
            #self.set_current_joint_values(self.target_joint_values)