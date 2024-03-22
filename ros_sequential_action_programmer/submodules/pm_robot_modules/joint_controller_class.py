from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class JointJogControl():
    EXEC_DURATION = 1
    def __init__(self, ros_node: Node, joint_names: list, action_name:str) -> None:
        self.node = ros_node
        self.joint_names = joint_names
        self._action_name = action_name
        self.target_joint_values = [0.0] * len(joint_names)
        self.joint_state_list = [None] * len(joint_names)
        callback_group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(self.node, FollowJointTrajectory, self._action_name, callback_group=callback_group)
        self._action_client.wait_for_server()

    def set_current_joint_values(self, joint_state_list: list[float])->bool:
        if len(joint_state_list) != len(self.joint_names):
            self.node.get_logger().error("Length of joint state list does not match the length of joint names")
            return False
        else:
            self.joint_state_list = joint_state_list
            return True
            
    def set_target_joint_value(self, joint_name:str, value:float)->bool:
        if joint_name not in self.joint_names:
            self.node.get_logger().error(f"Joint name {joint_name} not in list of joint names")
            return False
        index = self.joint_names.index(joint_name)
        self.target_joint_values[index] = value
        return True

    def set_target_joint_values(self, target_values:list[float])->bool:
        if len(target_values) != len(self.joint_names):
            self.node.get_logger().error("Length of target values does not match the length of joint names")
            return False
        else:
            self.target_joint_values = target_values
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
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=self.EXEC_DURATION, nanosec=0)  # Time from start for this point
        trajectory_msg.points.append(point)
        # Publish the trajectory
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = trajectory_msg     
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
    
    def set_target_from_current(self)->bool:
        self.target_joint_values = self.joint_state_list
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
            self.set_current_joint_values(self.target_joint_values)