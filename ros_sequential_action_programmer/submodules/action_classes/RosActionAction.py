import rclpy
from rclpy.node import Node
from datetime import datetime
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_interface, get_action
from rosidl_runtime_py.get_interfaces import get_service_interfaces, get_action_interfaces
from rqt_py_common import message_helpers
import collections
import copy
import json
import array
import numpy as np
from functools import partial
from ros_sequential_action_programmer.submodules.obj_dict_modules.dict_functions import flatten_dict_to_list, is_string_in_dict
from ros_sequential_action_programmer.submodules.obj_dict_modules.obj_functions import get_obj_value_from_key, set_obj_value_from_key, get_last_index_value
from typing import Union
import re
from typing import Tuple, Any
import rclpy.action
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from rclpy.action import ActionClient
from typing import Tuple, Any
from action_msgs.msg import GoalStatus

from rosidl_parser.definition import BasicType, Array, AbstractNestedType, NamespacedType
from collections import OrderedDict
from ros_sequential_action_programmer.submodules.action_classes.ros_messages_functions import field_type_map_recursive, field_type_map_recursive_with_msg_type
from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError, EvaluateActionReferenceError


STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

class RosActionAction(ActionBaseClass):
    def __init__(self, node: Node, client, action_type, name=None, description = "") -> None:
        super().__init__(node, name, description)

        self.client = client
        self.action_type = action_type

        self.metaclass = None
        self.response = None

        self.action_res_bool_messages = []
        self.action_success_key = None
        
        if name is None:
            self.name = self.client
        else:
            self.name = name

        self.valid = self.check_for_valid_inputs()

        if not self.valid:
           raise ActionInitializationError
        
        self.init_action()
        #self.init_service_res_bool_messages()

    def init_action(self):
        try:
            self.request = self.get_goal(self.action_type)
            self.empty_response = self.get_empthy_result(self.action_type)
            self.default_response_dict = message_to_ordereddict(self.empty_response)
            self.request_dict = message_to_ordereddict(self.request)
            self.metaclass = self.get_metaclass(self.action_type)
            self.response_dict = copy.deepcopy(self.default_response_dict)
            self.default_request = copy.deepcopy(self.request)

            self.node.get_logger().debug(f"Action {str(self.request_dict)} initialized!")

        except Exception as e:
            self.node.get_logger().fatal(str(e))
            self.node.get_logger().fatal(f"Error occured! Given action type does not exist (action_type: {self.action_type})!")  

    def check_for_valid_inputs(self) -> bool:

        list_of_active_ros_actions = rclpy.action.get_action_names_and_types(self.node)
        list_of_active_clients = [item[0] for item in list_of_active_ros_actions]
        list_of_client_types = [item[1][0] for item in list_of_active_ros_actions]
        service_interfaces = flatten_dict_to_list(get_action_interfaces())
        self.node.get_logger().debug(f"clients: {list_of_active_clients}")
        self.node.get_logger().debug(f"types: {list_of_client_types}")

        if self.action_type is not None:
            if not self.action_type in service_interfaces:
                self.node.get_logger().fatal(f"Ros Action Action could not be instantiated. Serivce type '{self.action_type}' does not exist!")
                return False
            else:
                return True
        else:
            for index, client in enumerate(list_of_active_clients):
                if client == self.client:
                    self.action_type = list_of_client_types[index]
                    return True
            return False
    

    def get_type_indicator(self)->str:
        return "RosActionAction"

    def execute(self, get_interupt_method: Any = None) -> bool:
        self.node.get_logger().error("Test 1")

        try:
            new_request_dict = self.evaluate_references()
            self.set_request_from_dict(new_request_dict)
        except EvaluateActionReferenceError as e:
            self.node.get_logger().error(
                f"Error occurred evaluating action references for service action '{self.get_name()}': {e}"
            )
            return False

        if not (self.request and self.metaclass and self.action_type):
            self.node.get_logger().error("Invalid request or metaclass/action_type not set.")
            return False

        self.node.get_logger().error("Test 2")

        execute_success = False
        _client = ActionClient(self.node, self.metaclass, self.client)
        srv_start_time = datetime.now()

        # Wait for server
        if not _client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error(f"Client {self.client} not available, aborting...")
            self.response_dict = collections.OrderedDict([("Error", "Client not available")])
            self.update_log_entry(False, srv_start_time, datetime.now(), additional_text="Client not available!")
            return False

        self.node.get_logger().info(f"Executing '{self.name}'...")
        self.node.get_logger().warn(f"Request {self.request}")
        self.node.get_logger().warn(f"Request dict {self.request_dict}")

        # Send goal asynchronously
        goal_future = _client.send_goal_async(self.request)
        rclpy.spin_until_future_complete(self.node, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("Goal was rejected by the action server.")
            self.update_log_entry(False, srv_start_time, datetime.now(), additional_text="Goal rejected.")
            _client.destroy()
            return False

        self.node.get_logger().info("Goal accepted. Waiting for result...")

        # Get result future
        result_future = goal_handle.get_result_async()

        # Monitor loop for cancellation
        while not result_future.done():
            if get_interupt_method and get_interupt_method():
                self.node.get_logger().warn("Interrupt detected! Cancelling goal...")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future)
                self.node.get_logger().info("Goal cancel request sent.")
                break

            # Small sleep so we don’t block the executor
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # If canceled before completion, skip waiting for result
        if not result_future.done():
            self.node.get_logger().warn("Goal was canceled before completion.")
            self.response_dict = collections.OrderedDict([("Status", "Canceled")])
            self.update_log_entry(False, srv_start_time, datetime.now(), additional_text="Goal canceled by user.")
            _client.destroy()
            return False

        result_info = result_future.result()
        result = result_info.result
        status = result_info.status

        self.response = result
        self.response_dict = message_to_ordereddict(self.response)

        if status == GoalStatus.STATUS_SUCCEEDED:
            execute_success = True
            self.node.get_logger().info("✅ Action completed successfully.")
        elif status == GoalStatus.STATUS_CANCELED:
            execute_success = False
            self.node.get_logger().warn("⚠️ Action was canceled before completion.")
        else:
            execute_success = False
            self.node.get_logger().error(f"❌ Action failed or aborted (status={status}).")

        srv_end_time = datetime.now()
        _client.destroy()
        self.update_log_entry(execute_success, srv_start_time, srv_end_time)

        return execute_success
        
    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime, additional_text:str = ""):
        self.log_entry["ros_action_client"] = self.client
        self.log_entry["action_type"] = self.action_type
        self.log_entry["srv_start_time"] = str(
            start_time.strftime("%Y-%m-%d_%H:%M:%S.%f")
        )
        self.log_entry["srv_end_time"] = str(end_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["execution_time"] = str(end_time - start_time)
        self.log_entry["request"] = json.loads(json.dumps(self.get_request_as_ordered_dict()))
        self.log_entry["response"] = json.loads(json.dumps(self.get_response_as_ordered_dict()))
        if not additional_text == '':
            self.log_entry["message"] = str(additional_text)
        self.log_entry["success"] = success

        self.node.get_logger().debug(f"Log entry: {self.log_entry}")

    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        new_instance = RosActionAction(client=self.client, action_type=self.action_type, node=self.node)
        new_instance.request_dict = copy.deepcopy(self.request_dict)
        new_instance.response = copy.deepcopy(self.response)
        new_instance.request = copy.deepcopy(self.request)
        new_instance.response_dict = copy.deepcopy(self.response_dict)
        new_instance.set_references(copy.deepcopy(self.get_references()))
        new_instance.log_entry = copy.deepcopy(self.log_entry)
        new_instance.set_name(f"{self.get_name()}_copy")

        return new_instance
    
    def get_init_success(self)-> bool:
        return self.valid
    
    def set_request_from_dict(self,request_dictionary:Union[dict,OrderedDict]) -> bool:
        """Update the ros service message from the dict"""
        try:
            set_message_fields(self.request, request_dictionary)
        except Exception as e:
            raise SetActionRequestError(f"Could not set request from dictionary for {self.name}!")
        
    def get_request_as_ordered_dict(self)->OrderedDict:
        return message_to_ordereddict(self.request)
    
    def get_response_as_ordered_dict(self):
        return message_to_ordereddict(self.response)

    def get_request_type(self):
        # slt = self.service_metaclass.Request.__slots__
        # #slt = ""
        # type = self.service_metaclass.Request.SLOT_TYPES
        # #type = (get_message_slot_types(self.service_metaclass.Request))
        # self.node.get_logger().warn(f"slt: {slt}, type: {type}")
        
        #self.node.get_logger().warn(f"slt: {slt}, type: {type}")

        type_dict = field_type_map_recursive_with_msg_type(self.metaclass.Goal)
        # 

        #self.node.get_logger().warn(f"dict HERE: {type_dict}")
        
        return type_dict
    
    def get_response_type(self):
        type_dict = field_type_map_recursive_with_msg_type(self.metaclass.Result)        
        return type_dict
    
    @staticmethod
    def get_goal(type):
        try:
            goal = get_action(type).Goal()
            return goal
        except Exception:
            return None
        
    @staticmethod
    def get_empthy_result(type):
        response = get_action(type).Result()
        return response
    
    @staticmethod
    def get_metaclass(type):
        metaclass = get_action(type)
        return metaclass