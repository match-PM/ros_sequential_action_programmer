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
        self.node = node

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
           return None
        
        self.init_action()
        #self.init_service_res_bool_messages()

    def init_action(self):
        try:
            self.request = self.get_goal(self.action_type)
            self.empty_response = self.get_empthy_result(self.action_type)
            self.default_response_dict = message_to_ordereddict(self.empty_response)
            self.request_dict = message_to_ordereddict(self.request)
            self.metaclass = self.get_metaclass(self.action_type)
            self.request_dict_implicit = copy.deepcopy(self.request_dict)
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
    



    def execute(self) -> bool:
        if self.request and self.metaclass and self.action_type:
            # update srv request from dictionary
            self.update_request_obj_from_dict()

            execute_success = False

            _client = ActionClient(self.node, self.metaclass, self.client)
            
            srv_start_time = datetime.now()
            
            if not _client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().error(f"Client {self.client} not available, aborting...")
                self.response_dict = collections.OrderedDict([("Error", "Client not available")])
                execute_success = False
                srv_end_time = datetime.now()
                self.update_log_entry(execute_success, srv_start_time, srv_end_time,additional_text='Client not available! Exited with error!')
                return execute_success

            self.node.get_logger().info(f"Executing '{self.name}'...")

            srv_start_time = datetime.now()
            # Call the service

            self.node.get_logger().warn(f"Request {self.request}")
            self.node.get_logger().warn(f"Request dict {self.request_dict}")
            self.node.get_logger().warn(f"Request dict implicit {self.request_dict_implicit}")

            future = _client.send_goal(self.request)
            
            self.node.get_logger().info(f"{future.status}")
            self.node.get_logger().info(f"{future}")

            self.response = future.result
            status = future.status

            # update srv response dict
            self.response_dict = message_to_ordereddict(self.response)
            
            if status == STATUS_SUCCEEDED:
                execute_success = True
            else:
                execute_success = False
            srv_end_time = datetime.now()

            _client.destroy()

            self.update_log_entry(execute_success, srv_start_time, srv_end_time)
            #self.node.get_logger().info(f"Service return: {self.response_dict}")
            return execute_success
        
    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime, additional_text:str = ""):
        self.log_entry["ros_action_client"] = self.client
        self.log_entry["action_type"] = self.action_type
        self.log_entry["srv_start_time"] = str(
            start_time.strftime("%Y-%m-%d_%H:%M:%S.%f")
        )
        self.log_entry["srv_end_time"] = str(end_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["execution_time"] = str(end_time - start_time)
        self.log_entry["request"] = json.loads(json.dumps(self.request_dict))
        self.log_entry["response"] = json.loads(json.dumps(self.response_dict))
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
        new_instance.request_dict = copy.deepcopy(self.request_dict)
        new_instance.response_dict = copy.deepcopy(self.response_dict)
        new_instance.log_entry = copy.deepcopy(self.log_entry)
        new_instance.request_dict_implicit = copy.deepcopy(self.request_dict_implicit)

        return new_instance
    
    def get_init_success(self)-> bool:
        return self.valid
    
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