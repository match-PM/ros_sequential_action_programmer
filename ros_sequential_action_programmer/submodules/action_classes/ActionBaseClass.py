import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_interface
from rosidl_runtime_py.get_interfaces import get_service_interfaces
from rqt_py_common import message_helpers
import collections
import copy
import json
from datetime import datetime
import array
import numpy as np
from functools import partial
from ros_sequential_action_programmer.submodules.obj_dict_modules.dict_functions import flatten_dict_to_list, is_string_in_dict
from typing import Union
import re
from typing import Tuple, Any
from ros_sequential_action_programmer.submodules.obj_dict_modules.obj_functions import get_obj_value_from_key, set_obj_value_from_key, get_last_index_value
class ActionBaseClass:
    def __init__(self, node: Node,name:str,description:str) -> None:
        self.node = node
        self.description = description
        self.log_entry = {}
        self.name = name

        self.request = None
        self.request_dict_implicit = None
        self.response_dict = None
        self.empty_response = None
        self.default_response_dict = None
        self.request_dict = None
        self.default_request = None

        self.log_entry = {}

    
    def get_action_name(self)-> str:
        return self.name
    
    def set_action_name(self, new_name:str) -> bool:
        try:
            self.name = new_name
            return True
        except Exception as e:
            self.node.get_logger().error(str(e))
            return False

    def execute(self) -> Tuple[bool, Any]:
        raise NotImplementedError
    
    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime, additional_text:str = ""):
        raise NotImplementedError
    
    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        raise NotImplementedError
    
    def get_log_entry(self) -> dict:
        return self.log_entry

    def set_request_dict_value_from_key(self, path_key: str, new_value: any, override_to_implicit=False) -> bool:
        """
        This function tries to set the value of the service request given the path_key to the value and a new value.
        Retuns false if key or value are incopatible with the service request.
        E.a. path_key = 'Foo.Fuu.Faa' (str)
        """
        self.node.get_logger().debug(f"Setting value '{new_value}' for key '{path_key}' in request dict!")

        if path_key is None:
            return False

        if new_value is None:
            return False

        try:

            test_request = copy.deepcopy(self.default_request)

            value_to_set = get_obj_value_from_key(test_request, path_key)

            # if given key leads to an array entry
            if '[' in path_key and ']' in path_key:
                value_is_list_entry = True
                list_path_key = re.sub(r'\[.*?\]', '', path_key)
                self.node.get_logger().debug(f"New path {list_path_key}")
                index = get_last_index_value(path_key)
                self.node.get_logger().debug(f"index is {index}")
            else:
                value_is_list_entry = False

            # in case the value leads to an list entry we will process
            if value_is_list_entry:
                test_dict = copy.deepcopy(self.request_dict_implicit)
                list_to_set = get_obj_value_from_key(test_dict, list_path_key)
                self.node.get_logger().debug(f"List old '{str(list_to_set)}'")
                self.node.get_logger().debug(f"test_dict old '{str(test_dict)}'")

                if list_to_set is None:
                    self.node.get_logger().error(f"Error occured accessing list element '{list_path_key}' in dict!")
                    return False
                
                list_to_set[index] = new_value
                
                if not override_to_implicit:
                    set_success = set_obj_value_from_key(self.request_dict, list_path_key, list_to_set)
                    self.update_request_obj_from_dict()
                else:
                    set_success = set_obj_value_from_key(self.request_dict_implicit, list_path_key, list_to_set)
                return set_success
            
            # Create a new service request object
            # Set test request with dict
            self.node.get_logger().debug(f"Path key {str(path_key)}")
            self.node.get_logger().debug(f"New value {str(new_value)}")
            self.node.get_logger().debug(f"Value to set {str(value_to_set)}")

            self.node.get_logger().debug(f"New value type {str(type(new_value))}")
            self.node.get_logger().debug(f"Value to set type {str(type(value_to_set))}")
            self.node.get_logger().debug(f"{str(type(value_to_set))}")

            if isinstance(value_to_set, np.ndarray) and isinstance(new_value, list):
                new_value = np.array(new_value)

            if isinstance(value_to_set, array.array) and isinstance(new_value, list):
                new_value = array.array("i", new_value)

            self.node.get_logger().debug(f"New value type {str(type(new_value))}")

            if not isinstance(new_value, type(value_to_set)) and not override_to_implicit:
                self.node.get_logger().debug(f"Given value '{new_value}' of type '{type(new_value)}' is incompatible for '{path_key}' of type '{type(value_to_set)}'!")
                return False

            # If the path does not lead to an existing value
            if value_to_set is None:
                return False
            
            if not override_to_implicit:
                set_success = set_obj_value_from_key(self.request_dict, path_key, new_value)
                #self.node.get_logger().debug(f"Set success {str((set_success))}")
                self.update_request_obj_from_dict()
            else:
                #self.node.get_logger().debug(f"Set success {str((self.request_dict_implicit))}")
                set_success = set_obj_value_from_key(self.request_dict_implicit, path_key, new_value)
                #self.node.get_logger().debug(f"Set success {str((self.request_dict_implicit))}")
                #self.node.get_logger().debug(f"Set success {str((set_success))}")

            return set_success
        except:
            self.node.get_logger().debug("Error occured in set_request_dict_value_from_key!")
            return False
        
    def get_new_request_obj(self,type:str)->any:
        raise NotImplementedError

    def update_request_obj_from_dict(self):
        """Update the ros service message from the dict"""
        set_message_fields(self.request, self.request_dict)

    def get_default_response_value_from_key(self, key: str) -> any:
        return get_obj_value_from_key(obj=self.default_response_dict, path_key=key)
    
    def get_response_value_from_key(self, key: str) -> any:
        return get_obj_value_from_key(obj=self.response_dict, path_key=key)
    

    def set_service_request(self, request: any):
        """
        Warning: The method doesn not check for valid inputs.
        This method set the service request object of the class from the input parameter request.
        It also updates the request dict.
        """
        self.request = request
        self.request_dict = message_to_ordereddict(self.request)