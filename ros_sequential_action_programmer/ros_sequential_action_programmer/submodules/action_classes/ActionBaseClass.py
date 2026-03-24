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
from typing import Tuple, Any
from ros_sequential_action_programmer.submodules.action_classes.ParameterReferences import ActionParameterReferences
from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import ActionResponseParameterReference, SeqParameterReference
from ros_sequential_action_programmer.submodules.rsap_modules.errors import EvaluateActionReferenceError
import uuid as uuid_lib
from ros_sequential_action_programmer.submodules.action_classes.compatibility_mapping import COMPATIBLE_TYPES
from rosidl_parser.definition import BasicType, Array, AbstractNestedType
from collections import OrderedDict

# Compatibility mapping

def find_fields_by_type(type_dict, target_type, parent_path="", compatible_map=None):
    """
    Recursively find all fields whose type matches target_type (or compatible).

    Args:
        type_dict (dict): ROS2-style type dictionary.
        target_type (str): Type to search for.
        parent_path (str): Internal recursion, for nested field paths.
        compatible_map (dict): Optional compatibility map. Defaults to COMPATIBLE_TYPES.

    Returns:
        List[str]: List of full paths to matching fields.

    Raises:
        ValueError: If target_type is not in compatible_map.
    """
    compatible_map = compatible_map or COMPATIBLE_TYPES

    # Strict check for supported types
    if target_type not in compatible_map:
        raise ValueError(f"Type '{target_type}' is not supported in the compatibility map.")

    matches = []
    compatible_types = compatible_map[target_type]

    for key, val_type in type_dict.items():
        full_path = f"{parent_path}.{key}" if parent_path else key
        typ = val_type.get("type")

        # Check primitive match
        if typ in compatible_types:
            matches.append(full_path)

        # If it's an array, mark with []
        if val_type.get("is_array", False) and typ in compatible_types:
            matches.append(f"{full_path}[]")

        # Recurse into nested fields
        if "fields" in val_type:
            sub_matches = find_fields_by_type(
                val_type["fields"], target_type, parent_path=full_path, compatible_map=compatible_map
            )
            matches.extend(sub_matches)

    return matches


class ActionBaseClass:
    def __init__(self, 
                 node: Node, 
                 name:str, 
                 description:str,
                 uuid: str = None) -> None:
    
        self.node = node
        self._description: str = description
        self.log_entry: dict = {}
        self.name:str = name

        self.request = None
        #self.request_dict_implicit = {}
        self.response_dict = {}
        self.empty_response = None
        self.default_response_dict = None
        self.request_dict = None
        self.default_request = None
        self._uuid: uuid_lib.UUID = uuid_lib.uuid4() if uuid is None else uuid_lib.UUID(uuid)

        self._has_breakpoint = False
        self._is_active = True
        self._success_key = None
        self._parameter_references = ActionParameterReferences()
    
    def get_references(self) -> ActionParameterReferences:
        return self._parameter_references
    
    def set_references(self, new_references: ActionParameterReferences) -> bool:
        if not isinstance(new_references, ActionParameterReferences):
            return False
        else:
            self._parameter_references = new_references
            return True
    
    def get_uuid(self)-> str:
        return str(self._uuid)
    
    def get_name(self)-> str:
        return self.name
    
    def set_name(self, new_name:str) -> bool:
        try:
            self.name = new_name
            return True
        except Exception as e:
            self.node.get_logger().error(str(e))
            return False

    def _set_default_identifier(self):
        if "success" in self.get_res_bool_fields():
            self.set_success_identifier("success")

    def toggle_active(self):
        self.node.get_logger().warn(f"State: {self._is_active}")
        self._is_active = not self._is_active

    def toggle_breakpoint(self):
        self._has_breakpoint = not self._has_breakpoint

    def is_active(self):
        return self._is_active
    
    def has_breakpoint(self):
        return self._has_breakpoint
    
    def set_breakpoint(self, state: bool):
        self._has_breakpoint = state
        
    def set_active(self, state: bool):
        self._is_active = state
        
    def get_log_entry(self) -> dict:
        return self.log_entry

    def set_success_identifier(self, identifier: str) -> bool:
        """
        Sets the current service execution identifier
        """
        self._success_key = identifier
        return True
    
    def get_type_indicator(self)->str:
        return "NotImplemented"
    
    def get_success_identifier(self)-> str:
        return self._success_key
    
    def get_res_bool_fields(self)-> list[str]:
        """
        Returns a list of strings containing the full keys to all bool messages of the service response
        """
        return []
    
    def set_description(self, new_description:str):
        self._description = new_description
    
    def get_description(self)->str:
        return self._description   

    def execute(self, get_interupt_method:Any = None) -> Tuple[bool, Any]:
        raise NotImplementedError
    
    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime, additional_text:str = ""):
        raise NotImplementedError
    
    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        raise NotImplementedError
    
    def get_request_as_ordered_dict(self)->OrderedDict:
        raise NotImplementedError
    
    def get_response_as_ordered_dict(self)->OrderedDict:
        raise NotImplementedError
             
    def set_request_from_dict(self,request_dictionary:Union[dict,OrderedDict]) -> bool:
        raise NotImplementedError
    
    def get_request_type(self):
        raise NotImplementedError
    
    def get_response_type(self):
        raise NotImplementedError
    
    def get_response_keys_for_type(self, field_type:str)->list[str]:
        key_list :list[str] = []
        type_dict = self.get_response_type()
        
        key_list = find_fields_by_type(type_dict=type_dict, 
                                       target_type=field_type)
        return key_list
    
    def get_request_keys_for_type(self, field_type:str)->list[str]:
        key_list :list[str] = []
        type_dict = self.get_request_type()
        key_list = find_fields_by_type(type_dict=type_dict, 
                                target_type=field_type)
        return key_list

    
    def set_request_from_request(self, new_request:any) -> bool:
        if not isinstance(self.request, new_request):
            return False
        else:
            self.request = new_request
            return True
    
    def set_request_dict_value_from_key(self, path_key: str, new_value: any) -> bool:
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

            original_request = copy.deepcopy(self.request_dict)

            value_to_set = get_obj_value_from_key(original_request, path_key)

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
                pass
                
            else:
                set_success = set_obj_value_from_key(original_request, path_key, new_value)
                if not set_success:
                    self.node.get_logger().debug(f"Failed to set value for key '{path_key}'!")
                    return False
                
                set_success = self.set_request_from_dict(original_request)

                if not set_success:
                    self.node.get_logger().debug(f"Failed to set value for key '{path_key}'!")

            return set_success
        
        except Exception as e:
            self.node.get_logger().error(f"Error occured in set_request_dict_value_from_key! {e}")
            return False
    
    def evaluate_references(self)->dict:
        """
        Evaluate all parameter references and update the request dict accordingly.
        """

        copy_request_dict = copy.deepcopy(self.get_request_as_ordered_dict())
        
        for index, _reference in enumerate(self._parameter_references.get_reference_list()):

            if isinstance(_reference, ActionResponseParameterReference):
                _reference: ActionResponseParameterReference
                value_key = _reference.get_value_key()
                response_reference_key = _reference.get_reference_key()
                reference_action = _reference.get_reference_action()

                self.node.get_logger().warn(f"Evaluating action response reference for key '{_reference.get_value_key()}' from action '{_reference.get_reference_action().get_name()}'")

            elif isinstance(_reference, SeqParameterReference):
                _reference: SeqParameterReference
                value_key = _reference.get_value_key()
                seq_parameter = _reference.get_parameter()
                value_to_set = seq_parameter.get_value()
                
                if isinstance(value_to_set, dict):
                    # if dict has only a single key called data
                    if len(value_to_set) == 1 and "data" in value_to_set:
                        set_obj_value_from_key(copy_request_dict, path_key=value_key, new_value=value_to_set["data"])
                    else:
                        set_obj_value_from_key(copy_request_dict, path_key=value_key, new_value=value_to_set)

                #raise EvaluateActionReferenceError(f"Failed to set sequence parameter reference value for key '{value_key}' from parameter '{seq_parameter.get_name()}'")

                self.node.get_logger().warn(f"Evaluating sequence parameter reference for key '{_reference.get_value_key()}' to {value_to_set}")

            else:
                continue
            
            #self.node.get_logger().warn(f"{copy_request_dict}")

        return copy_request_dict
        
    # def update_request_obj_from_dict(self):
    #     """Update the ros service message from the dict"""
    #     set_message_fields(self.request, self.request_dict)

    def get_default_response_value_from_key(self, key: str) -> any:
        return get_obj_value_from_key(obj=self.default_response_dict, path_key=key)
    
    def get_response_value_from_key(self, key: str) -> any:
        return get_obj_value_from_key(obj=self.response_dict, path_key=key)
    
    def clear_log_entry(self):
        self.log_entry = {}
