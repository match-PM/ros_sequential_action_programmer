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


class ActionBaseClass:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.description = ""
        self.log_entry = {}
        self.name = None
    
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