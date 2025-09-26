
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI, TERMINAL
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from ros_sequential_action_programmer.submodules.action_classes.ParameterValueSetGenerator import ParameterValueSetGenerator

from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError

from rclpy.node import Node

class RefKeyListElement():
    def __init__(self, 
            action_name:str,
            action_index:int,
            keys:list[str]):
        self.action_name = action_name
        self.action_index = action_index
        self.key = keys
        
        
class RefKeyList():
    def __init__(self):
        self._list: list[RefKeyListElement] = []
        
    def as_str_list(self)->list[str]:
        return ["1", "2", "3"]
    
    def append_element(self, new_entry:RefKeyListElement):
        if isinstance(new_entry, RefKeyListElement):
            self._list.append(new_entry)
        else:
            raise ValueError("HEERRRREE")
        
    def get_list(self):
        return self._list

class ActionParameterValueManager():
    def __init__(self, 
                sequence_list: list[ActionBaseClass], 
                node:Node):
        self._node = node
        self.parameter_values_set_generator = ParameterValueSetGenerator(self._node)
        self._sequence_list = sequence_list
        
    def get_comp_res_ref_keys(self, 
                              target_action:ActionBaseClass,
                              field_type:str)->RefKeyList:
        keys_list = RefKeyList()
        for index, action in enumerate(self._sequence_list):
            if action is target_action:
                return keys_list

            _keys = action.get_response_keys_for_type(field_type)
            _name = action.get_name()
            
            _new_element = RefKeyListElement(action_name=_name,
                                    action_index=index,
                                    keys=_keys)
                        
            keys_list.append_element(new_entry=_new_element)
        
        return keys_list
    
    def get_value_headers(self,
                         field_type:str)->list[str]:
        headers = self.parameter_values_set_generator.value_sets.get_all_value_set_names(field_type)
        return headers
    
    def get_values_for_set_header(self, set_header:str):
        _set = self.parameter_values_set_generator.value_sets.get_set_for_set_name(set_header)
        return _set.get_values_list()
    
    