
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from ros_sequential_action_programmer.submodules.action_classes.ParameterValueSetGenerator import ParameterValueSetGenerator

from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError
from ros_sequential_action_programmer.submodules.action_classes.compatibility_mapping import COMPATIBLE_TYPES
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameterManager, SeqParameter, SeqParameterError, SeqParameterManagerError
from rclpy.node import Node
from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import ActionResponseParameterReference, SeqParameterReference

class RefKeyListElement():
    def __init__(self, 
            action_name:str,
            action_index:int,
            key:str):
        self.action_name = action_name
        self.action_index = action_index
        self.key = key


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
        self.seq_parameter_manager = SeqParameterManager()
        self._sequence_list = sequence_list

    def get_comp_res_ref_keys(self, 
                            target_action:ActionBaseClass,
                            field_type:str)->RefKeyList:
        """
        Collect compatible response reference keys for a given field type,
        using the REVERSED_COMPATIBLE_TYPES mapping.
        """
        keys_list = RefKeyList()
        
        # Determine compatible types for the requested field_type (who can assign to me)
        compatible_types = COMPATIBLE_TYPES.get(field_type, [field_type])

        for index, action in enumerate(self._sequence_list):
            if action is target_action:
                return keys_list  # stop at the target action

            response_keys = []

            for t in compatible_types:
                keys_for_t = action.get_response_keys_for_type(t)
                response_keys.extend(keys_for_t)

            if not response_keys:
                continue
            
            #self._node.get_logger().error(f"Found compatible response keys in action '{action.get_name()}' for field type '{field_type}': {response_keys}")

            for key in response_keys:
                #self._node.get_logger().error(f"Found compatible response key in action '{action.get_name()}' for field type '{field_type}': {key}")

                _new_element = RefKeyListElement(
                    action_name=action.get_name(),
                    action_index=index,
                    key=key
                )
                keys_list.append_element(_new_element)  # append **inside the loop**

        return keys_list
    
    def get_value_headers(self,
                         field_type:str)->list[str]:
        headers = self.parameter_values_set_generator.value_sets.get_all_value_set_names(field_type)
        return headers
    
    def get_values_for_set_header(self, set_header:str):
        _set = self.parameter_values_set_generator.value_sets.get_set_for_set_name(set_header)
        return _set.get_values_list()
    
    def get_value_set_for_header(self, set_header:str):
        _set = self.parameter_values_set_generator.value_sets.get_set_for_set_name(set_header)
        return _set
    
    def get_action_at_index(self, index:int)->ActionBaseClass|None:
        if index < 0 or index >= len(self._sequence_list):
            return None
        return self._sequence_list[index]
    
    def reinit_sequence_parameter_values(self):
        """
        Call this after the sequence parameter file has been changed externally.
        """

        for index, action in  enumerate(self._sequence_list):
            references = action.get_references()
            to_delete:list[SeqParameterReference] = []

            for ref in references.get_reference_list():
                if not isinstance(ref, SeqParameterReference):
                    continue

                else:
                    param = ref.get_parameter()
                    param_name = param.get_name()
                    try:
                        new_param = self.seq_parameter_manager.get_parameter_by_name(param_name)  # verify parameter exists
                        ref.set_parameter(new_param)
                        self._node.get_logger().info(f"Updated SeqParameterReference to parameter '{param_name}' with value: {new_param.get_value()}")
                    except SeqParameterError as e:
                        self._node.get_logger().warn(f"In '{action.get_name()}': Failed to update SeqParameterReference to parameter '{param_name}'. Loaded parameter file may be missing this parameter: {e}")
                        # delete the reference since the parameter no longer exists
                        to_delete.append(ref)

                    # param.set_value(new_param.get_value())  # re-set to trigger any internal updates
                    # self._node.get_logger().info(f"SeqParameterReference to parameter '{param_name}' with value: {new_param.get_value()}")

            for ref in to_delete:
                references.del_reference(ref.get_value_key())
                self._node.get_logger().warn(f"In '{action.get_name()}' - Deleted SeqParameterReference with for key '{ref.get_value_key()}' due to missing parameter.")
