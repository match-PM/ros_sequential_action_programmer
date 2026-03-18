from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import ActionResponseParameterReference, SeqParameterReference
from typing import Union

class ActionParameterReferences():
    def __init__(self):
        self.reference_list: list[ActionResponseParameterReference, SeqParameterReference] = []

    def add_reference(self, reference:Union[ActionResponseParameterReference, SeqParameterReference]):
        if not isinstance(reference, (ActionResponseParameterReference, SeqParameterReference)):
            raise ValueError("Reference must be an instance of ActionResponseParameterReference or SeqParameterReference.")

        key_already_referenced = self.has_key_reference(reference.get_value_key())

        if not key_already_referenced:  
            self.reference_list.append(reference)
            
    def del_reference(self, field_key: str):
        for index, ref in enumerate(self.reference_list):
            if field_key == ref.get_value_key():
                del self.reference_list[index]
                break  # stop after deleting the first (and only) match
            
    def has_key_reference(self, field_key)->bool:
        for index, ref in enumerate(self.reference_list):
            if field_key == ref.get_value_key():
                return True
        return False

    def get_ref_id_for_field_key(self, field_key)->str:
        for index, ref in enumerate(self.reference_list):
            if not field_key == ref.get_value_key():
                continue

            if isinstance(ref, ActionResponseParameterReference):
                ref: ActionResponseParameterReference 
                action = ref.get_reference_action()
                output = f"{action.get_name()}.response.{ref.get_reference_key()}"
                return output
            
            if isinstance(ref, SeqParameterReference):
                ref: SeqParameterReference 
                output = f"Param: {ref.get_parameter_name()}"
                return output
                
        return None
    
    def get_reference_action_for_field_key(self, field_key)->any:
        for index, ref in enumerate(self.reference_list):
            if isinstance(ref, ActionResponseParameterReference):
                if field_key == ref.get_value_key():
                    return ref.get_reference_action()
        return None
    
    def get_all_value_keys_with_reference(self)->list[str]:
        key_list :list[str] = []
        for index, ref in enumerate(self.reference_list):
            key_list.append(ref.get_value_key())
        return key_list
    
    def get_reference_list(self)->list[Union[ActionResponseParameterReference, SeqParameterReference]]:
        return self.reference_list