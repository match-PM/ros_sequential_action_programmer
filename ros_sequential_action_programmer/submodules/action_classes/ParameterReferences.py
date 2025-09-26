from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import ParameterReference

class ParameterReferences():
    def __init__(self):
        self.reference_list: list[ParameterReference] = []
        
    def add_reference(self, reference:ParameterReference):
        if isinstance(reference, ParameterReference):
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

    def get_reference_key_for_field_key(self, field_key)->bool:
        for index, ref in enumerate(self.reference_list):
            if field_key == ref.get_value_key():
                return True
        return False
    
    def get_reference_action_for_field_key(self, field_key)->any:
        for index, ref in enumerate(self.reference_list):
            if field_key == ref.get_value_key():
                return ref.get_reference_action()
        return None