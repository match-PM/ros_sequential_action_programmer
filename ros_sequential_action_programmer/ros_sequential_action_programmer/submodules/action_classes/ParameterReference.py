from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameter

class ActionResponseParameterReference():
    def __init__(self, 
                 value_key:str, 
                 reference_key:str,
                 action_pointer:any,
                 ):
        self._value_key = value_key
        self._reference_key = reference_key
        self._action = action_pointer

    def get_value_key(self):
        return self._value_key

    def get_reference_key(self):
        return self._reference_key
    
    def get_reference_action(self):
        return self._action
    

class SeqParameterReference():
    def __init__(self, 
                 value_key:str, 
                 param:SeqParameter):
        self._value_key = value_key
        self._parameter = param

    def get_value_key(self):
        return self._value_key

    def get_parameter(self) -> SeqParameter:
        return self._parameter
    
    def set_parameter(self, param:SeqParameter):
        self._parameter = param

    def get_parameter_name(self):
        return self._parameter.get_name()
