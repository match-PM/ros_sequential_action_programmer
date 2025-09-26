class ParameterReference():
    def __init__(self, 
                 value_key:str, 
                 reference_key:str,
                 action_p:any,
                 ):
        self._value_key = value_key
        self._reference_key = reference_key
        self._action = action_p

    def get_value_key(self):
        return self._value_key

    def get_reference_key(self):
        return self._reference_key
    
    def get_reference_action(self):
        return self._action