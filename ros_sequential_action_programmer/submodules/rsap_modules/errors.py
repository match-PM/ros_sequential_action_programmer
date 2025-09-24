
class ActionInitializationError(Exception):
    def __init__(self, message="Error occured in RSAP."):
        self.message = message
        super().__init__(self.message)
        
class SetActionRequestError(Exception):
    def __init__(self, message="Action Request could not be set.!"):
        self.message = message
        super().__init__(self.message)