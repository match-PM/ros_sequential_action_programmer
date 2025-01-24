

class RsapConfig:
    def __init__(self) -> None:
        self.ros_log_levels = RosLogLevels()


class RosLogLevels:
    def __init__(self) -> None:
        self._log_info = True
        self._log_warn = True
        self._log_error = True
        self._log_debug = True

    def set_log_info(self, value: bool):
        self._log_info = value

    def set_log_warn(self, value: bool):
        self._log_warn = value

    def set_log_error(self, value: bool):
        self._log_error = value

    def set_log_debug(self, value: bool):
        self._log_debug = value

    def get_log_info(self) -> bool:
        return self._log_info
    
    def get_log_warn(self) -> bool:
        return self._log_warn
    
    def get_log_error(self) -> bool:
        return self._log_error
    
    def get_log_debug(self) -> bool:
        return self._log_debug
    
    def get_as_dict(self) -> dict:
        return {
            'log_info': self._log_info,
            'log_warn': self._log_warn,
            'log_error': self._log_error,
            'log_debug': self._log_debug
        }
    
    def set_from_dict(self, log_dict: dict):
        self._log_info = log_dict.get('log_info', True)
        self._log_warn = log_dict.get('log_warn', True)
        self._log_error = log_dict.get('log_error', True)
        self._log_debug = log_dict.get('log_debug', True)