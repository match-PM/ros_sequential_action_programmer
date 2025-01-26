from ament_index_python.packages import get_package_share_directory
import os
# import ros logger
from rclpy.impl.rcutils_logger import RcutilsLogger
import yaml

class RsapConfig:
    def __init__(self, package_name:str,
                 logger: RcutilsLogger=None) -> None:
        self.ros_log_levels = RosLogLevels()
        self.execution_log = ExecutionLog()
        self._package_name = package_name
        self.config_file_path = get_package_share_directory('ros_sequential_action_programmer')
        self.file_path = self.config_file_path + '/rsap_config.yaml'
        self.logger = logger
        self.load_config_from_file()
        
    def get_as_dict(self) -> dict:
        return {
            'ros_log_levels': self.ros_log_levels._get_as_dict(),
            'execution_logging': self.execution_log._get_as_dict()
        }
        
    def set_from_dict(self, config_dict: dict, save_to_file: bool = True):
        self.ros_log_levels._set_from_dict(config_dict.get('ros_log_levels', {}))
        self.execution_log._set_from_dict(config_dict.get('execution_logging', {}))
        if save_to_file:
            self.save_config_to_file()
    
    def save_config_to_file(self)->bool:
        try:
            with open(self.file_path, 'w') as file:
                yaml.dump(self.get_as_dict(), file, default_flow_style=False)
            
            self.log_info('Config saved to file: ' + self.file_path)
            return True
        
        except Exception as e:
            self.log_error('Error saving config to file: ' + self.file_path)
            self.log_error(str(e))
            return False
            
    def load_config_from_file(self)->bool:
        # if file does not exist, create it
        if not os.path.exists(self.file_path):
            self.save_config_to_file()
            self.log_info('Config file created: ' + self.file_path)
            
            return True
        try:
            with open(self.file_path, 'r') as file:
                yaml_content = file.read()

            self.set_from_dict(yaml.safe_load(yaml_content), save_to_file=False)
            self.log_info('Config loaded from file: ' + self.file_path)
            return True
        except Exception as e:
            print(str(e))
            self.log_error('Error loading config from file: ' + self.file_path)
            self.log_error(str(e))
            return False
    
    def log_info(self, message:str):
        if self.logger is not None:
            self.logger.info(message)
            
    def log_warn(self, message:str):
        if self.logger is not None:
            self.logger.warn(message)
            
    def log_error(self, message:str):
        if self.logger is not None:
            self.logger.error(message)
            
    def log_debug(self, message:str):
        if self.logger is not None:
            self.logger.debug(message)

class ExecutionLog:
    LOG_NEVER = 0
    LOG_AT_END = 1
    LOG_ALWAYS = 2
    
    def __init__(self):
        self._execution_log_mode = ExecutionLog.LOG_NEVER
    
    def _get_as_dict(self) -> dict:
        return {
            'execution_log_mode': self._get_state_as_string(self._execution_log_mode),
            '_fields_execution_log_mode': self._get_states_as_list()
        }
    
    def _set_from_dict(self, log_dict: dict):
        self._execution_log_mode = self._get_state_from_string(log_dict.get('execution_log_mode', 'LOG_AT_END'))
    
    def _get_state_as_string(self, state:int)->str:
        if state == ExecutionLog.LOG_NEVER:
            return 'LOG_NEVER'
        elif state == ExecutionLog.LOG_AT_END:
            return 'LOG_AT_END'
        elif state == ExecutionLog.LOG_ALWAYS:
            return 'LOG_ALWAYS'
        else:
            return 'UNKNOWN'
        
    def _get_states_as_list(self)->list:
        return [self._get_state_as_string(ExecutionLog.LOG_NEVER), 
                self._get_state_as_string(ExecutionLog.LOG_AT_END),
                self._get_state_as_string(ExecutionLog.LOG_ALWAYS)]
    
    def _get_state_from_string(self, state_str:str)->int:
        if state_str == 'LOG_NEVER':
            return ExecutionLog.LOG_NEVER
        elif state_str == 'LOG_AT_END':
            return ExecutionLog.LOG_AT_END
        elif state_str == 'LOG_ALWAYS':
            return ExecutionLog.LOG_ALWAYS
        else:
            return ExecutionLog.LOG_AT_END
    
    def get_execution_log_mode(self)->int:
        return self._execution_log_mode
    
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
    
    def _get_as_dict(self) -> dict:
        return {
            'log_info': self._log_info,
            'log_warn': self._log_warn,
            'log_error': self._log_error,
            'log_debug': self._log_debug
        }
    
    def _set_from_dict(self, log_dict: dict):
        self._log_info = log_dict.get('log_info', True)
        self._log_warn = log_dict.get('log_warn', True)
        self._log_error = log_dict.get('log_error', True)
        self._log_debug = log_dict.get('log_debug', True)