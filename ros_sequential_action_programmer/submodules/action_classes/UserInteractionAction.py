import rclpy
from rclpy.node import Node
import collections
import copy
import json
from datetime import datetime
from rosidl_runtime_py.get_interfaces import get_service_interfaces
import numpy as np
from PyQt6.QtWidgets import  QDialog
from ros_sequential_action_programmer.submodules.RsapApp_submodules.UserInteractionActionDialog import UserInteractionActionDialog
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from typing import Tuple, Any
from PyQt6.QtCore import Qt, QByteArray, pyqtSignal, QObject, QThread
from collections import OrderedDict
from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError, EvaluateActionReferenceError
from typing import Union


class UserInteractionModes():
    TERMINAL = 1
    GUI = 2

TERMINAL = 1
GUI = 2

class OpenUserInteractionSignal(QObject):
    """Signal to notify that the execution of the sequence has been started."""
    signal = pyqtSignal(str, object)

class ResultReadySignal(QObject):
    """Signal to notify that the execution of the sequence has been started."""
    signal = pyqtSignal(bool)


class UserInteractionActionRequest():
    def __init__(self, text: str) -> None:
        self.interaction_text:str = text
        
class UserInteractionActionResponse():
    def __init__(self) -> None:
        self.success:bool = False
        
        
class UserInteractionAction(ActionBaseClass):

    MODES = UserInteractionModes()

    def __init__(self, node: Node, 
                 interaction_mode: int, 
                 name:str ='UserInteraction', 
                 action_text:str = "",
                 description:str="") -> None:
        #super().__init__(node)
        super().__init__(node, name, description)

        self.node = node
        self.name = name
        self.interaction_mode = interaction_mode
        
        self.request = UserInteractionActionRequest(action_text)
        self.response = UserInteractionActionResponse()
        
        if name == "":
            self.name = 'UserInteraction'
        self.log_entry={}
        description = ""
        
        self.open_user_interaction_signal = OpenUserInteractionSignal()
        self.result_ready_signal = ResultReadySignal()


    def execute(self, get_interupt_method = None) -> bool:
        exec_success = False

        start_time = datetime.now()

        try:
            new_request_dict = self.evaluate_references()
            self.set_request_from_dict(new_request_dict)
            
        except EvaluateActionReferenceError as e:
            self.node.get_logger().error(f"Error occured evaluating action references for service action '{self.get_name()}'! {str(e)}")
            return False
    
        if self.interaction_mode == TERMINAL:
            self.node.get_logger().info(self.request.interaction_text)
            self.node.get_logger().info("Enter y/n and press enter to proceed! Waiting for user interaction...")
            user_input = input("Enter something: ")
            if user_input == 'y':
                exec_success = True

        elif self.interaction_mode == GUI:
            result = self.request_user_interaction(self.request.interaction_text)
            exec_success = result
            # self.node.get_logger().error("User interaction successful10000!")
            # user_dialog = UserInteractionActionDialog(self.action_text)
            # self.node.get_logger().error("User interaction successful2000!")
            # result = user_dialog.exec()
            # self.node.get_logger().error("User interaction successful1!")
            # if result == QDialog.DialogCode.Accepted:
                
            #     exec_success = True
            #     self.node.get_logger().error("User interaction successful2!")

        end_time = datetime.now()
        self.update_log_entry(success=exec_success, start_time=start_time, end_time=end_time)
   
        return exec_success
    
    def get_request_type(self):
        type_dict = {'interaction_text': {'type': 'str'}}
        return type_dict
    
    def get_response_type(self):
        type_dict = {'success': {'type': 'bool'}}
        return type_dict
    
    def request_user_interaction(self, messsage:str)->bool:
        result_holder = {"result": None}
        
        self.open_user_interaction_signal.signal.emit(messsage, result_holder)
        
        while result_holder["result"] is None:
            QThread.msleep(100)
            
        return result_holder["result"]
    
    def get_type_indicator(self)->str:
        return "UserInteractionAction"
    
    def get_interaction_text(self)->str:
        return self.request.interaction_text
    
    def set_interaction_text(self, new_text:str):
        self.request.interaction_text = new_text
    
    def get_request_as_ordered_dict(self)->OrderedDict:
        dictionary = OrderedDict([('interaction_text', self.request.interaction_text)])
        return dictionary
    
    def get_response_as_ordered_dict(self)->OrderedDict:
        dictionary = OrderedDict([('success', self.response.success)])
        return dictionary
    
    def set_request_from_dict(self,request_dictionary:Union[dict,OrderedDict]):
        """Update the ros service message from the dict"""
        try:
            self.request.interaction_text = request_dictionary['interaction_text']

        except Exception as e:
            raise SetActionRequestError(f"Could not set request from dictionary for {self.get_name()}! Error: {str(e)}")
        
    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime, additional_text:str = ""):
        #self.log_entry={}
        self.log_entry["description"] = self.request.interaction_text
        self.log_entry["start_time"] = str(start_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["end_time"] = str(end_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["execution_time"] = str(end_time - start_time)
        self.log_entry["request"] = {"interaction_text": self.request.interaction_text}
        self.log_entry["success"] = success

    def get_log_entry(self) -> dict:
        return self.log_entry
            
    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        new_instance = UserInteractionAction(node=self.node, 
                                             interaction_mode=self.interaction_mode, 
                                             name=f"{self.name}_copy", 
                                             action_text=self.request.interaction_text)
        new_instance.set_name(f"{self.get_name()}_copy")
        new_instance.set_references(copy.deepcopy(self.get_references()))
        return new_instance
    