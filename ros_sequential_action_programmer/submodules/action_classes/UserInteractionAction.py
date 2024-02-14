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
class UserInteractionModes():
    TERMINAL = 1
    GUI = 2

TERMINAL = 1
GUI = 2
class UserInteractionAction():

    MODES = UserInteractionModes()

    def __init__(self, node: Node, interaction_mode: int, name:str ='UserInteraction', action_text:str = "") -> None:
        #super().__init__(node)
    
        self.node = node
        self.name = name
        self.action_text = action_text
        self.interaction_mode = interaction_mode
        if name == "":
            self.name = 'UserInteraction'
        self.log_entry={}


    def execute(self) -> bool:
        exec_success = False

        start_time = datetime.now()

        if self.interaction_mode == TERMINAL:
            self.node.get_logger().info(self.action_text)
            self.node.get_logger().info("Enter y/n and press enter to proceed! Waiting for user interaction...")
            user_input = input("Enter something: ")
            if user_input == 'y':
                exec_success = True

        elif self.interaction_mode == GUI:
            user_dialog = UserInteractionActionDialog(self.action_text)
            result = user_dialog.exec()

            if result == QDialog.DialogCode.Accepted:
                
                exec_success = True

        end_time = datetime.now()
        self.update_log_entry(success=exec_success, start_time=start_time, end_time=end_time)
   
        return exec_success
    
    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime, additional_text:str = ""):
        #self.log_entry={}
        self.log_entry["description"] = self.action_text
        self.log_entry["start_time"] = str(start_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["end_time"] = str(end_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["execution_time"] = str(end_time - start_time)
        self.log_entry["success"] = success

    def get_log_entry(self) -> dict:
        return self.log_entry
    
    def get_action_name(self)-> str:
        return self.name
    
    def set_action_name(self, new_name:str) -> bool:
        try:
            self.name = new_name
            return True
        except Exception as e:
            self.node.get_logger().error(str(e))
            return False
        
    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        new_instance = UserInteractionAction(node=self.node, interaction_mode=self.interaction_mode, name=f"{self.name}_copy", action_text=self.action_text)
        
        return new_instance
    