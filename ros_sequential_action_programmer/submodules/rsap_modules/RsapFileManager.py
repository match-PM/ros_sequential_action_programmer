
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI, TERMINAL
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass

from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError
import json
import os
from datetime import datetime
from rclpy.node import Node

class RsapFileManager():
    def __init__(self, 
                 sequence_list: list[ActionBaseClass], 
                 node:Node):
        
        self.sequence_list = sequence_list
        self.node = node
        
        self._sequence_file_path = None
        self._folder_path = None
        self._sequence_name = None
        
    def get_sequence_name(self)->str:
        return self._sequence_name
    
    def set_sequence_name(self, name:str):
        self._sequence_name = name
        
    def get_action_sequence_file_path(self)->str:
        return self._sequence_file_path

    def set_folder_path(self, folder_path:str):
        self._folder_path = folder_path

    def get_folder_path(self)->str:
        return self._folder_path
    
    def load_from_JSON(self, file_path) -> bool:
        """
        Sets the action list from a given json-file path.
        :param file_path: The path to the json-file.
        :return: True if the action list was set successfully, False otherwise.
        """
        file_data = None
        loading_action_list = []
        try:
            with open(file_path, "r") as json_file:
                file_data = json.load(json_file)
            # clear action_list to be empty
            self.sequence_list.clear()
        except Exception as e:
            self.node.get_logger().error(f"Error opening file '{file_path}'!")

        if not file_data:
            return False
        
        self._sequence_file_path = file_path
        self._folder_path = os.path.dirname(file_path)
        self._sequence_name = file_data["name"]

        for index, action_dict in enumerate(file_data["action_list"]):
            try:
                action_dict:dict
                _request = action_dict.get("request",{})
                if _request == {}:
                    _request = action_dict.get("service_request",{})
                _description = action_dict["description"]
                _name = action_dict["name"]
                _type = action_dict['action_type']
                
                if _type == 'ServiceAction':
                    _service_client = action_dict["service_client"]
                    _service_type = action_dict["service_type"]

                    action_from_item = ServiceAction(node=self.node,
                                                    client=_service_client,
                                                    service_type=_service_type,
                                                    name=_name)
                    
                    set_success = action_from_item.set_success_identifier(action_dict["error_identifier"])
                    action_from_item.set_description(_description)
                    action_from_item.set_request_from_dict(_request)
                    self.sequence_list.append(action_from_item)

                elif _type == 'UserInteractionAction':
                    action_from_item = UserInteractionAction(node=self.node,
                                                            interaction_mode = action_dict['interaction_mode'],
                                                            name=_name,
                                                            action_text=action_dict['action_text'])
                    action_from_item.set_description(_description)
                    self.sequence_list.append(action_from_item)
                    
                elif _type == 'RosActionAction':
                    _ros_action_client = action_dict["ros_action_client"]
                    _ros_action_type = action_dict["ros_action_type"]
                    action_from_item = RosActionAction(node = self.node,
                                                    client = _ros_action_client,
                                                    action_type = _ros_action_type,
                                                    name = _name)
                    
                    action_from_item.set_description(_description)
                    action_from_item.set_request_from_dict(_request)
                    self.sequence_list.append(action_from_item)

                else:
                    self.node.get_logger().error(f"Action type '{_type}' not supported!")
                    continue
                
            except (ActionInitializationError,SetActionRequestError) as e:
                self.node.get_logger().error(f"{e}. Skipping loading this action")
            
        self.node.get_logger().error(f"SUCCEEESS!")
        return True

    def save_to_JSON(self) -> bool:
        """
        Saves the action_list to a json-file.
        In order to execute the function successfully, a valid path and the action_list name must be set.
        """

        if (self._folder_path is not None) and (self._sequence_name is not None):
            self._sequence_file_path = f"{self._folder_path}/{self._sequence_name}.json"
            process_dict = {}
            process_dict["name"] = self._sequence_name
            process_dict["saved_at"] = str(datetime.now())  # Add a timestamp

            action_list = []
            for index, action in enumerate(self.sequence_list):
                action_dict = {}
                                
                action_dict["name"] = action.get_name()
                action_dict["description"] = action.get_description()
                self.node.get_logger().error(f'desc{action_dict["description"]}')

                action_dict["action_type"] = action.get_type_indicator()
                action_dict["action_position"] = index

                if isinstance(action, ServiceAction):
                    action: ServiceAction
                    action_dict["service_client"] = action.client
                    action_dict["service_type"] = action.service_type
                    action_dict["error_identifier"] = action.get_success_identifier()
                    action_dict["request"] = json.loads(json.dumps(action.get_request_as_ordered_dict()))
                
                elif isinstance(action, UserInteractionAction):
                    action: UserInteractionAction
                    action_dict["action_text"] = action.request.interaction_text
                    action_dict["interaction_mode"] = action.interaction_mode
                    action_dict["request"] = json.loads(json.dumps(action.get_request_as_ordered_dict()))

                elif isinstance(action, RosActionAction):
                    action: RosActionAction
                    action_dict["ros_action_client"] = action.client
                    action_dict["ros_action_type"] = action.action_type
                    action_dict["error_identifier"] = action.get_success_identifier()
                    action_dict["request"] = json.loads(json.dumps(action.get_request_as_ordered_dict()))
                    
                action_dict["has_breakpoint"] = action.has_breakpoint()
                action_dict["is_active"] = action.is_active()
                
                if action_dict:
                    action_list.append(action_dict)

            process_dict["action_list"] = action_list
            try:
                with open(f"{self._folder_path}/{self._sequence_name}.json", "w") as json_file:
                    json.dump(process_dict, json_file,indent=4)
                self.node.get_logger().info("Saved!")
                return True
            except FileNotFoundError as e:
                self.node.get_logger().error(f"{e}")
                self.node.get_logger().error("Directory does not exist!")
                return False
            except Exception as e:
                self.node.get_logger().error(f"{e}")
                self.node.get_logger().error("Saving file failed!")
                return False
        else:
            return False
        
        
