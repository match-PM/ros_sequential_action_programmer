
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI, TERMINAL
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import ActionResponseParameterReference, SeqParameterReference
from ros_sequential_action_programmer.submodules.action_classes.ParameterReferences import ActionParameterReferences
from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameterManager, SeqParameter, SeqParameterError, SeqParameterManagerError
import json
import os
from datetime import datetime
from rclpy.node import Node

class RsapFileManager():
    def __init__(self, 
                 sequence_list: list[ActionBaseClass],
                 node:Node,
                 seq_parameter_manager: SeqParameterManager = None):
        
        self.sequence_list = sequence_list
        self.node = node
        self.seq_parameter_manager = seq_parameter_manager
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

        _sequence_file_path = file_data.get("seq_parameters_file", None)

        # its important to load the seq parameters first
        if _sequence_file_path and self.seq_parameter_manager:
            try:
                self.seq_parameter_manager.load_file(_sequence_file_path)
                self.node.get_logger().info(f"Success loading sequence parameters from '{_sequence_file_path}'!")
            except SeqParameterManagerError as e:
                self.node.get_logger().error(f"Error loading sequence parameters from '{_sequence_file_path}': {e}")
                self.node.get_logger().error(f"No sequence parameters loaded!")


        for index, action_dict in enumerate(file_data["action_list"]):
            try:
                action_dict:dict
                _request = action_dict.get("request",{})
                if _request == {}:
                    _request = action_dict.get("service_request",{})
                _description = action_dict["description"]
                _name = action_dict["name"]
                _type = action_dict['action_type']
                _param_references_dict = action_dict.get('parameter_references', None)
                _param_references = ActionParameterReferences()

                if _param_references_dict:
                    for ref in _param_references_dict:
                        if ref["type"] == "action_response":
                            action_pointer = None
                            # find the action pointer from the current sequence list
                            action_pointer = self.sequence_list[ref["action_index"]] if ref["action_index"] < len(self.sequence_list) else None

                            _new_ref = ActionResponseParameterReference(
                                value_key=ref["value_key"],
                                reference_key=ref["reference_key"],
                                action_pointer=action_pointer
                            )
                            _param_references.add_reference(_new_ref)
                        elif ref["type"] == "seq_parameter":
                            try:
                                param = self.seq_parameter_manager.get_parameter_by_name(ref["parameter_name"])
                                if param:
                                    _new_ref = SeqParameterReference(
                                        value_key=ref["value_key"],
                                        param=param
                                    )
                                    _param_references.add_reference(_new_ref)
                            except SeqParameterError as e:
                                self.node.get_logger().error(f"Error loading SeqParameter reference '{ref['parameter_name']}': {e}")
                                continue
                        else:
                            self.node.get_logger().error(f"Unknown parameter reference type '{ref['type']}'!")
                            continue

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

                elif _type == 'UserInteractionAction':
                    action_from_item = UserInteractionAction(node=self.node,
                                                            interaction_mode = action_dict['interaction_mode'],
                                                            name=_name,
                                                            action_text=action_dict['action_text'])
                    action_from_item.set_description(_description)
                    
                elif _type == 'RosActionAction':
                    _ros_action_client = action_dict["ros_action_client"]
                    _ros_action_type = action_dict["ros_action_type"]
                    action_from_item = RosActionAction(node = self.node,
                                                    client = _ros_action_client,
                                                    action_type = _ros_action_type,
                                                    name = _name)
                    
                    action_from_item.set_description(_description)
                    action_from_item.set_request_from_dict(_request)

                else:
                    self.node.get_logger().error(f"Action type '{_type}' not supported!")
                    continue

                action_from_item.set_references(_param_references)
                self.sequence_list.append(action_from_item)

                
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
            if self.seq_parameter_manager:
                process_dict["seq_parameters_file"] = self.seq_parameter_manager.get_file_path()

            action_list = []
            for index, action in enumerate(self.sequence_list):
                action_dict = {}
                                
                action_dict["name"] = action.get_name()
                action_dict["description"] = action.get_description()
                self.node.get_logger().error(f'desc{action_dict["description"]}')

                action_dict["action_type"] = action.get_type_indicator()
                action_dict["action_position"] = index
                action_dict["parameter_references"] = self.references_as_dict_list(action=action)

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

                # save the seq parameters as well if available
                if self.seq_parameter_manager and self.seq_parameter_manager.get_is_initialized():
                    self.seq_parameter_manager.save_to_file()
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
        
    def references_as_dict_list(self, action:ActionBaseClass)->list[dict]:
        dict_list :list[dict] = []
        for index, ref in enumerate(action.get_references().reference_list):
            if isinstance(ref, ActionResponseParameterReference):
                ref_action = ref.get_reference_action()
                ref_action_name = ref_action.get_name() if ref_action else "None"
                ref_index = self._get_index_of_action(ref_action) if ref_action else None
                ref: ActionResponseParameterReference
                dict_list.append({
                    "type": "action_response",
                    "value_key": ref.get_value_key(),
                    "reference_key": ref.get_reference_key(),
                    "action_name": ref_action_name,
                    "action_index": ref_index,
                })
            elif isinstance(ref, SeqParameterReference):
                ref: SeqParameterReference
                dict_list.append({
                    "type": "seq_parameter",
                    "value_key": ref.get_value_key(),
                    "parameter_name": ref.get_parameter().get_name()
                })

        return dict_list
    
    def _get_index_of_action(self, action:ActionBaseClass)->int|None:
        for index, act in enumerate(self.sequence_list):
            if act is action:
                return index
        return None
    
    def reset_sequence_parameter_manager(self) -> None:
        self.seq_parameter_manager = SeqParameterManager()