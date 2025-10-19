import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_interface
from ament_index_python.packages import get_package_share_directory
from rqt_py_common import message_helpers
from rcl_interfaces.msg import Log

from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import RosActionAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import UserInteractionAction, GUI, TERMINAL
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError
from ros_sequential_action_programmer.submodules.rsap_modules.RsapFileManager import RsapFileManager
from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import RsapConfig
from ros_sequential_action_programmer.submodules.RsapApp_submodules.rsap_signals import IncomingLogSignal, ExecutionStatusSignal, CurrentActionSignal
from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import ActionParameterValueManager

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import ExecutionLog
import json
from datetime import datetime
import os
import copy
import collections
from collections import OrderedDict
from typing import Tuple
import yaml
from yaml.loader import SafeLoader
from ros_sequential_action_programmer.submodules.obj_dict_modules.dict_functions import convert_to_ordered_dict, get_key_value_pairs_from_dict
from typing import Union
import subprocess
import time

CHECK = 0
SET = 1

CHECK_COMPATIBILITY_ONLY = 0
SET_IMPLICIT_SRV_DICT = 1
SET_SRV_DICT = 2

class RosSequentialActionProgrammer:
    def __init__(self, node: Node) -> None:
        self.action_list: list[ServiceAction,UserInteractionAction] = []
        self.node = node
        self.current_action_index = 0
        self.current_action = None

        self.action_parameter_value_manager = ActionParameterValueManager(self.action_list, 
                                                                          self.node)
        
        self.rsap_file_manager = RsapFileManager(self.action_list, 
                                                 self.node,
                                                 self.action_parameter_value_manager.seq_parameter_manager)
        
        self.list_of_active_services = None
        self.list_of_active_clients = None
        self._list_of_service_types = None
        self.callback_group_reentrant = ReentrantCallbackGroup()
        self.callback_group_exclusive = MutuallyExclusiveCallbackGroup()
        self.initialize_service_list()
        self.initialize_ros_action_list()
        self.action_sequence_log = {}
        self.action_log = []
        self.config = RsapConfig('ros_sequential_action_programmer', self.node.get_logger())
        self.log_subscription = self.node.create_subscription(Log, "/rosout", self.log_callback, 10, callback_group=self.callback_group_reentrant)
        self._init_signals()
        self._stop_execution = False
        self._interupt_execution = False

    def get_interrupt_execution(self)->bool:
        return self._interupt_execution
    
    def set_interrupt_execution(self, value: bool):
        self._interupt_execution = value

    def initialize_service_list(self):
        """
        Initialize list of active services.
        """
        self.list_of_active_services = self.node.get_service_names_and_types()
        self.list_of_active_clients = [item[0] for item in self.list_of_active_services]
        self._list_of_service_types = [item[1] for item in self.list_of_active_services]

    def _init_signals(self):
        self.signal_incoming_log = IncomingLogSignal()
        self.signal_execution_status= ExecutionStatusSignal()
        self.signal_current_action = CurrentActionSignal()

    def log_callback(self, msg: Log):
        """
        Callback for log messages.
        """
        
        app_log = f"[{msg.name}]: {msg.msg}"
    
        match msg.level:
            case 10:
                if self.config.ros_log_levels.get_log_debug():
                    self.signal_incoming_log.signal.emit(app_log,10)
            case 20:
                if self.config.ros_log_levels.get_log_info():
                    self.signal_incoming_log.signal.emit(app_log,20)
            case 30:
                if self.config.ros_log_levels.get_log_warn():
                    self.signal_incoming_log.signal.emit(app_log,30)
            case 40:
                if self.config.ros_log_levels.get_log_error():
                    self.signal_incoming_log.signal.emit(app_log,40)
                    

    def initialize_ros_action_list(self):
        self.list_of_active_ros_actions = rclpy.action.get_action_names_and_types(self.node)
        self.list_of_active_ros_action_clients = [item[0] for item in self.list_of_active_ros_actions]
        self._list_of_ros_action_types = [item[1] for item in self.list_of_active_ros_actions]
        
    def get_active_services(self)->list[Tuple[str, list[str]]]:
        """
        Returns a list of active services as a tuple of client and service type.
        """
        return self.list_of_active_services

    def get_active_ros_actions(self)->list[Tuple[str, list[str]]]:
        """
        Returns a list of active ros actions as a tuple of client and service type.
        """
        return self.list_of_active_ros_actions
    
    def get_service_type_for_client(self, service_client) -> str:
        """
        Returns the service type of a clients as a string.
        """
        for index, client in enumerate(self.list_of_active_clients):
            if client == service_client:
                return self._list_of_service_types[index][0]
        return None

    def append_service_to_action_list(self, service_client: str, service_type: str = None, service_name=None) -> bool:
        """
        This function appends a service to the action_list.
        :param service_client: The name of the service client.
        :param service_type: The type of the service. If not given, the function will try to get the service type from the service client.
        :param service_name: The name of the service. If not given, the function will use the service_client as name.

        :return: True if the service was appended successfully, False otherwise.
        """
        index = len(self.action_list)-1
        if index < 0:
            index = 0

        return self.append_service_to_action_list_at_index(service_client=service_client,
                                                        service_type=service_type,
                                                        service_name=service_name,
                                                        index=index)

    def append_service_to_action_list_at_index(
        self,
        service_client: str,
        index: int,
        service_type: str = None,
        service_name=None) -> bool:
        """
        This function appends a service to the action_list at a given index.
        :param service_client: The name of the service client.
        :param index: The index where the service should be appended.
        :param service_type: The type of the service. If not given, the function will try to get the service type from the service client.
        :param service_name: The name of the service. If not given, the function will use the service_client as name.
        
        :return: True if the service was appended successfully, False otherwise.
        """

        if not service_type:
            service_type = self.get_service_type_for_client(service_client)

        if not index > len(self.action_list):
            try:
                new_action = ServiceAction(
                        node=self.node,
                        client=service_client,
                        service_type=service_type,
                        name=service_name)
                
                self.action_list.insert(index, new_action)
                self.current_action_index = index
                self.node.get_logger().info(f"Inserted Service for {service_client} at {index}")
                return True
            
            except ActionInitializationError as e:
                self.node.get_logger().error(f"Service client: '{service_client}' with srv_type '{service_type}' could not be appended! {e}")
                return False


    def append_ros_action_to_action_list_at_index(
        self,
        action_client: str,
        index: int,
        action_type: str = None,
        action_name=None) -> bool:
        
        if not action_type:
            #action_type = self.get_action_type_for_client(action_client)
            pass

        if not index > len(self.action_list):
            try:
                new_action = RosActionAction(
                        node=self.node,
                        client=action_client,
                        action_type=action_type,
                        name=action_name)
                
                self.action_list.insert(index, new_action)

                self.current_action_index = index
                self.node.get_logger().info(f"Inserted Ros Action for {action_client} at {index}")
                return True

            except ActionInitializationError as e:
                self.node.get_logger().error(f"Service client: '{service_client}' with srv_type '{service_type}' could not be appended! {e}")
                return False
            
    def append_user_interaction_to_action_list_at_index(
        self,
        index: int,
        action_name: str,
        action_description: str,
        interaction_mode = TERMINAL) -> bool:
        """
        This function appends a user interaction to the action_list at a given index.
        :param index: The index where the user interaction should be appended.
        :param action_name: The name of the user interaction.
        :param action_description: The description of the user interaction.
        :param interaction_mode: The mode of the user interaction. Can be either TERMINAL or GUI.
        
        :return: True if the user interaction was appended successfully, False otherwise.
        """
        if not index > len(self.action_list):
            new_action = UserInteractionAction(
                    node=self.node,
                    name=action_name,
                    action_text=action_description,
                    interaction_mode=interaction_mode)
            
            self.action_list.insert(index, new_action)
            self.current_action_index = index
            
            self.node.get_logger().info(f"Inserted Service for {action_name} at {index}")
            return True
        else:
            return False
        
    def get_action_at_index(self, index: int) -> Union[ServiceAction, UserInteractionAction]:
        """
        Returns the action at the index from the action_list.
        :param index: The index of the action.

        :return: The action at the index.
        """
        if not index > len(self.action_list):
            return self.action_list[index]
        else:
            return None

    def delete_action_at_index(self, index: int) -> bool:
        """
        Deletes the action at the index from the action_list.
        :param index: The index of the action.
        """
        if not index > len(self.action_list):
            del self.action_list[index]
            return True
        else:
            return False
        
    def delete_actions_at_indexes(self, index_list: list[int]) -> bool:
        """
        Deletes the actions at the indexes from the action_list.
        :param index_list: The list of indexes of the actions.
        """
        # sort the index list in descending order
        index_list.sort(reverse=True)
        for index in index_list:
            if not index > len(self.action_list):
                del self.action_list[index]
            else:
                return False
        return True

    def set_current_action(self, index: int) -> bool:
        """
        Sets the current action to the index from input.
        :param index: The index of the action to be set.
        :return: True if the action was set successfully, False otherwise.
        """
        if index < len(self.action_list):
            self.current_action_index = index
            return True
        else:
            return False

    def get_current_action_name(self) -> str:
        return self.get_action_at_index(self.current_action_index).get_name()

    def execute_current_action(self, log_mode: int = ExecutionLog.LOG_NEVER, shift_action:bool = False) -> bool:
        """
        This function executes the current action. It returns a bool value for success indication.
        :param log_mode: The mode of the log. Can be either LOG_NEVER or LOG_AT_END.
        """

        self.signal_execution_status.signal.emit(True)
        # set the interrupt flag to False
        self.set_interrupt_execution(False)
        try:
            # Set values from earlier service respones to this service request, might fail, if earlier call has not been executed
            
            current_action = self.get_action_at_index(self.current_action_index)
            
            #if isinstance(current_action, ServiceAction):
                # set_success = self.process_action_dict_at_index(self.current_action_index, SET_SRV_DICT)
                # if not set_success:
                #     raise Exception

            if not current_action.is_active():
                self.node.get_logger().info(f"Action {self.get_current_action_name()} is not active!")
                if shift_action:
                    self.shift_to_next_action()
                return True
            
            success_exec = current_action.execute(self.get_interrupt_execution)    # pass interrupt method to check if interruption is demanded
            # append action log to history
            self.append_action_log(
                index=self.current_action_index,
                action_name=self.get_current_action_name(),
                action_log=self.get_current_action_log(),
            )

            if (self.get_current_action_index() == len(self.action_list) - 1) and log_mode == ExecutionLog.LOG_AT_END:
                self.save_action_sequence_log()

            if log_mode == ExecutionLog.LOG_ALWAYS:
                self.save_action_sequence_log()
            
            if success_exec:
                self.node.get_logger().info(
                    f"Action {self.get_current_action_name()} executed successfully!"
                )
            else:
                raise Exception("Error executing action!")

            if shift_action:
                self.shift_to_next_action()
                
            return success_exec

        except Exception as e:
            self.node.get_logger().error(f"Error {e}!")           
            self.node.get_logger().error(f"Error executing {self.get_current_action_name()}!")
            return False
        finally:
            self.signal_execution_status.signal.emit(False)

    def execute_action_list(self, index_start: int, log_mode:int =ExecutionLog.LOG_NEVER) -> Tuple[bool, int]:
        """
        This function executes the action_list successifly starting at the start_index.
        It returns bool value for success indication and also the index of the action when the method terminates.
        :param index_start: The index of the action to start with.
        :param log_mode: The mode of the log. Can be either LOG_NEVER or LOG_AT_END.
        :return: Tuple of bool and int. The bool value indicates success, the int value indicates the index of the action when the method terminates.
        """
        
        self.clear_all_log_entries()
        self._stop_execution = False
        if index_start < len(self.action_list):
            ind = index_start
            
            # This is needed if the start action has a breakpoint
            breakpoint_override = False
            if self.has_current_action_breakpoint():
                breakpoint_override = True
                
            while ind < len(self.action_list) or self.config.execution_behavior.get_value():

                # this is need for looping
                if self.config.execution_behavior.get_value() and ind == len(self.action_list):
                    ind = 0

                self.set_current_action(ind)
                self.signal_current_action.signal.emit(ind)
                
                if self.has_current_action_breakpoint() and not breakpoint_override:
                    self.node.get_logger().info(f"Breakpoint at {self.get_current_action_name()}!")
                    return True, self.current_action_index
                
                success = self.execute_current_action(log_mode=log_mode)
                if not success:
                    return False, self.current_action_index
                # check if stop_execution flag is set
                if self._stop_execution:
                    return True, self.current_action_index
                
                # reset breakpoint_override after first action
                breakpoint_override = False
                ind += 1
                time.sleep(0.5)

            # for ind in range(index_start, len(self.action_list)):
            #     self.set_current_action(ind)
            #     self.signal_current_action.signal.emit(ind)
            #     success = self.execute_current_action(log_mode=log_mode)
            #     if not success:
            #         return False, self.current_action_index
            #     # check if stop_execution flag is set
            #     if self._stop_execution:
            #         return True, self.current_action_index

            self.set_current_action(0)
            return True, self.current_action_index
        else:
            return False, self.current_action_index


    def load_recent_file(self):
        """
        Loads the last opened file from the yaml file.
        """
        path = get_package_share_directory('ros_sequential_action_programmer')

        # Specify the path to your YAML file
        yaml_file_path = f"{path}/recent_file.yaml"
        try:
            # Read the content of the YAML file
            with open(yaml_file_path, 'r') as file:
                yaml_content = yaml.safe_load(file)

            process_file_path = yaml_content['recent_file'] 
            self.rsap_file_manager.load_from_JSON(process_file_path)
        except Exception as e:
            self.node.get_logger().error(f"Error loading recent file: {e}")
            self.node.get_logger().warn("No recent file found! Skipping loading of recent file!")

    def has_current_action_breakpoint(self) -> bool:
        """
        Returns a bool value if the current action has a breakpoint.
        """
        return self.get_action_at_index(self.current_action_index).has_breakpoint()
    
    def get_service_responses_list(self) -> list[collections.OrderedDict]:
        """
        Returns a list that contains all the service responses from the action_list in an ordered dict.
        """

        def update_identifier(data, identifier_list: list, parent_key=None):
            # iterate through the dict
            for key, value in data.items():
                if parent_key is not None:
                    full_key = parent_key + "." + key
                else:
                    full_key = key

                if isinstance(value, collections.OrderedDict):
                    update_identifier(value, identifier_list, full_key)
                else:
                    identifier_list.append(full_key)

        service_response_list = []
        service_response_dict_list = []
        # process_dict = collections.OrderedDict()
        for index, action in enumerate(self.action_list):
            if isinstance(action, ServiceAction):
                service_response_list = []
                dictionary = {}
                process_dict = collections.OrderedDict()
                action_response_dict = message_to_ordereddict(
                    action.get_empthy_service_response()
                )
                # append client name to dict
                process_dict[action.client] = action_response_dict

                update_identifier(process_dict, service_response_list)
                dictionary[f"{index}.{action.name}"] = service_response_list
                service_response_dict_list.append(dictionary)

        return service_response_dict_list

    def get_active_client_whtlist(self)-> list:
        """
        This function returns only clients, that are listed in the whitelist.yaml
        """
        self.initialize_service_list()
        client_list = [t[0] for t in self.list_of_active_services]
        service_type_list = [t[1][0] for t in self.list_of_active_services]

        return self.get_client_whitelist(clients_list = client_list, service_list = service_type_list)

    def get_active_client_blklist(self)-> list:
        """
        This function returns only clients, that are not listed in the blacklist.yaml
        """
        self.initialize_service_list()
        client_list = [t[0] for t in self.list_of_active_services]
        service_type_list = [t[1][0] for t in self.list_of_active_services]
        return self.get_client_blacklist(clients_list = client_list, service_list = service_type_list)

    def get_memorized_client_blklist(self)-> list:
        """
        This function returns only clients, that are not listed in the blacklist.yaml
        """
        self.save_all_service_req_res_to_JSON()

        client_list = [t[0] for t in self.list_of_memorized_services]
        service_type_list = [t[1] for t in self.list_of_memorized_services]
        return self.get_client_blacklist(clients_list = client_list, service_list = service_type_list)
    
    def get_memorized_client_whitelist(self)-> list:
        """
        This function returns only clients, that are not listed in the whitelist.yaml
        """
        client_list = [t[0] for t in self.list_of_memorized_services]
        service_type_list = [t[1] for t in self.list_of_memorized_services]
        return self.get_client_whitelist(clients_list = client_list, service_list = service_type_list)
    
    def get_client_whitelist(self, clients_list: list, service_list: list)->list:
        """
        This function returns only clients, that are listed in the whitelist.yaml
        """
        client_whitelisted = []
        try:
            package_share_directory = get_package_share_directory('ros_sequential_action_programmer')
            whitelist_path = package_share_directory + '/whitelist.yaml'
            with open(whitelist_path, 'r') as file:
                FileData = yaml.safe_load(file)
                list_of_wht_clients = FileData['clients_by_name']
                list_of_wht_types = FileData['clients_by_type']

            # If list in yaml empty list will be None -> set to empty
            if not list_of_wht_clients:
                list_of_wht_clients = []
            if not list_of_wht_types:
                list_of_wht_types= []
            
            client_whitelisted = [s for s in clients_list if any(client in s for client in list_of_wht_clients)]

            return client_whitelisted
        except Exception as e:
            self.node.get_logger().error(e)
            self.node.get_logger().error(f"Error opening whitelist. Check formatting of '{whitelist_path}'!")

    def get_client_blacklist(self, clients_list: list, service_list: list)->list:
        client_blacklisted = []
        try:
            package_share_directory = get_package_share_directory('ros_sequential_action_programmer')
            blacklist_path = package_share_directory + '/blacklist.yaml'
            with open(blacklist_path, 'r') as file:
                FileData = yaml.safe_load(file)
                list_of_blk_clients = FileData['clients_by_name']
                list_of_blk_types = FileData['clients_by_type']

            # If list in yaml empty list will be None -> set to empty
            if not list_of_blk_clients:
                list_of_blk_clients = []
            if not list_of_blk_types:
                list_of_blk_types = []
            
            for index, client in enumerate(clients_list):
                if not client in list_of_blk_clients and not service_list[index] in list_of_blk_types:
                    client_blacklisted.append(client)
            return client_blacklisted
        except Exception as e:
            self.node.get_logger().error(e)
            self.node.get_logger().error(f"Error opening blacklist. Check formatting of '{blacklist_path}'!")

    def get_all_service_req_res_dict(self, list_of_clients):
        """
        This function returns all dict of service request and responses. 
        !!Changes!!: list_of_client as input
        """
        self.initialize_service_list()
        service_dict = {}
        for client in list_of_clients:
            try:
                request = ServiceAction.get_service_request(
                    self.get_service_type_for_client(client)
                )
                request_dict = json.loads(json.dumps(message_to_ordereddict(request)))

                response = ServiceAction.get_service_response(
                    self.get_service_type_for_client(client)
                )
                response_dict = json.loads(json.dumps(message_to_ordereddict(response)))

                service_dict[client] = {
                    "service_type": self.get_service_type_for_client(client),
                    "request": request_dict,
                    "response": response_dict,
                }
            except:
                # self.node.get_logger().warn(f"Skipped: {client}")
                self.node.get_logger().debug(f"Skipped: {client}")
        return service_dict

    def save_all_service_req_res_to_JSON(self) -> bool:
        """
        This function saves all the available service request dict and service response dict to a json file.
        If the file exists, it appends it.
        """

        def merge_dicts(dict1: dict, dict2: dict)->dict:
            result_dict = dict1.copy()
            for key, value in dict2.items():
                if key not in result_dict:
                    result_dict[key] = value
            return result_dict

        merged_dict = {}
        data = self.get_all_service_req_res_dict(self.list_of_active_clients)
        path = get_package_share_directory("ros_sequential_action_programmer")
        file_name = "all_service_req_res_dicts.json"
        file_path = f"{path}/{file_name}"
        try:
            with open(file_path, "r") as json_file:
                file_data = json.load(json_file)

            merged_dict = merge_dicts(data, file_data)
        except Exception:
            self.node.get_logger().warn("No history of service clients found! Creating a new one!")
            pass
        try:
            with open(file_path, "w") as json_file:
                json.dump(merged_dict, json_file)
            self.node.get_logger().debug("Json with all service requests and responses exported!")

            self.list_of_memorized_services =[]
            for key, value in merged_dict.items():
                tuple_val=(key,value['service_type'])
                self.list_of_memorized_services.append(tuple_val)

            return True
        except FileNotFoundError as e:
            print(e)
            self.node.get_logger().error("Directory does not exist!")
            return False
        except Exception as e:
            print(e)
            self.node.get_logger().error("Saving file failed!")
            return False

    def get_list_memorized_service_clients(self)->list:
        """
        Returns a list of all memorized service clients.
        """
        self.save_all_service_req_res_to_JSON()
        return [t[0] for t in self.list_of_memorized_services]
        
    def list_of_clients_to_dict(self, list_of_clients: list)->dict:
        """
        This function returns a dictionary of clients and their services.
        """
        result_dict = {}
        for client in list_of_clients:
            parts = client.split("/", 2)
            if len(parts) >= 2:
                client_node = f"/{parts[1]}"
                client_service = "/".join(parts[1:])
                client_service = f"/{client_service}"
                if client_node not in result_dict:
                    result_dict[client_node] = []
                result_dict[client_node].append(client_service)
            else:
                # Handle cases where there are not enough parts
                print(f"Unexpected format for client: {client}")
        return result_dict
    
    def get_srv_type_from_memorized_client(self, client:str)->str:
        for clt, service_type in self.list_of_memorized_services:
            if client == clt:
                return service_type
        return None

    def check_for_reference(self, input_value: any) -> tuple[bool, bool, int, str]:
        """
        This function takes an input value as input. The function checks if the value conatins an reference to another action value.
        If yes, it returns 
        1. True/False for if it contains a reference, 
        2. if a given refence contains erros,
        3. if a ref the index of the action in the action plan 
        4. the key of the value of the targeted action.
        """
        ref_index = None
        ref_key = None
        error_in_reference = False
        if not isinstance(input_value, str):
            return False, error_in_reference, ref_index, ref_key

        identifier = input_value.split(".")
        #self.node.get_logger().warn(str(identifier))
        if identifier[0] == "service_response":
            ref_index = int(identifier[1].split("-")[0])
            ref_action_name = str(identifier[1].split("-")[1])
            if (ref_action_name != self.get_action_at_index(ref_index).name):
                error_in_reference = True
                self.node.get_logger().error(f"Action name {ref_action_name} or index {ref_index} does not match!")
                return False, error_in_reference, ref_index, ref_key

            ref_key = ".".join(identifier[2:])
            return True, error_in_reference, ref_index, ref_key
        else:
            return False, error_in_reference, ref_index, ref_key

    def get_current_action_log(self) -> dict:
        """
        Returns the log entry of the current action. If the action has not been executed yet, it will return an empty dict.
        """
        return self.get_action_at_index(self.current_action_index).get_log_entry()

    def get_current_action_index(self) -> int:
        """
        Returns the index of the current action.
        """
        return self.current_action_index

    # Log - Functions
    def append_action_log(self, index: int, action_name: str, action_log) -> None:
        """
        Appends the given action log to the actions log.
        """
        key = f"{index}_{action_name}"
        self.action_log.append({key: action_log})

    def _update_action_sequence_log(self):
        """
        Updates the action sequence log with the current action sequence log.
        """
        self.action_sequence_log["action_sequence_name"] = self.rsap_file_manager.get_sequence_name()
        self.action_sequence_log["action_log"] = self.action_log

    def copy_action_at_index_and_insert(self, index: int = None) -> bool:
        """
        This function copies the action at index and inserts it behind the action at the index.
        """
        if index is None:
            index = self.current_action_index

        if index < len(self.action_list):
            new_action = copy.deepcopy(self.action_list[index])
            self.action_list.insert(index + 1, new_action)
            return True
        else:
            return False
        
    def copy_actions_from_index_list_and_insert(self, index_list: list[int] = None) -> bool:
        """
        This function copies the actions at indexes specified in the list and inserts it behind the last action of the indexes.
        If no list is given, the current action index will be used.
        """
        # log the index_list

        if index_list is None:
            index = self.current_action_index

        # sort the index list
        index_list.sort()
        insert_index = max(index_list)

        for index in index_list:
            if index < len(self.action_list):
                new_action = copy.deepcopy(self.action_list[index])
                self.action_list.insert(insert_index + 1, new_action)
                insert_index += 1
            else:
                return False
            
        return True
    
        # if index < len(self.action_list):
        #     new_action = copy.deepcopy(self.action_list[index])
        #     self.action_list.insert(index + 1, new_action)
        #     return True
        # else:
        #     return False
    
    def set_stop_execution(self) -> None:
        self._stop_execution = True

    def save_action_sequence_log(self) -> bool:
        """
        Saves the action sequence log to the folder.
        """
        if (self._folder_path is not None) and (self.rsap_file_manager.get_sequence_name() is not None):
            self._update_action_sequence_log()
            export_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            try:
                file_path = f"{self._folder_path}/logs/"
                if not os.path.exists(file_path):
                    os.makedirs(file_path)
                with open(
                    f"{file_path}/{self.rsap_file_manager.get_sequence_name()}_log_{str(export_time)}.json", "w"
                ) as json_file:
                    json.dump(self.action_sequence_log, json_file,indent=4)
                self.node.get_logger().info("Action sequence log saved!")
                self.action_log.clear()
                return True
            except FileNotFoundError as e:
                print(e)
                self.node.get_logger().error("Directory does not exist!")
                return False
            except Exception as e:
                print(e)
                self.node.get_logger().error("Saving action sequence log failed!")
                return False
        else:
            return False

    def move_action_at_index_to_index(self, old_index: int,new_index:int) -> bool:
        """
        This function moves the action at old_index to new_index.
        :param old_index: The index of the action to be moved.
        :param new_index: The index where the action should be moved to.
        :return: True if the action was moved successfully, False otherwise.
        """
        if ((old_index >= 0) and 
        (old_index <= len(self.action_list)) and 
        (old_index != new_index) and
        (new_index >= 0) and 
        (new_index <= len(self.action_list))):
            element_to_move = self.action_list.pop(old_index)
            self.action_list.insert(new_index,element_to_move)
            return True
        else:
            self.node.get_logger().error("Invalid indicies given. Out of bounds")
            return False

    def shift_to_next_action(self):
        """
        This function shifts the current action to the next action.
        """
        if self.current_action_index < len(self.action_list) - 1:
            self.current_action_index += 1
        else:
            self.current_action_index = 0
    
    def clear_all_log_entries(self):
        """
        This function clears the log entries.
        """
        for action in self.action_list:
            action.clear_log_entry()

    
    @staticmethod
    def get_ros2_executables():
        result = subprocess.run(["ros2", "pkg", "executables"], capture_output=True, text=True)
        executables = result.stdout.strip().split("\n")
        executables_list = []
        current_package = None
        reslt_dict = {}
        for line in executables:
           
            if not line:
                continue
            
            package, executable = line.split()
            
            print(f"Package: {package}")
            print(f"Executable: {executable}")
            
            if current_package != package:
                if current_package is not None:
                    reslt_dict[current_package] = copy.copy(executables_list)
                    
                current_package = package
                executables_list.clear()
                executables_list.append(executable)
            else:
                executables_list.append(executable)

        return reslt_dict
    
    @staticmethod
        
    def get_ros2_launch_executables():
        try:
            package_share_directory = get_package_share_directory('ros_sequential_action_programmer')
            launch_file_path = package_share_directory + '/launch_files.yaml'
            
            with open(launch_file_path, 'r') as file:
                FileData = yaml.safe_load(file)
                                
            result_dict = FileData
            
            return FileData
            
        except Exception as e:
            print(print(e))
            result_dict = None
    
    @staticmethod
    def run_ros2_executable(package: str, 
                            executable: str, 
                            terminal: str = "gnome-terminal",
                            launch_mode: bool = False):
        """
        Runs a ROS 2 executable in a new terminal window.

        Args:
            package (str): The name of the ROS 2 package.
            executable (str): The name of the executable within the package.
            terminal (str, optional): The terminal emulator to use (default: "gnome-terminal").
        """
        if launch_mode:
            command = f"ros2 launch {package} {executable}"
        else:
            command = f"ros2 run {package} {executable}"
        
        if terminal == "gnome-terminal":
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])
        elif terminal == "x-terminal-emulator":
            subprocess.Popen(["x-terminal-emulator", "-e", f"bash -c '{command}; exec bash'"])
        elif terminal == "konsole":
            subprocess.Popen(["konsole", "-e", f"bash -c '{command}; exec bash'"])
        elif terminal == "xterm":
            subprocess.Popen(["xterm", "-e", f"bash -c '{command}; exec bash'"])
        else:
            raise ValueError(f"Unsupported terminal: {terminal}")

        return command

    
if __name__ == "__main__":
    res = RosSequentialActionProgrammer.get_ros2_launch_executables()
    print(res)
