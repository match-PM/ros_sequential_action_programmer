import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_interface
from rosidl_runtime_py.get_interfaces import get_service_interfaces
from rqt_py_common import message_helpers
import collections
import copy
import json
from datetime import datetime
import array
import numpy as np
from functools import partial
from ros_sequential_action_programmer.submodules.obj_dict_modules.dict_functions import flatten_dict_to_list, is_string_in_dict
from typing import Union
import re
from typing import Tuple, Any


class ServiceAction:
    def __init__(self, node: Node, client: str, service_type: str = None, name=None) -> None:
        self.client = client
        self.service_type = service_type
        self.node = node
        self.service_request = None
        self.service_req_dict = None
        self.service_metaclass = None

        self.default_service_res_dict = None
        self.service_response = None
        self.service_res_dict = None
        self.empty_service_response = None

        self.service_res_bool_messages = []
        self.service_success_key = None

        self.service_req_dict_implicit = None
        self.log_entry = {}

        self.description = ""
        if name is None:
            self.name = self.client
        else:
            self.name = name

        self.valid = self.check_for_valid_inputs()

        if not self.valid:
            return None
        
        self.init_service()
        self.init_service_res_bool_messages()

    def get_init_success(self)-> bool:
        return self.valid
    
    def get_action_name(self)-> str:
        return self.name
    
    def set_action_name(self, new_name:str) -> bool:
        try:
            self.name = new_name
            return True
        except Exception as e:
            self.node.get_logger().error(str(e))
            return False
    
    def check_for_valid_inputs(self) -> bool:
        list_of_active_services = self.node.get_service_names_and_types()
        list_of_active_clients = [item[0] for item in list_of_active_services]
        list_of_service_types = [item[1][0] for item in list_of_active_services]
        service_interfaces = flatten_dict_to_list(get_service_interfaces())

        if self.service_type is not None:
            if not self.service_type in service_interfaces:
                self.node.get_logger().fatal(f"Service Action could not be instantiated. Serivce type '{self.service_type}' does not exist!")
                return False
            else:
                return True
        else:
            for index, client in enumerate(list_of_active_clients):
                if client == self.client:
                    self.service_type = list_of_service_types[index]
                    return True
            return False
        
    def init_service(self):
        try:
            self.service_request = self.get_service_request(self.service_type)
            self.empty_service_response = self.get_empthy_service_response()
            self.default_service_res_dict = message_to_ordereddict(self.empty_service_response)
            self.service_req_dict = message_to_ordereddict(self.service_request)
            self.service_metaclass = self.get_service_metaclass()
            self.service_req_dict_implicit = copy.deepcopy(self.service_req_dict)
        except Exception as e:
            print(e)
            self.node.get_logger().fatal(
                f"Error occured! Given service type does not exist (service_type: {self.service_type})!"
            )

    def execute(self) -> bool:
        if self.service_request and self.service_metaclass and self.service_type:
            # update srv request from dictionary
            self.update_srv_req_obj_from_dict()

            client = self.node.create_client(self.service_metaclass, self.client)
            
            srv_start_time = datetime.now()
            
            if not client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().error(f"Client {self.client} not available, aborting...")
                self.service_res_dict = collections.OrderedDict([("Error", "Client not available")])
                srv_call_success = False
                srv_end_time = datetime.now()
                self.update_log_entry(srv_call_success, srv_start_time, srv_end_time,additional_text='Client not available! Exited with error!')
                return srv_call_success

            self.node.get_logger().info(f"Executing '{self.name}'...")
            srv_start_time = datetime.now()
            # Call the service
            self.future = client.call_async(self.service_request)

            node_name = self.get_node_name_from_client(client.srv_name)
            timer = None
            if node_name is not None:
                timer = self.node.create_timer(timer_period_sec=1,callback=partial(self.client_executer_watchdog, node_name, self.future))
            else:
                self.node.get_logger().warn(f"Service execution watchdog for '{client.srv_name}' not available. Service client name does not adhere to the naming convention starting with the node name.")

            while not self.future.done() and not self.future.cancelled():
                rclpy.spin_once(self.node)

            success_val_from_srv_res = None

            if not self.future.cancelled():
                self.service_response = self.future.result()
                # update srv response dict
                self.service_res_dict = message_to_ordereddict(self.service_response)
                # get service success value if it is set
                success_val_from_srv_res = self.get_obj_value_from_key(self.service_response, self.service_success_key)

                if success_val_from_srv_res is not None:
                    srv_call_success = success_val_from_srv_res
                else:
                    srv_call_success = True
            else:
                srv_call_success = False

            srv_end_time = datetime.now()

            client.destroy()
            if timer is not None:
                timer.destroy()
            self.update_log_entry(srv_call_success, srv_start_time, srv_end_time)
            self.node.get_logger().info(f"Service return: {self.service_res_dict}")
            return srv_call_success

    def client_executer_watchdog(self, node_name, future_obj):
        node_names = self.node.get_node_names()

        if node_name in node_names:
            self.node.get_logger().info(f"Service Watchdog - node '{node_name}' is still active!")
        else:
            self.node.get_logger().info(f"Service Watchdog - node '{node_name}' is no longer active! Aboarting execution...")
            future_obj.cancel()
    
    def get_node_name_from_client(self, client: str)-> str:
        names = self.node.get_node_names()
        str_from_client_str = client.split('/')[1]
        if str_from_client_str in names:
            return str_from_client_str
        else:
            return  None
        
    def set_srv_req_dict_value_from_key(self, path_key: str, new_value: any, override_to_implicit=False) -> bool:
        """
        This function tries to set the value of the service request given the path_key to the value and a new value.
        Retuns false if key or value are incopatible with the service request.
        E.a. path_key = 'Foo.Fuu.Faa' (str)
        """
        if path_key is None:
            return False

        if new_value is None:
            return False

        try:
            test_request = self.get_service_request(self.service_type)
            value_to_set = self.get_obj_value_from_key(test_request, path_key)

            # if given key leads to an array entry
            if '[' in path_key and ']' in path_key:
                value_is_list_entry = True
                list_path_key = re.sub(r'\[.*?\]', '', path_key)
                self.node.get_logger().debug(f"New path {list_path_key}")
                index = self.get_last_index_value(path_key)
                self.node.get_logger().debug(f"index is {index}")
            else:
                value_is_list_entry = False

            # in case the value leads to an list entry we will process
            if value_is_list_entry:
                test_dict = copy.deepcopy(self.service_req_dict_implicit)
                list_to_set = self.get_obj_value_from_key(test_dict, list_path_key)
                self.node.get_logger().debug(f"List old '{str(list_to_set)}'")
                self.node.get_logger().debug(f"test_dict old '{str(test_dict)}'")

                if list_to_set is None:
                    self.node.get_logger().error(f"Error occured accessing list element '{list_path_key}' in dict!")
                    return False
                
                list_to_set[index] = new_value
                
                if not override_to_implicit:
                    set_success = self.set_obj_value_from_key(self.service_req_dict, list_path_key, list_to_set)
                    self.update_srv_req_obj_from_dict()
                else:
                    set_success = self.set_obj_value_from_key(self.service_req_dict_implicit, list_path_key, list_to_set)
                return set_success
            
            # Create a new service request object
            # Set test request with dict
            self.node.get_logger().debug(f"Path key {str(path_key)}")
            self.node.get_logger().debug(f"New value {str(new_value)}")
            self.node.get_logger().debug(f"Value to set {str(value_to_set)}")

            self.node.get_logger().debug(f"New value type {str(type(new_value))}")
            self.node.get_logger().debug(f"Value to set type {str(type(value_to_set))}")
            self.node.get_logger().debug(f"{str(type(value_to_set))}")

            if isinstance(value_to_set, np.ndarray) and isinstance(new_value, list):
                new_value = np.array(new_value)

            if isinstance(value_to_set, array.array) and isinstance(new_value, list):
                new_value = array.array("i", new_value)

            self.node.get_logger().debug(f"New value type {str(type(new_value))}")

            if not isinstance(new_value, type(value_to_set)) and not override_to_implicit:
                self.node.get_logger().debug(f"Given value '{new_value}' of type '{type(new_value)}' is incompatible for '{path_key}' of type '{type(value_to_set)}'!")
                return False

            # If the path does not lead to an existing value
            if value_to_set is None:
                return False
            
            if not override_to_implicit:
                set_success = self.set_obj_value_from_key(self.service_req_dict, path_key, new_value)
                #self.node.get_logger().debug(f"Set success {str((set_success))}")
                self.update_srv_req_obj_from_dict()
            else:
                #self.node.get_logger().debug(f"Set success {str((self.service_req_dict_implicit))}")
                set_success = self.set_obj_value_from_key(self.service_req_dict_implicit, path_key, new_value)
                #self.node.get_logger().debug(f"Set success {str((self.service_req_dict_implicit))}")
                #self.node.get_logger().debug(f"Set success {str((set_success))}")

            return set_success
        except:
            self.node.get_logger().debug("Error occured in set_srv_req_dict_value_from_key!")
            return False

    def set_req_message_from_dict(self, dict: collections.OrderedDict) -> bool:
        """
        This mehtod sets the request message from a given ordereddict.
        It first tries if the dict is a valid dict before setting it.
        """
        try:
            # Create a new service request object
            test_request = self.get_service_request(self.service_type)
            # Set test request with dict
            set_message_fields(test_request, dict)

            # if the function above does not fail, its a valid input dict. We can set it
            self.service_req_dict = dict

            self.update_srv_req_obj_from_dict()
            return True
        except:
            return False

    def get_empthy_service_response(self):
        service_response = get_service(self.service_type).Response()
        return service_response

    def get_service_metaclass(self):
        service = get_service(self.service_type)
        return service

    def init_service_res_bool_messages(self) -> None:
        """
        Initializes the service response bool messages from an empty service response
        """

        def check_for_bool_values(data, bool_list: list, parent_key=None):
            # iterate through the dict
            for key, value in data.items():
                if parent_key is not None:
                    full_key = parent_key + "." + key
                else:
                    full_key = key

                if isinstance(value, collections.OrderedDict):
                    check_for_bool_values(value, bool_list, full_key)
                else:
                    if isinstance(value, bool):
                        bool_list.append(full_key)

        if self.default_service_res_dict is not None:
            check_for_bool_values(self.default_service_res_dict, self.service_res_bool_messages)
        else:
            self.node.get_logger().warn("Init of default service response not possible")

    def get_service_bool_fields(self):
        """
        Returns a list of strings containing the full keys to all bool messages of the service response
        """
        return self.service_res_bool_messages

    def get_service_bool_identifier(self):
        """
        Returns the currently set service execution identifier
        """
        return self.service_success_key

    def set_service_bool_identifier(self, identifier: str) -> bool:
        """
        Sets the current service execution identifier
        """
        self.service_success_key = identifier
        return True

    def set_service_request(self, service_request: any):
        """
        Warning: The method doesn not check for valid inputs.
        This method set the service request object of the class from the input parameter service_request.
        It also updates the request dict.
        """
        self.service_request = service_request
        self.service_req_dict = message_to_ordereddict(self.service_request)

    def update_srv_req_obj_from_dict(self):
        """Update the ros service message from the dict"""
        set_message_fields(self.service_request, self.service_req_dict)

    def get_srv_res_value_from_key(self, key: str) -> any:
        return self.get_obj_value_from_key(obj=self.service_res_dict, path_key=key)

    def get_default_srv_res_value_from_key(self, key: str) -> any:
        return self.get_obj_value_from_key(
            obj=self.default_service_res_dict, path_key=key
        )

    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime,additional_text:str = ""):
        self.log_entry["service_client"] = self.client
        self.log_entry["service_type"] = self.service_type
        self.log_entry["srv_start_time"] = str(
            start_time.strftime("%Y-%m-%d_%H:%M:%S.%f")
        )
        self.log_entry["srv_end_time"] = str(end_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["execution_time"] = str(end_time - start_time)
        self.log_entry["srv_request"] = json.loads(json.dumps(self.service_req_dict))
        self.log_entry["srv_response"] = json.loads(json.dumps(self.service_res_dict))
        if not additional_text == '':
            self.log_entry["message"] = str(additional_text)
        self.log_entry["success"] = success

    def get_log_entry(self) -> dict:
        return self.log_entry

    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        new_instance = ServiceAction(
            client=self.client, service_type=self.service_type, node=self.node
        )
        new_instance.service_req_dict = copy.deepcopy(self.service_req_dict)
        new_instance.service_response = copy.deepcopy(self.service_response)
        new_instance.service_request = copy.deepcopy(self.service_request)
        new_instance.service_req_dict = copy.deepcopy(self.service_req_dict)
        new_instance.service_res_dict = copy.deepcopy(self.service_res_dict)
        new_instance.log_entry = copy.deepcopy(self.log_entry)
        new_instance.service_res_bool_messages = copy.deepcopy(
            self.service_res_bool_messages
        )
        new_instance.service_success_key = copy.deepcopy(self.service_success_key)
        new_instance.service_req_dict_implicit = copy.deepcopy(
            self.service_req_dict_implicit
        )

        return new_instance
    
    @staticmethod
    def get_service_request(service_type):
        try:
            service_request = get_service(service_type).Request()
            return service_request
        except Exception:
            return None

    @staticmethod
    def get_service_response(service_type):
        try:
            service_request = get_service(service_type).Response()
            return service_request
        except Exception:
            return None

    @staticmethod
    def set_obj_value_from_key(obj: any, path_key: str, new_value: any) -> bool:
        """
        This function sets the new_value at the value to which the path_key leads to. The obj can be any object, dict, list
        Returns False if value cant be set
        """
        keys = path_key.split(".")
        current_value = obj

        try:
            for key in keys[:-1]:
                if isinstance(current_value, dict) and key in current_value:
                    current_value = current_value[key]
                elif isinstance(current_value, list):
                    key = int(key)
                    current_value = current_value[key]
                elif hasattr(current_value, key):
                    current_value = getattr(current_value, key)
                else:
                    return False

            last_key = keys[-1]
            if isinstance(current_value, dict) and last_key in current_value:
                current_value[last_key] = new_value
            elif isinstance(current_value, list):
                last_key = int(last_key)
                current_value[last_key] = new_value
            elif hasattr(current_value, last_key):
                setattr(current_value, last_key, new_value)
            else:
                return False
            # Finally
            return True
        except (KeyError, IndexError, AttributeError, ValueError):
            return False


    def get_last_index_value(self, input_string:str)->int:
        # Use regular expression to find all occurrences of '[x]' and extract 'x' from the last one
        matches = re.findall(r'\[(\d+)\]', input_string)
        
        if matches:
            return int(matches[-1])
        else:
            return None
    
    #@staticmethod
    # def get_obj_value_from_key(obj: any, path_key: str) -> any:
    #     """
    #     This function iterates through an object (any object, list, dict) and returns the value given in the path.
    #     E.a. path_key = 'Foo.Fuu.Faa'
    #     """
    #     if path_key is None or obj is None:
    #         return None

    #     keys = path_key.split(".")
    #     current_value = obj

    #     try:
    #         for key in keys:
    #             if isinstance(current_value, dict) and key in current_value:
    #                 current_value = current_value[key]
    #             elif isinstance(current_value, list):
    #                 key = int(key)
    #                 current_value = current_value[key]
    #             elif hasattr(current_value, key):
    #                 current_value = getattr(current_value, key)
    #             else:
    #                 return None
    #     except (KeyError, IndexError, AttributeError, ValueError):
    #         return None

    #     return current_valu
    @staticmethod   
    def get_obj_value_from_key(obj: Union[dict, list, any], path_key: str) -> any:
        """
        This function iterates through an object (any object, list, dict) and returns the value given in the path.
        E.g. path_key = 'Foo.Fuu.Faa'
        """
        if path_key is None or obj is None:
            return None

        keys = path_key.split(".")
        current_value = obj

        try:
            for key in keys:
                if isinstance(current_value, dict) and key in current_value:
                    current_value = current_value[key]
                elif isinstance(current_value, list):
                    try:
                        key = int(key)
                        current_value = current_value[key]
                    except (ValueError, IndexError):
                        return None
                elif hasattr(current_value, key):
                    current_value = getattr(current_value, key)
                else:
                    return None
        except (KeyError, AttributeError):
            return None

        return current_value