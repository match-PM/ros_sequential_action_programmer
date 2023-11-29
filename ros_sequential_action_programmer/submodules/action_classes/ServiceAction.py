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
from rosidl_runtime_py.get_interfaces import get_service_interfaces
import array
import numpy as np


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
    
    def check_for_valid_inputs(self) -> bool:
        list_of_active_services = self.node.get_service_names_and_types()
        list_of_active_clients = [item[0] for item in list_of_active_services]
        list_of_service_types = [item[1][0] for item in list_of_active_services]
        service_interfaces = ServiceAction.flatten_dict_to_list(get_service_interfaces())

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
        
    @staticmethod
    def flatten_dict_to_list(input_dict):
        result_list = []
        for key, values in input_dict.items():
            for value in values:
                result_list.append(f"{key}/{value}")
        return result_list
    
    @staticmethod
    def is_string_in_dict(my_dict, target_string):
        for key, value in my_dict.items():
            if isinstance(value, list):
                # Check if the target string is in the list
                if target_string in value:
                    return True
            elif isinstance(value, dict):
                # Recursively check in nested dictionaries
                if ServiceAction.is_string_in_dict(value, target_string):
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
                srv_call_success = False
                srv_end_time = datetime.now()
                self.update_log_entry(srv_call_success, srv_start_time, srv_end_time,additional_text='Client not available! Exited with error!')
                return srv_call_success

            self.node.get_logger().info(f"Executing '{self.name}'...")
            srv_start_time = datetime.now()
            # Call the service
            self.future = client.call_async(self.service_request)
            # spin until complete

            while not self.future.done() and self.client_executer_watchdog(self.client):
                rclpy.spin_once(self.node)
            #rclpy.spin_until_future_complete(self.node, self.future)

            self.service_response = self.future.result()
            srv_end_time = datetime.now()
            # update srv response dict
            self.service_res_dict = message_to_ordereddict(self.service_response)
            # get service success value if it is set
            success_val_from_srv_res = self.get_obj_value_from_key(
                self.service_response, self.service_success_key
            )
            if success_val_from_srv_res is not None:
                srv_call_success = success_val_from_srv_res
            else:
                srv_call_success = True

            client.destroy()
            self.update_log_entry(srv_call_success, srv_start_time, srv_end_time)
            return srv_call_success

    def client_executer_watchdog(self, client)-> bool:
        list_of_active_services = self.node.get_service_names_and_types()
        list_of_active_clients = [item[0] for item in list_of_active_services]
        if client in list_of_active_clients:
            return True
        else:
            self.node.get_logger().warn(f"Client '{client} is not longer active. Aborting execution!")
            return False
        
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
            # Create a new service request object
            test_request = self.get_service_request(self.service_type)
            # Set test request with dict
            value_to_set = self.get_obj_value_from_key(test_request, path_key)
            self.node.get_logger().debug(f"Path key {str(path_key)}")
            self.node.get_logger().debug(f"New value {str(new_value)}")
            self.node.get_logger().debug(f"Value to set {str(value_to_set)}")

            self.node.get_logger().debug(f"New value type {str(type(new_value))}")
            self.node.get_logger().debug(f"Value to set type {str(type(value_to_set))}")

            # TO-DO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1

            if isinstance(value_to_set, np.ndarray) and isinstance(new_value, list):
                new_value = np.array(new_value)

            if isinstance(value_to_set, array.array) and isinstance(new_value, list):
                new_value = array.array("i", new_value)

            if not isinstance(new_value, type(value_to_set)):
                self.node.get_logger().debug(f"Given value '{new_value}' of type '{type(new_value)}' is incompatible for '{path_key}' of type '{type(value_to_set)}'!")
                return False

            # If the path does not lead to an existing value
            if value_to_set is None:
                return False

            if not override_to_implicit:
                set_success = self.set_obj_value_from_key(
                    self.service_req_dict, path_key, new_value
                )
                self.node.get_logger().debug(f"Set success {str((set_success))}")
                self.update_srv_req_obj_from_dict()
            else:
                self.node.get_logger().debug(f"Set success {str((self.service_req_dict_implicit))}")
                set_success = self.set_obj_value_from_key(
                    self.service_req_dict_implicit, path_key, new_value
                )
                self.node.get_logger().debug(f"Set success {str((self.service_req_dict_implicit))}")
                self.node.get_logger().debug(f"Set success {str((set_success))}")

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

        check_for_bool_values(
            self.default_service_res_dict, self.service_res_bool_messages
        )

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

    @staticmethod
    def get_obj_value_from_key(obj: any, path_key: str) -> any:
        """
        This function iterates through an object (any object, list, dict) and returns the value given in the path.
        E.a. path_key = 'Foo.Fuu.Faa'
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
                    key = int(key)
                    current_value = current_value[key]
                elif hasattr(current_value, key):
                    current_value = getattr(current_value, key)
                else:
                    return None
        except (KeyError, IndexError, AttributeError, ValueError):
            return None

        return current_value

    @staticmethod
    def get_key_value_pairs_from_dict(
        dictionary: collections.OrderedDict,
    ) -> list[dict]:
        """
        Returns a list of all keys and values of a given dictionary.
        """

        def get_key_values(dic, k_v_list: list, parent_key=None):
            # iterate through the dict
            for key, value in dic.items():
                if parent_key is not None:
                    full_key = parent_key + "." + key
                else:
                    full_key = key

                if isinstance(value, collections.OrderedDict):
                    get_key_values(value, k_v_list, full_key)
                else:
                    k_v_list.append({full_key: value})

        key_value_list = []
        get_key_values(dictionary, key_value_list)
        return key_value_list
