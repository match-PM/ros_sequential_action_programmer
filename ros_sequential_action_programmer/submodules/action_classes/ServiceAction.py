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
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from typing import Tuple, Any
from collections import OrderedDict

from rosidl_parser.definition import BasicType, Array, AbstractNestedType, NamespacedType
from ros_sequential_action_programmer.submodules.action_classes.ros_messages_functions import field_type_map_recursive, field_type_map_recursive_with_msg_type
from ros_sequential_action_programmer.submodules.rsap_modules.errors import ActionInitializationError, SetActionRequestError, EvaluateActionReferenceError

def set_at_index(lst, index, value):
    while len(lst) <= index:  # Expand list if needed
        lst.append(None)
    lst[index] = value
    
class ServiceAction(ActionBaseClass):
    def __init__(self, node: Node, client: str, service_type: str = None, name=None, description="") -> None:
        super().__init__(node, name, description)
        
        self.client = client
        self.service_type = service_type
        
        self.logger = self.node.get_logger()
        self.service_metaclass = None
        
        self.response = None

        self.service_res_bool_messages = []

        if name is None:
            self.name = self.client
        else:
            self.name = name

        self.valid = self.check_for_valid_inputs()

        if not self.valid:
            raise ActionInitializationError(f"Service Action '{self.get_name()}' could not be initialized!")
        
        self.init_service()
        self.init_service_res_bool_messages()

    
    def get_type_indicator(self)->str:
        return "ServiceAction"
    
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
            self.request = self.get_service_request(self.service_type)
            self.empty_response = self.get_empthy_service_response()
            self.default_response_dict = message_to_ordereddict(self.empty_response)
            self.request_dict = message_to_ordereddict(self.request)
            self.service_metaclass = self.get_service_metaclass()
            self.request_dict_implicit = copy.deepcopy(self.request_dict)
            self.default_request = copy.deepcopy(self.request)
        except Exception as e:
            print(e)
            self.node.get_logger().fatal(
                f"Error occured! Given service type does not exist (service_type: {self.service_type})!"
            )

    def execute(self, get_interupt_method:Any = None) -> bool:

        # evaluate references and update request dict
        try:
            new_request_dict = self.evaluate_references()
            #self.node.get_logger().warn(f"Service Action '{self.get_name()}' evaluated request dict: {new_request_dict}")

            self.set_request_from_dict(new_request_dict)
            #self.node.get_logger().warn(f"Service Action '{self.get_name()}' evaluated request dict: {self.request}")


        except EvaluateActionReferenceError as e:
            self.node.get_logger().error(f"Error occured evaluating action references for service action '{self.get_name()}'! {str(e)}")
            return False

        if self.request and self.service_metaclass and self.service_type:
            # update srv request from dictionary

            client = self.node.create_client(self.service_metaclass, self.client)
            
            srv_start_time = datetime.now()
            
            if not client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().error(f"Client {self.client} not available, aborting...")
                self.response_dict = collections.OrderedDict([("Error", "Client not available")])
                srv_call_success = False
                srv_end_time = datetime.now()
                self.update_log_entry(srv_call_success, srv_start_time, srv_end_time, additional_text='Client not available! Exited with error!')
                return srv_call_success

            self.node.get_logger().info(f"Executing '{self.name}'...")
            srv_start_time = datetime.now()
            # Call the service
            self.future = client.call_async(self.request)

            node_name = self.get_node_name_from_client(client.srv_name)
            timer = None
            if node_name is not None:
                timer = self.node.create_timer(timer_period_sec=1,callback=partial(self.client_executer_watchdog, 
                                                                                   node_name, 
                                                                                   self.future,
                                                                                   get_interupt_method))
            else:
                self.node.get_logger().warn(f"Service execution watchdog for '{client.srv_name}' not available. Service client name does not adhere to the naming convention starting with the node name.")

            while not self.future.done() and not self.future.cancelled():
                rclpy.spin_once(self.node)

            success_val_from_srv_res = None

            if not self.future.cancelled():
                self.response = self.future.result()
                # update srv response dict
                self.response_dict = message_to_ordereddict(self.response)
                # get service success value if it is set
                success_val_from_srv_res = self.get_obj_value_from_key(self.response, self._success_key)

                if success_val_from_srv_res is not None:
                    srv_call_success = success_val_from_srv_res
                else:
                    srv_call_success = True
            else:
                srv_call_success = False

            srv_end_time = datetime.now()

            #self.node.get_logger().info(f"Test1..")

            client.destroy()
            if timer is not None:
                timer.destroy()
            
            #self.node.get_logger().info(f"Test2..")
            self.update_log_entry(srv_call_success, srv_start_time, srv_end_time)
            #self.node.get_logger().info(f"Test3..")
            #self.node.get_logger().info(f"Service return: {self.response_dict}")
            
            return srv_call_success

        else:
            self.node.get_logger().error(f"Service Action '{self.get_name()}' not properly initialized!")
            return False

    def client_executer_watchdog(self, node_name, future_obj, get_interupt_method):
        node_names = self.node.get_node_names()
        if get_interupt_method is not None:
            if get_interupt_method():
                self.node.get_logger().info(f"Service Watchdog - Interupt method triggered! Aborting...")
                future_obj.cancel()

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
        
    def get_empthy_service_response(self):
        response = get_service(self.service_type).Response()
        return response

    def get_service_metaclass(self):
        service = get_service(self.service_type)
        return service


    def init_service_res_bool_messages(self) -> None:
        """
        Initializes the service response bool messages from the type information of the response message.
        Populates self.service_res_bool_messages with full keys to all boolean fields.
        """
        self.service_res_bool_messages = []  # clear previous entries
        #self.service_res_bool_messages.append("None")

        def recurse_fields(fields: dict, parent_key=None):
            for key, info in fields.items():
                full_key = f"{parent_key}.{key}" if parent_key else key

                # Debug log: current key and info
                #self.node.get_logger().debug(f"Checking field: '{full_key}', info: {info}")

                # Check if the field is boolean
                if info.get('type') == 'boolean' or info.get('type') == 'bool':
                    #self.node.get_logger().info(f"Found boolean field: '{full_key}'")
                    self.service_res_bool_messages.append(full_key)

                # Recurse into nested fields
                if 'fields' in info:
                    #self.node.get_logger().debug(f"Recursing into nested fields of '{full_key}'")
                    recurse_fields(info['fields'], full_key)

                # If it's an array of nested messages, recurse into fields
                if info.get('is_array') and 'fields' in info:
                    #self.node.get_logger().debug(f"Recursing into array of nested messages for '{full_key}'")
                    recurse_fields(info['fields'], full_key + '.*')  # optional: add '*' to indicate array

        # Get type dict of the response message
        if self.service_metaclass and hasattr(self.service_metaclass, 'Response'):
            type_dict = field_type_map_recursive_with_msg_type(self.service_metaclass.Response)
            self.node.get_logger().debug(f"Response type dict: {type_dict}")
            recurse_fields(type_dict)
            #self.node.get_logger().info(f"All boolean fields found: {self.service_res_bool_messages}")
        else:
            self.node.get_logger().warn(
                "Cannot initialize service response bool messages: Response type not available."
            )

    def get_res_bool_fields(self)->list[str]:
        """
        Returns a list of strings containing the full keys to all bool messages of the service response
        """
        return self.service_res_bool_messages

    def update_log_entry(self, success: bool, start_time: datetime, end_time: datetime,additional_text:str = ""):

        self.log_entry["service_client"] = self.client
        self.log_entry["service_type"] = self.service_type
        self.log_entry["srv_start_time"] = str(start_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["srv_end_time"] = str(end_time.strftime("%Y-%m-%d_%H:%M:%S.%f"))
        self.log_entry["execution_time"] = str(end_time - start_time)

        self.log_entry["srv_request"] = json.loads(json.dumps(self.get_request_as_ordered_dict()))

        if self.response is not None:
            self.log_entry["srv_response"] = json.loads(json.dumps(self.get_response_as_ordered_dict()))

        if not additional_text == '':
            self.log_entry["message"] = str(additional_text)
        self.log_entry["success"] = success
    

    def __deepcopy__(self, memo):
        """
        deepcopy of this class is not possible without this mehtod definition
        """
        new_instance = ServiceAction(
            client=self.client, service_type=self.service_type, node=self.node
        )
        new_instance.request_dict
        new_instance.request_dict = copy.deepcopy(self.request_dict)
        new_instance.response = copy.deepcopy(self.response)
        new_instance.request = copy.deepcopy(self.request)
        new_instance.request_dict = copy.deepcopy(self.request_dict)
        new_instance.request_dict = copy.deepcopy(self.request_dict)
        new_instance.log_entry = copy.deepcopy(self.log_entry)
        new_instance.service_res_bool_messages = copy.deepcopy(
            self.service_res_bool_messages
        )
        new_instance.set_references(copy.deepcopy(self.get_references()))
        new_instance._success_key = copy.deepcopy(self._success_key)
        new_instance.request_dict_implicit = copy.deepcopy(
            self.request_dict_implicit
        )
        new_instance.set_name(f"{self.get_name()}_copy")

        return new_instance
    
    def get_request_type(self):

        type_dict = field_type_map_recursive_with_msg_type(self.service_metaclass.Request)
        
        return type_dict
    
    def get_response_type(self):
        type_dict = field_type_map_recursive_with_msg_type(self.service_metaclass.Response)        
        return type_dict
        
    def get_request_as_ordered_dict(self)->OrderedDict:
        return message_to_ordereddict(self.request)
    
    def get_response_as_ordered_dict(self):
        return message_to_ordereddict(self.response)
    
    def set_request_from_dict(self,request_dictionary:Union[dict,OrderedDict]) -> bool:
        """Update the ros service message from the dict"""
        try:
            #self.logger.warn(f"New Dict {str(request_dictionary)}")

            set_message_fields(self.request, request_dictionary)
            self.request_dict = message_to_ordereddict(self.request)
            
            #self.logger.warn(f"Result {str(self.request)}")
            #self.logger.warn(f"Request Dict {str(self.request_dict)}")

        except Exception as e:
            raise SetActionRequestError(f"Could not set request from dictionary for {self.name}!")
    
    @staticmethod
    def get_service_request(service_type):
        try:
            request = get_service(service_type).Request()
            return request
        except Exception:
            return None

    @staticmethod
    def get_service_response(service_type):
        try:
            request = get_service(service_type).Response()
            return request
        except Exception:
            return None

    @staticmethod
    def set_obj_value_from_key(obj: any, 
                               path_key: str, 
                               new_value: any,
                               logger = None) -> bool:
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
                    #if logger is not None:
                    #    logger.warn(f"TEST CURRENT VALUE {str(current_value)}")
                elif hasattr(current_value, key):
                    current_value = getattr(current_value, key)
                else:
                    return False

            last_key = keys[-1]

            #if logger is not None:
            #    logger.error(f"TEST CURRENT '{str(current_value)}' LAST KEY '{str(last_key)}' NEW VALUE '{str(new_value)}' PATH KEY '{str(path_key)}'")


            if isinstance(current_value, dict) and last_key in current_value:
                current_value[last_key] = new_value
            elif isinstance(current_value, list):

                if logger is not None:
                    logger.warn(f"TEST CURRENT VALUE {str(current_value)}")

                current_value.clear()

                if logger is not None:
                    logger.warn(f"TEST CURRENT VALUE {str(current_value)}")

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