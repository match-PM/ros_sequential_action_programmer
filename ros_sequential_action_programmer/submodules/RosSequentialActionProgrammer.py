import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_interface
from ament_index_python.packages import get_package_share_directory
from rqt_py_common import message_helpers
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import (
    ServiceAction,
)
from ros_sequential_action_programmer.submodules.action_classes.RecomGenerator import (
    RecomGenerator,
)
import json
from datetime import datetime
import os
import copy
import collections
from collections import OrderedDict
from typing import Tuple

CHECK = 0
SET = 1

CHECK_COMPATIBILITY_ONLY = 0
SET_IMPLICIT_SRV_DICT = 1
SET_SRV_DICT = 2


class RosSequentialActionProgrammer:
    def __init__(self, node: Node) -> None:
        self.name = None
        self.action_list: list[ServiceAction] = []
        self.node = node
        self.folder_path = None
        self.current_action_index = 0
        self.current_action = None

        self.list_of_active_services = None
        self.list_of_active_clients = None
        self._list_of_service_types = None

        self.initialize_service_list()
        self.action_sequence_log = {}
        self.action_log = []
        self.recommendations = RecomGenerator(node)

    def initialize_service_list(self):
        """
        Initialize list of active services.
        """
        self.list_of_active_services = self.node.get_service_names_and_types()
        self.list_of_active_clients = [item[0] for item in self.list_of_active_services]
        self._list_of_service_types = [item[1] for item in self.list_of_active_services]

    def get_active_services(self):
        return self.list_of_active_services

    def get_service_type_for_client(self, service_client) -> str:
        """
        Returns the service type of a clients as a string.
        """
        for index, client in enumerate(self.list_of_active_clients):
            if client == service_client:
                return self._list_of_service_types[index][0]
        return None

    def append_service_to_action_list(
        self, service_client: str, service_type: str = None, service_name=None
    ) -> bool:
        if not service_type:
            service_type = self.get_service_type_for_client(service_client)
        else:
            test_obj = ServiceAction.get_service_request(service_type)
            if not test_obj:
                self.node.get_logger().error(
                    f"Service type '{service_type}' is unknown to the system!"
                )
                return False

        if service_type:
            self.action_list.append(
                ServiceAction(
                    client=service_client,
                    service_type=service_type,
                    node=self.node,
                    name=service_name,
                )
            )
            self.node.get_logger().info(
                f"Inserted Service for {service_client} at {len(self.action_list)-1}"
            )
            return True
        else:
            self.node.get_logger().error(
                f"Service for client '{service_client}' could not be appended because service type does not exist or is unknown!"
            )
            return False

    def append_service_to_action_list_at_index(
        self,
        service_client: str,
        index: int,
        service_type: str = None,
        service_name=None,
    ) -> bool:
        if not service_type:
            service_type = self.get_service_type_for_client(service_client)

        if not index > len(self.action_list):
            self.action_list.insert(
                index,
                ServiceAction(
                    client=service_client,
                    service_type=service_type,
                    node=self.node,
                    name=service_name,
                ),
            )

            self.current_action_index = index
            self.node.get_logger().info(
                f"Inserted Service for {service_client} at {index}"
            )
            return True
        else:
            return False

    def get_action_at_index(self, index: int) -> ServiceAction:
        if not index > len(self.action_list):
            return self.action_list[index]
        else:
            return None

    def delete_action_at_index(self, index: int) -> bool:
        if not index > len(self.action_list):
            del self.action_list[index]
            return True
        else:
            return False

    def get_service_req_from_dict(
        self, client: str, dict: dict, service_type=None
    ) -> any:
        """
        This function returns a service request object given the client and a dict.
        If the client is not spinning when calling this function, you also have to specify the service_type.
        The dict has to have the appropriate structure or the function will return 'None'.
        """
        if service_type is None:
            service_type = self.get_service_type_for_client(client)
        try:
            service_request = ServiceAction.get_service_request(service_type)
            service_ordered_dict = collections.OrderedDict(dict)
            set_message_fields(service_request, service_ordered_dict)
            return service_request
        except Exception as e:
            print(e)
            return None

    def load_from_JSON(self, file_path) -> bool:
        """
        Sets the action list from a given json-file path.
        """
        file_data = None
        loading_action_list = []
        try:
            with open(file_path, "r") as json_file:
                file_data = json.load(json_file)
            # clear action_list to be empty
            self.action_list.clear()
        except Exception as e:
            self.node.get_logger().error("Error opening file")

        if file_data:
            self.folder_path = os.path.dirname(file_path)
            self.name = file_data["name"]

            for index, item in enumerate(file_data["action_list"]):
                service_client = item["service_client"]

                action_from_item = ServiceAction(
                    service_client, item["service_type"], self.node, item["name"]
                )

                set_success = action_from_item.set_service_bool_identifier(
                    item["error_identifier"]
                )
                action_from_item.description = item["description"]
                if not set_success:
                    return False

                self.action_list.append(action_from_item)
                set_success = self.process_action_dict_at_index(
                    index=index,
                    mode=SET_IMPLICIT_SRV_DICT,
                    input_impl_dict=item["service_request"],
                )
                if not set_success:
                    self.action_list.clear()
                    return False

            return True
        else:
            return False

    def save_to_JSON(self) -> bool:
        """
        Saves the action_list to a json-file.
        In order to execute the function successfully, a valid path and the action_list name must be set.
        """

        if (self.folder_path is not None) and (self.name is not None):
            process_dict = {}
            process_dict["name"] = self.name
            process_dict["saved_at"] = str(datetime.now())  # Add a timestamp

            action_list = []
            for index, action in enumerate(self.action_list):
                service_dict = {}
                service_dict["action_position"] = index
                service_dict["name"] = action.name
                service_dict["service_client"] = action.client
                service_dict["service_type"] = action.service_type
                service_dict["error_identifier"] = action.service_success_key
                service_dict["description"] = action.description
                service_dict["service_request"] = json.loads(
                    json.dumps(action.service_req_dict_implicit)
                )
                action_list.append(service_dict)

            process_dict["action_list"] = action_list
            try:
                with open(f"{self.folder_path}/{self.name}.json", "w") as json_file:
                    json.dump(process_dict, json_file)
                self.node.get_logger().info("Saved!")
                return True
            except FileNotFoundError as e:
                print(e)
                self.node.get_logger().error("Directory does not exist!")
                return False
            except Exception as e:
                print(e)
                self.node.get_logger().error("Saving file failed!")
                return False
        else:
            return False

    def set_current_action(self, index: int) -> bool:
        """
        Sets the current action to the index from input.
        """
        if index < len(self.action_list):
            self.current_action_index = index
            return True
        else:
            return False

    def check_if_name_exists(self, name: str) -> bool:
        """
        Checks if a service with the given name exixts in the action_list.
        """
        for action in self.action_list:
            if action.name == name:
                return True
        return False

    def get_current_action_name(self) -> str:
        return self.action_list[self.current_action_index].name

    def execute_current_action(self) -> bool:
        try:
            # Set values from earlier service respones to this service request, might fail, if earlier call has not been executed
            set_success = self.process_action_dict_at_index(
                self.current_action_index, SET_SRV_DICT
            )

            if not set_success:
                raise Exception

            success_exec = self.action_list[self.current_action_index].execute_service()
            # append action log to history
            self.append_action_log(
                index=self.current_action_index,
                action_name=self.get_current_action_name(),
                action_log=self.get_current_action_log(),
            )

            if self.get_current_action_index() == len(self.action_list) - 1:
                self.save_action_sequence_log()

            if success_exec:
                self.node.get_logger().info(
                    f"Action {self.get_current_action_name()} executed successfully!"
                )
            else:
                raise Exception

            return success_exec

        except Exception:
            self.node.get_logger().error(
                f"Error executing {self.get_current_action_name()}!"
            )
            return False

    def execute_action_list(self, index_start) -> Tuple[bool, int]:
        """
        This function executes the action_list successifly starting at the start_index.
        It returns bool value for success indication and also the index of the action when the method terminates.
        """
        if index_start < len(self.action_list):
            for index in range(len(self.action_list)):
                self.current_action_index = index
                success = self.execute_current_action()
                if not success:
                    return False, self.current_action_index

            return True, self.current_action_index
        else:
            return False, self.current_action_index

    def get_copy_srv_dict_at_index(self, index: int) -> collections.OrderedDict:
        """
        This method returns the service request as a OrderedDict.
        The dict will be copied so changes to the OrderedDict will not apply to the process action_list.
        """
        try:
            return_value = copy.deepcopy(self.action_list[index].service_req_dict)
            return return_value
        except Exception as e:
            return None

    def get_copy_impl_srv_dict_at_index(self, index: int) -> collections.OrderedDict:
        """
        This method returns the service request as a OrderedDict.
        The dict will be copied so changes to the OrderedDict will not apply to the process action_list.
        """
        try:
            return_value = copy.deepcopy(
                self.action_list[index].service_req_dict_implicit
            )
            return return_value
        except Exception as e:
            return None

    def set_service_req_from_dict_at_index(
        self, index: int, dict: collections.OrderedDict
    ) -> bool:
        """
        Tries to set the service request of the service at index. Returns bool for success.
        """
        try:
            set_success = self.action_list[index].set_req_message_from_dict(dict)
            return set_success
        except:
            return False

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

    def get_all_service_req_res_dict(self):
        """
        This function returns all dict of service request and responses.
        """
        self.initialize_service_list()
        service_dict = {}
        for client in self.list_of_active_clients:
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

        def merge_dicts(dict1: dict, dict2: dict):
            result_dict = dict1.copy()
            for key, value in dict2.items():
                if key not in result_dict:
                    result_dict[key] = value
            return result_dict

        merged_dict = {}
        data = self.get_all_service_req_res_dict()
        path = get_package_share_directory("ros_sequential_action_programmer")
        file_name = "all_service_req_res_dicts.json"
        file_path = f"{path}/{file_name}"
        try:
            with open(file_path, "r") as json_file:
                file_data = json.load(json_file)

            merged_dict = merge_dicts(data, file_data)
        except Exception:
            self.node.get_logger().warn(
                "No history of service clients found! Creating a new one!"
            )
            pass
        try:
            with open(file_path, "w") as json_file:
                json.dump(merged_dict, json_file)
            self.node.get_logger().info(
                "Json with all service requests and responses exported!"
            )
            return True
        except FileNotFoundError as e:
            print(e)
            self.node.get_logger().error("Directory does not exist!")
            return False
        except Exception as e:
            print(e)
            self.node.get_logger().error("Saving file failed!")
            return False

    def get_recom_for_action_at_index_from_key(self, index: int, key: str) -> list:
        possible_list = self.get_possible_srv_res_fields_at_index(
            index=index, target_key=key
        )
        self.recommendations.append_to_recommendation(
            {"Service Responses": possible_list}
        )
        custom_recom_list = []
        for category in self.recommendations.get_recommendations():
            for cat_key, cat_list in category.items():
                list_of_items = []
                for item in cat_list:
                    set_possible = self.process_action_dicts_at_index_from_key(
                        index=index, key=key, value=item, mode=CHECK_COMPATIBILITY_ONLY
                    )
                    if set_possible:
                        list_of_items.append(item)
                if list_of_items:
                    custom_recom_list.append({str(cat_key): list_of_items})
        return custom_recom_list

    def get_possible_srv_res_fields_at_index(self, index: int, target_key: str) -> list:
        compatible_list = []
        for ind in range(0, index):
            key_value_list: list = ServiceAction.get_key_value_pairs_from_dict(
                self.action_list[ind].default_service_res_dict
            )
            for item in key_value_list:
                # unfold key value pairs
                for key, value in item.items():
                    compatible = self.process_action_dicts_at_index_from_key(
                        index=index,
                        key=target_key,
                        value=value,
                        mode=CHECK_COMPATIBILITY_ONLY,
                    )
                    if compatible:
                        compatible_list.append(
                            f"service_response.{ind}-{self.get_action_at_index(ind).name}.{key}"
                        )
        return compatible_list

    def convert_to_ordered_dict(self, d):
        if isinstance(d, dict):
            return OrderedDict(
                (key, self.convert_to_ordered_dict(value)) for key, value in d.items()
            )
        elif isinstance(d, list):
            return [self.convert_to_ordered_dict(item) for item in d]
        else:
            return d

    def process_action_dict_at_index(
        self, index: int, mode: int, input_impl_dict=None
    ) -> bool:
        """
        This function provides functionality to check/set input_dict for the dict of a action at index specified by the key.
        Use constants CHECK_COMPATIBILITY_ONLY (=0) or SET_IMPLICIT_SRV_DICT (=1) SET_SRV_DICT (=2).
        This function can do three things:
        1. CHECK_COMPATIBILITY_ONLY - Check if the given dict is applicable to the dict of an action at index specified by the key (the dict can also contain references to another action output dict. It checks for compatibility with the default values)
        2. SET_IMPLICIT_SRV_DICT - Set the implicit dictionary of an action at index to the given dict (The dict can also contain references to another action; compatibility will be checked).
        3. SET_SRV_DICT - Set the dict (explicit) of an action at index to the input dict (if not given the implicit dict of the action) (this can only be a dict with actual values and should not contain references; function checks for this).
        """
        # get a key value list that contains the key to the values of the service request.
        # because it is the implicit, there might be references to earlier service responses.
        if input_impl_dict is None:
            key_value_list: list = ServiceAction.get_key_value_pairs_from_dict(
                self.action_list[index].service_req_dict_implicit
            )
        else:
            if not isinstance(input_impl_dict, OrderedDict):
                input_impl_dict = self.convert_to_ordered_dict(input_impl_dict)
            key_value_list: list = ServiceAction.get_key_value_pairs_from_dict(
                input_impl_dict
            )

        # By default True, only if an setting or checking fails False is returned
        process_success = False
        try:
            # iterate through the list of key value pairs. Look for references to earlier respones.
            for item in key_value_list:
                # unfold key value pairs
                for key, value in item.items():
                    process_success = self.process_action_dicts_at_index_from_key(
                        index=index, key=key, value=value, mode=mode
                    )
                    if not process_success:
                        return False
            return process_success
        except:
            return False

    def process_action_dicts_at_index_from_key(
        self, index: int, key: str, value: any, mode: int
    ) -> bool:
        """
        This function provides functionality to check/set new_values for a value of a action at index specified by the key.
        Use constants CHECK_COMPATIBILITY_ONLY (=0) or SET_IMPLICIT_SRV_DICT (=1) SET_SRV_DICT (=2).
        This function can do three things:
        1. CHECK_COMPATIBILITY_ONLY - Check if the given value is applicable to the value of an action at index specified by the key (the value can also be a reference to another action output. It checks for compatibility with the default values)
        2. SET_IMPLICIT_SRV_DICT - Set the value of the implicit dictionary of an action at index specified by the key to the given value (The value can also contain references to another action; compatibility will be checked).
        3. SET_SRV_DICT - Set the value of the dictionary (explicit) of an action at index specified by the key to the given value (this can only be an actual value and not a reference; function checks for this).
        """
        try:
            set_success = False
            (
                is_reference,
                error_in_reference,
                ref_index,
                ref_key,
            ) = self.check_for_reference(value)
            if error_in_reference:
                return False

            check_value = value

            if is_reference:
                if mode == CHECK_COMPATIBILITY_ONLY or mode == SET_IMPLICIT_SRV_DICT:
                    check_value = self.get_action_at_index(
                        ref_index
                    ).get_default_srv_res_value_from_key(key=ref_key)
                else:
                    value = self.get_action_at_index(
                        ref_index
                    ).get_srv_res_value_from_key(key=ref_key)

            if mode == CHECK_COMPATIBILITY_ONLY:
                if isinstance(self.get_action_at_index(index), ServiceAction):
                    test_action = copy.deepcopy(self.get_action_at_index(index))
                    set_success = test_action.set_srv_req_dict_value_from_key(
                        key, check_value
                    )
                    del test_action

            if mode == SET_IMPLICIT_SRV_DICT:
                if isinstance(self.get_action_at_index(index), ServiceAction):
                    test_action = copy.deepcopy(self.get_action_at_index(index))
                    set_success = test_action.set_srv_req_dict_value_from_key(
                        key, check_value
                    )
                    del test_action
                    if set_success:
                        set_success = self.get_action_at_index(
                            index
                        ).set_srv_req_dict_value_from_key(
                            path_key=key, new_value=value, override_to_implicit=True
                        )

            if mode == SET_SRV_DICT:
                if isinstance(self.get_action_at_index(index), ServiceAction):
                    set_success = self.get_action_at_index(
                        index
                    ).set_srv_req_dict_value_from_key(path_key=key, new_value=value)

            if not set_success:
                if not mode == CHECK_COMPATIBILITY_ONLY:
                    self.node.get_logger().error(
                        f"In {self.get_action_at_index(index).name}, failed at setting key '{str(key)}' to value: '{str(value)}'"
                    )
                    if is_reference:
                        self.node.get_logger().error(
                            f"Action {ref_index}-{self.get_action_at_index(ref_index).name} might not have been executed yet!"
                        )
                return False

            return set_success
        except:
            return False

    def check_for_reference(self, input_value: any) -> (bool, int, str):
        """
        This function takes an input value as input. The function checks if the value conatins an reference to another action value.
        If yes, it returns 1. True/False for if it contains a reference, 2. if a given refence contains erros,
        3. if a ref the index of the action in the action plan and 4. the key of the value of the targeted action.
        """
        ref_index = None
        ref_key = None
        error_in_reference = False
        if not isinstance(input_value, str):
            return False, error_in_reference, ref_index, ref_key

        identifier = input_value.split(".")
        if identifier[0] == "service_response":
            ref_index = int(identifier[1].split("-")[0])
            ref_action_name = str(identifier[1].split("-")[1])
            # print(ref_index)
            # print(ref_action_name)
            if (
                ref_action_name != self.get_action_at_index(ref_index).name
                or not ref_index
            ):
                error_in_reference = True
                self.node.get_logger().error("Action name or index does not match!")
                return False, error_in_reference, ref_index, ref_key

            ref_key = ".".join(identifier[2:])
            return True, error_in_reference, ref_index, ref_key
        else:
            return False, error_in_reference, ref_index, ref_key

    def get_current_action_log(self) -> dict:
        return self.action_list[self.current_action_index].get_srv_log_entry()

    def get_current_action_index(self) -> int:
        return self.current_action_index

    # Log - Functions
    def append_action_log(self, index: int, action_name: str, action_log) -> None:
        """
        Appends the given action log to the actions log.
        """
        key = f"{index}_{action_name}"
        self.action_log.append({key: action_log})

    def update_action_sequence_log(self):
        self.action_sequence_log["action_sequence_name"] = self.name
        self.action_sequence_log["action_log"] = self.action_log

    def save_action_sequence_log(self) -> bool:
        """
        Saves the action sequence log to the folder.
        """
        if (self.folder_path is not None) and (self.name is not None):
            self.update_action_sequence_log()
            export_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            try:
                file_path = f"{self.folder_path}/logs/"
                if not os.path.exists(file_path):
                    os.makedirs(file_path)
                with open(
                    f"{file_path}/{self.name}_log_{str(export_time)}.json", "w"
                ) as json_file:
                    json.dump(self.action_sequence_log, json_file)
                self.node.get_logger().info("Action sequence log saved!")
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


if __name__ == "__main__":
    pass
