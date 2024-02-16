import os
import json
from rclpy.node import Node
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.obj_dict_modules.dict_functions import convert_to_ordered_dict, get_key_value_pairs_from_dict
from typing import Tuple, Any

class ServiceCalls:
    def __init__(self, node:Node) -> None:
        super().__init__()

        self.service_node = node
        self.service_action = None

    def init_service_action(self,service_name, service_type) -> bool:
        """
        Initialize a service action.

        :param service_name: The name of the service.
        :param service_type: The type of the service.
        :return: True if initialization is successful, False otherwise.
        """
        self.service_action = ServiceAction(
            node=self.service_node,
            client=service_name,
            service_type=service_type
        )

        return self.service_action.get_init_success()
    
        
    def execute_service_call(self,srv_values) -> Tuple[bool, Any]:
        """
        Execute a service call.

        :param srv_values: A dictionary containing service values.
        """
        if self.init_service_action(srv_values["service_name"],srv_values["service_type"]):
            print(self.service_action.service_req_dict_implicit)

            success_set_values = self.set_srv_values(srv_values)

            print(self.service_action.service_req_dict)
            print(success_set_values)

            success_execute = self.service_action.execute()
            service_response = self.service_action.service_res_dict
            print(success_execute)    

            return success_execute, service_response
        else:
            return False

    def set_srv_values(self, openai_response) -> bool:
        request_data = json.loads(openai_response['request'])
        request_data = convert_to_ordered_dict(request_data)

        key_value_list: list = get_key_value_pairs_from_dict(request_data)


        # if list is empty we can return early
        if not key_value_list:
            return True
        try:
            # iterate through the list of key value pairs. Look for references to earlier respones.
            for item in key_value_list:
                # unfold key value pairs
                for key, value in item.items():
                    process_success = self.service_action.set_srv_req_dict_value_from_key(path_key=key,new_value=value)
                    if not process_success:
                        return False
            return process_success
        except:
            return False


if __name__ == "__main__":
    pass