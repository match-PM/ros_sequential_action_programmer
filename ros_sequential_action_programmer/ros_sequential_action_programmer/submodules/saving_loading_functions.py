from ament_index_python.packages import get_package_share_directory
import json
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer as PartialRosSequentialActionProgrammer

class RosSequentialActionProgrammer(PartialRosSequentialActionProgrammer):
    def save_all_service_req_res_to_JSON(self: PartialRosSequentialActionProgrammer) -> bool:
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
        data = self.get_all_service_req_res_dict()
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

            self.list_of_memorized_services = []
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