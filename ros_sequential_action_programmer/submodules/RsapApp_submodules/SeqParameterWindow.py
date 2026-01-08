from rosidl_runtime_py.utilities import get_message
from ament_index_python.packages import get_packages_with_prefixes
from rosidl_runtime_py import get_message_interfaces
from PyQt6.QtWidgets import QWidget
from PyQt6.QtWidgets import QWidget, QPushButton, QVBoxLayout, QDialog
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameterManager
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionParameterWidget import ROS2DictEditor
from ros_sequential_action_programmer.submodules.action_classes.ros_messages_functions import field_type_map_recursive_with_msg_type

import json

class SeqParameterManagerDialog(QDialog):
    def __init__(self,seq_parameter_manager:SeqParameterManager,
                 logger=None,
                  parent=None):
        
        super().__init__(parent)

        self.seq_parameter_manager = seq_parameter_manager
        self.setWindowTitle("Widget Dialog")
        self.resize(600, 400)
        
        widget = SeqParameterWidget(self.seq_parameter_manager, logger=logger, parent=self)

        layout = QVBoxLayout(self)
        layout.addWidget(widget)


class SeqParameterWidget(QWidget):
    def __init__(self, 
                seq_parameter_manager:SeqParameterManager,
                logger=None,
                parent=None):
        super().__init__(parent)

        self.seq_parameter_manager = seq_parameter_manager
        self.logger = logger    
        messages_interfaces  = self._get_message_interfaces()

        self.logger.warning(f"Found {(messages_interfaces)} message types.")




        list_dd=SeqParameterWidget._get_message_interfaces()
        print(list_dd)
        my_msg = list_dd[0]
        print(my_msg)
        msg_class = get_message(my_msg)
        print(msg_class.get_fields_and_field_types())

        dict_msg = message_to_ordereddict(msg_class())
        print("DEBUG")
        print(dict_msg)

        msg_dict = json.loads(json.dumps(dict_msg))

        type_test =field_type_map_recursive_with_msg_type(msg_class)

        editor = ROS2DictEditor(types_dict=type_test,
                                values_dict=msg_dict)
        
        layout = QVBoxLayout(self)
        layout.addWidget(editor)
        # Additional UI setup code here
        # msgs = get_message()
        # for pkg, msg_list in msgs.items():
        #     print(pkg, msg_list)

    @staticmethod
    def _get_message_interfaces()->list[str]:
        msgs_interfaces = get_message_interfaces()
        interfaces = []
        for pkg, types in msgs_interfaces.items():
            for t in types:
                interfaces.append(f"{pkg}/{t}")
        return interfaces



if __name__ == "__main__":
    list_dd=SeqParameterWidget._get_message_interfaces()
    print(list_dd)
    my_msg = list_dd[0]
    print(my_msg)
    msg_class = get_message(my_msg)
    print(msg_class.get_fields_and_field_types())

    dict_msg = message_to_ordereddict(msg_class())
    print("DEBUG")
    print(dict_msg)

