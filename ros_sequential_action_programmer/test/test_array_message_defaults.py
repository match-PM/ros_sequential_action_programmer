"""New array messages must respect their declared ROS defaults."""

import os

import pytest
from PyQt6.QtWidgets import QApplication
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionParameterWidget import (
    ArrayEditDialog,
)
from ros_sequential_action_programmer.submodules.action_classes.ros_messages_functions import (
    field_type_map_recursive_with_msg_type,
)


@pytest.fixture(scope='module')
def qt_app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.mark.parametrize('message_type', [
    'geometry_msgs/msg/Quaternion',
    'geometry_msgs/msg/Pose',
    'std_msgs/msg/Float64MultiArray',
    'active_alignment_interfaces/msg/AlignTopic',
])
def test_added_message_uses_declared_defaults(qt_app, message_type):
    if message_type.startswith('active_alignment_interfaces/'):
        pytest.importorskip('active_alignment_interfaces.msg')
    message_class = get_message(message_type)
    values = []
    dialog = ArrayEditDialog(None, values, {
        'type': message_type,
        'fields': field_type_map_recursive_with_msg_type(message_class),
        'is_array': True,
    })
    try:
        dialog.add_element()
        dialog.add_element()
        expected = message_to_ordereddict(message_class())
        assert values == [expected, expected]
        assert values[0] is not values[1]
        if message_type.endswith('/AlignTopic'):
            assert values[0]['weight_factor'] == 1.0
            assert values[0]['minimize_not_maximize'] is True
        if message_type.endswith('/Pose'):
            values[0]['orientation']['w'] = 0.5
            assert values[1]['orientation']['w'] == 1.0
    finally:
        dialog.close()
        dialog.deleteLater()
