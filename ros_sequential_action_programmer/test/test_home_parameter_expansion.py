"""HOME expansion must affect execution without changing saved action parameters."""

import importlib
import json
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import (
    ActionBaseClass,
)
from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import (
    ActionResponseParameterReference, SeqParameterReference,
)
from ros_sequential_action_programmer.submodules.action_classes.RosActionAction import (
    RosActionAction,
)
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.action_classes.UserInteractionAction import (
    GUI, UserInteractionAction,
)
from ros_sequential_action_programmer.submodules.rsap_modules.RsapFileManager import RsapFileManager
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameter


def dictionary_action(request):
    action = ActionBaseClass(Mock(), 'test', '')
    action.get_request_as_ordered_dict = Mock(return_value=request)
    return action


def test_nested_parameters_expand_only_home_at_evaluation(monkeypatch):
    request = {
        'path': '$HOME/data/${HOME}/file.json',
        'nested': {'paths': ['$HOME/a', {'data': '${HOME}/b'}]},
        'unchanged': ['$HOMES/a', '$HOME_backup/b', '$USER/c', '~/d', '', 0, False, None],
    }
    original = deepcopy(request)
    action = dictionary_action(request)
    monkeypatch.setenv('HOME', '/home/first')
    result = action.evaluate_references()
    assert result['path'] == '/home/first/data//home/first/file.json'
    assert result['nested'] == {'paths': ['/home/first/a', {'data': '/home/first/b'}]}
    assert result['unchanged'] == original['unchanged']
    assert request == original
    monkeypatch.setenv('HOME', '/home/second')
    assert action.evaluate_references()['nested']['paths'][0] == '/home/second/a'
    assert request == original


@pytest.mark.parametrize('source_type', ['sequence_parameter', 'action_response'])
def test_referenced_paths_expand_without_changing_source(monkeypatch, source_type):
    value = {'nested': ['$HOME/data', '${HOME}/config']}
    original = deepcopy(value)
    target = dictionary_action({'value': {}})
    if source_type == 'sequence_parameter':
        source = SeqParameter('paths', 'custom', value)
        reference = SeqParameterReference('value', source)
        get_source = source.get_value
    else:
        source = dictionary_action({})
        source.get_response_as_ordered_dict = Mock(return_value={'paths': value})
        reference = ActionResponseParameterReference('value', 'paths', source)

        def get_source():
            return source.get_response_as_ordered_dict()['paths']
    target.get_references().add_reference(reference)
    monkeypatch.setenv('HOME', '/home/current')
    assert target.evaluate_references() == {
        'value': {'nested': ['/home/current/data', '/home/current/config']},
    }
    assert get_source() == original
    assert target.get_request_as_ordered_dict() == {'value': {}}


def executable_action(kind, monkeypatch, success):
    node = Mock()
    node.get_node_names.return_value = []
    if kind == 'interaction':
        action = UserInteractionAction(node, GUI, action_text='$HOME/config.json')
        action.request_user_interaction = Mock(return_value=success)
        return action, action.request_user_interaction, 'interaction_text', 'request'

    action_class = ServiceAction if kind == 'service' else RosActionAction
    action = action_class.__new__(action_class)
    ActionBaseClass.__init__(action, node, kind, '')
    # A real string-bearing ROS message exercises conversion and copying;
    # service/action transport is mocked so no ROS servers are required.
    action.request = String(data='$HOME/config.json')
    action.request_dict = action.get_request_as_ordered_dict()
    action.response = None
    action.client = '/test/run'
    action.get_request_type = Mock(return_value={'data': {'type': 'string'}})
    client = Mock()
    client.srv_name = action.client

    if kind == 'service':
        action.service_type = 'test/srv/Request'
        action.service_metaclass = Mock()
        node.create_client.return_value = client
        client.wait_for_service.return_value = success
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.cancelled.return_value = False
        client.call_async.return_value.result.return_value = String(data='done')
        return action, client.call_async, 'data', 'srv_request'

    action.action_type = 'test/action/Request'
    action.metaclass = Mock()
    module = importlib.import_module(RosActionAction.__module__)
    monkeypatch.setattr(module, 'ActionClient', Mock(return_value=client))
    client.wait_for_server.return_value = success
    goal_handle = client.send_goal_async.return_value.result.return_value
    goal_handle.accepted = True
    goal_handle.get_result_async.return_value.result.return_value = SimpleNamespace(
        result=String(data='done'), status=GoalStatus.STATUS_SUCCEEDED,
    )
    return action, client.send_goal_async, 'data', 'request'


@pytest.mark.parametrize('kind', ['service', 'ros_action', 'interaction'])
@pytest.mark.parametrize('success', [True, False])
def test_execution_and_saving_preserve_home_placeholder(
        kind, success, monkeypatch, tmp_path):
    action, send_request, field, log_key = executable_action(kind, monkeypatch, success)
    original = deepcopy(action.get_request_as_ordered_dict())
    for user in ('first', 'second'):
        monkeypatch.setenv('HOME', f'/home/{user}')
        assert action.execute() is success
        expected = f'/home/{user}/config.json'
        if kind == 'interaction':
            send_request.assert_called_with(expected)
        elif success:
            assert send_request.call_args.args[0].data == expected
            assert send_request.call_args.args[0] is not action.request
        else:
            send_request.assert_not_called()
        assert action.get_log_entry()[log_key][field] == expected
        assert action.get_request_as_ordered_dict() == original
        if kind != 'interaction':
            assert action.request_dict == original

    manager = RsapFileManager([action], action.node)
    manager.set_folder_path(str(tmp_path))
    manager.set_sequence_name('portable')
    assert manager.save_to_JSON()
    saved = json.loads((tmp_path / 'portable.rsap.json').read_text())['action_list'][0]
    assert saved['request'] == original
    if kind == 'interaction':
        assert saved['action_text'] == '$HOME/config.json'
