"""Regression tests for response reference assignment and the reference picker."""

import os
from copy import deepcopy
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt6.QtWidgets import QApplication
from example_interfaces.srv import AddTwoInts

from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import (
    ActionBaseClass, find_fields_by_type,
)
from ros_sequential_action_programmer.submodules.action_classes.ParameterReference import (
    ActionResponseParameterReference, SeqParameterReference,
)
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from ros_sequential_action_programmer.submodules.rsap_modules.ActionParameterValueManager import (
    ActionParameterValueManager,
)
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameter
from ros_sequential_action_programmer.submodules.rsap_modules.errors import EvaluateActionReferenceError
from ros_sequential_action_programmer.submodules.RsapApp_submodules.ActionParameterValueManagerDialog import (
    ActionParameterValueManagerDialog,
)


class DictionaryAction(ActionBaseClass):
    def __init__(self, name, request=None, response=None, response_types=None):
        super().__init__(Mock(), name, '')
        self.request_dict = request or {}
        self.response_dict = response or {}
        self.response_types = response_types or {}

    def get_request_as_ordered_dict(self):
        return self.request_dict

    def get_response_as_ordered_dict(self):
        return self.response_dict

    def get_response_type(self):
        return self.response_types


def reference(target, source, value_key='value', response_key='result'):
    target.get_references().add_reference(
        ActionResponseParameterReference(value_key, response_key, source)
    )


def manager_for(actions):
    manager = ActionParameterValueManager.__new__(ActionParameterValueManager)
    manager._sequence_list = actions
    return manager


def test_response_reference_uses_latest_ros_response():
    source = ServiceAction.__new__(ServiceAction)
    ActionBaseClass.__init__(source, Mock(), 'sum', '')
    source.response = AddTwoInts.Response(sum=12)
    target = ServiceAction.__new__(ServiceAction)
    ActionBaseClass.__init__(target, Mock(), 'consumer', '')
    target.request = AddTwoInts.Request(a=1, b=2)
    reference(target, source, 'a', 'sum')
    evaluated = target.evaluate_references()
    assert evaluated == {'a': 12, 'b': 2}
    assert target.request.a == 1
    target.set_request_from_dict(evaluated)
    assert target.request.a == 12
    source.response.sum = 23
    assert target.evaluate_references()['a'] == 23


@pytest.mark.parametrize(('source_value', 'target_value', 'expected'), [
    (False, True, False), (0, 9, 0), ('', 'old', ''),
    ({'data': 5}, 0, 5), (5, {'data': 0}, {'data': 5}),
    ({'data': 5}, {'data': 0}, {'data': 5}),
    ({'position': {'x': 4.0}}, {'position': {'x': 0.0}}, {'position': {'x': 4.0}}),
])
def test_reference_values_and_message_shapes(source_value, target_value, expected):
    source = DictionaryAction('source', response={'result': source_value})
    target = DictionaryAction('target', request={'nested': {'value': target_value}})
    original = deepcopy(target.request_dict)
    original_response = deepcopy(source.response_dict)
    reference(target, source, 'nested.value')
    result = target.evaluate_references()
    assert result['nested']['value'] == expected
    assert target.request_dict == original
    if isinstance(expected, dict):
        result['nested']['value'].clear()
        assert source.response_dict == original_response


@pytest.mark.parametrize('value', [False, 0, '', {'data': 4}])
def test_sequence_parameters_are_assigned(value):
    target = DictionaryAction('target', request={'value': 10})
    target.get_references().add_reference(
        SeqParameterReference('value', SeqParameter('parameter', 'int64', value))
    )
    expected = value['data'] if isinstance(value, dict) else value
    assert target.evaluate_references()['value'] == expected


@pytest.mark.parametrize(('source', 'target_key', 'response_key'), [
    (None, 'value', 'result'),
    (DictionaryAction('source'), 'value', 'missing'),
    (DictionaryAction('source', response={'result': 1}), 'missing', 'result'),
])
def test_broken_references_report_evaluation_error(source, target_key, response_key):
    target = DictionaryAction('target', request={'value': 0})
    reference(target, source, target_key, response_key)
    with pytest.raises(EvaluateActionReferenceError, match=target_key):
        target.evaluate_references()
    assert target.request_dict == {'value': 0}


def test_picker_lists_each_compatible_key_once_and_stops_before_target():
    source = DictionaryAction('source', response_types={
        'small': {'type': 'int8'},
        'large': {'type': 'int64'},
        'nested': {'type': 'custom/msg/Result', 'fields': {'value': {'type': 'int16'}}},
    })
    target = DictionaryAction('target')
    manager = manager_for([source, target, source])
    entries = manager.get_comp_res_ref_keys(target, 'int16').get_list()
    assert [(e.action_index, e.key) for e in entries] == [(0, 'small'), (0, 'nested.value')]
    # int64 is not assignable to int8 through uint8's wider compatibility.
    assert [e.key for e in manager.get_comp_res_ref_keys(target, 'int8').get_list()] == ['small']
    assert manager.get_comp_res_ref_keys(source, 'int16').get_list() == []


def test_custom_message_types_match_exactly_and_arrays_are_not_scalar_references():
    types = {
        'pose': {'type': 'geometry_msgs/msg/Pose', 'fields': {'x': {'type': 'double'}}},
        'values': {'type': 'double', 'is_array': True},
        'poses': {'type': 'geometry_msgs/msg/Pose', 'is_array': True,
                  'fields': {'x': {'type': 'double'}}},
    }
    assert find_fields_by_type(types, 'geometry_msgs/msg/Pose') == ['pose']
    assert find_fields_by_type(types, 'double') == ['pose.x']


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def make_dialog(source, target):
    manager = manager_for([source, target])
    manager.seq_parameter_manager = Mock()
    manager.seq_parameter_manager.get_is_initialized.return_value = False
    manager.get_value_headers = Mock(return_value=[])
    return ActionParameterValueManagerDialog(target, manager, 'int16', target.request_dict, 'value')


def test_dialog_rows_are_unique_and_selected_reference_is_assigned(app):
    source = DictionaryAction('source', response={'result': 17},
                              response_types={'result': {'type': 'int8'}})
    target = DictionaryAction('target', request={'value': 0})
    dialog = make_dialog(source, target)
    dialog.tabs.setCurrentWidget(dialog.references_tab)
    dialog.reference_dropdown.setCurrentText(dialog.RERFERENCE_TAB_ACTION_RESPONSES)
    assert dialog.reference_val_tree.topLevelItemCount() == 1
    dialog.reference_val_tree.setCurrentItem(dialog.reference_val_tree.topLevelItem(0))
    dialog.on_ok()
    assert target.evaluate_references() == {'value': 17}
    dialog.deleteLater()


def test_empty_reference_source_does_not_reuse_old_selection(app):
    source = DictionaryAction('source', response_types={'result': {'type': 'int8'}})
    target = DictionaryAction('target', request={'value': 0})
    dialog = make_dialog(source, target)
    dialog.tabs.setCurrentWidget(dialog.references_tab)
    dialog.reference_dropdown.setCurrentText(dialog.RERFERENCE_TAB_ACTION_RESPONSES)
    dialog.reference_val_tree.setCurrentItem(dialog.reference_val_tree.topLevelItem(0))
    dialog.reference_dropdown.setCurrentText(dialog.RERFERENCE_TAB_SEQUENCE_PARAMETERS)
    dialog.on_ok()
    assert target.get_references().get_reference_list() == []
    dialog.deleteLater()
