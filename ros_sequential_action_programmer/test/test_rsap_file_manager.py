"""Process-file validation must preserve the currently open sequence."""

import json
from unittest.mock import Mock

import pytest

from ros_sequential_action_programmer.submodules.rsap_modules.RsapFileManager import (
    RsapFileManager,
)


@pytest.mark.parametrize('contents', [
    '{', '[]', '{}',
    json.dumps({'metadata': {}, 'parameters': []}),
    json.dumps({'name': 'process'}),
    json.dumps({'name': 'process', 'action_list': {}}),
    json.dumps({'name': None, 'action_list': []}),
])
def test_invalid_file_preserves_current_sequence(tmp_path, contents):
    path = tmp_path / 'invalid.json'
    path.write_text(contents)
    actions = [object()]
    original = actions.copy()
    parameters = Mock()
    manager = RsapFileManager(actions, Mock(), parameters)
    manager._sequence_file_path = '/original/process.rsap.json'
    manager.set_folder_path('/original')
    manager.set_sequence_name('process')

    assert manager.load_from_JSON(str(path)) is False
    assert actions == original
    assert manager.get_action_sequence_file_path() == '/original/process.rsap.json'
    assert manager.get_folder_path() == '/original'
    assert manager.get_sequence_name() == 'process'
    parameters.clear_parameters.assert_not_called()


def test_valid_empty_process_loads(tmp_path):
    path = tmp_path / 'process.rsap.json'
    path.write_text(json.dumps({'name': 'process', 'action_list': []}))
    actions = [object()]
    manager = RsapFileManager(actions, Mock(), Mock())

    assert manager.load_from_JSON(str(path)) is True
    assert actions == []
    assert manager.get_sequence_name() == 'process'
    assert manager.get_action_sequence_file_path() == str(path)
