"""Verify complete log snapshots and execution-triggered publication."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from rsap_interfaces.msg import RsapLogs
from rclpy.serialization import serialize_message, deserialize_message

from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import (
    RosSequentialActionProgrammer,
)


def make_action(name, log_entry):
    action = Mock()
    action.get_name.return_value = name
    action.get_uuid.return_value = f'uuid-{name}'
    action.get_log_entry.side_effect = lambda: log_entry.copy()
    action.clear_log_entry.side_effect = log_entry.clear
    return action


@pytest.fixture
def programmer():
    instance = RosSequentialActionProgrammer.__new__(RosSequentialActionProgrammer)
    instance._action_logs_publisher = Mock()
    instance.action_list = []
    instance.action_log = []
    instance.action_sequence_log = {}
    instance.node = Mock()
    instance.node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=42)
    instance.rsap_file_manager = Mock()
    instance.rsap_file_manager.get_sequence_name.return_value = 'Test sequence'
    return instance


def latest_logs(programmer):
    return programmer._action_logs_publisher.publish.call_args.args[0].logs


def test_snapshot_contains_all_nonempty_logs_in_sequence_order(programmer):
    first_log = {'success': True, 'response': {'values': [1, 2]}}
    programmer.action_list = [
        make_action('Move', first_log),
        make_action('Unused', {}),
        make_action('Inspect', {'success': False}),
    ]
    for index in (0, 2):
        action = programmer.action_list[index]
        programmer.append_action_log(index, action.get_name(),
                                     dict(action.get_log_entry(), uuid=action.get_uuid()))
    programmer.publish_action_logs()
    logs = latest_logs(programmer)
    assert [entry.action_index for entry in logs] == [0, 2]
    assert [entry.action_name for entry in logs] == ['Move', 'Inspect']
    assert [entry.action_uuid for entry in logs] == ['uuid-Move', 'uuid-Inspect']
    assert [entry.success for entry in logs] == [True, False]
    assert json.loads(logs[0].log_json)['response'] == first_log['response']
    assert json.loads(logs[1].log_json)['success'] is False
    message = programmer._action_logs_publisher.publish.call_args.args[0]
    assert message.sequence_name == 'Test sequence'
    assert message.stamp.sec == 42
    assert deserialize_message(serialize_message(message), RsapLogs) == message
    first_log['success'] = False
    programmer.publish_action_logs()
    assert len(latest_logs(programmer)) == 2
    assert latest_logs(programmer)[0].success is True
    assert json.loads(latest_logs(programmer)[0].log_json)['success'] is True


def test_clearing_logs_publishes_empty_list(programmer):
    programmer.action_list = [make_action('Move', {'success': True})]
    programmer.clear_all_log_entries()
    assert latest_logs(programmer) == []
    assert programmer.action_list[0].get_log_entry() == {}


def test_snapshot_without_a_loaded_sequence(programmer):
    programmer.rsap_file_manager.get_sequence_name.return_value = None
    programmer.publish_action_logs()
    message = programmer._action_logs_publisher.publish.call_args.args[0]
    assert message.sequence_name == ''
    assert message.logs == []


@pytest.mark.parametrize('outcome', [True, False, RuntimeError('Interrupted')])
def test_execution_publishes_updated_logs_even_on_failure(programmer, outcome):
    log_entry = {}
    action = make_action('Move', log_entry)

    def execute(_interrupt):
        log_entry['success'] = outcome is True
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    action.execute.side_effect = execute
    programmer.action_list = [action]
    programmer.current_action_index = 0
    programmer.config = Mock()
    programmer.config.step_execution_behavior.get_value.return_value = False
    programmer.signal_execution_status = SimpleNamespace(signal=Mock())
    programmer._send_tick = Mock()
    programmer.action_log = []

    assert programmer.execute_current_action() is (outcome is True)
    programmer._action_logs_publisher.publish.assert_called_once()
    assert len(latest_logs(programmer)) == 1
    entry = latest_logs(programmer)[0]
    assert entry.action_index == 0
    assert entry.action_name == 'Move'
    assert entry.success is (outcome is True)
    assert json.loads(entry.log_json)['success'] is (outcome is True)
    programmer.signal_execution_status.signal.emit.assert_called_with(False)


def configure_execution(programmer, action):
    programmer.action_list = [action]
    programmer.current_action_index = 0
    programmer.config = Mock()
    programmer.config.step_execution_behavior.get_value.return_value = False
    programmer.signal_execution_status = SimpleNamespace(signal=Mock())
    programmer._send_tick = Mock()


def test_end_export_is_single_nonempty_snapshot(programmer, tmp_path):
    from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import ExecutionLog
    programmer.rsap_file_manager.get_folder_path.return_value = str(tmp_path)
    log = {}
    action = make_action('Move', log)
    action.execute.side_effect = lambda _: log.update(success=True, execution_time='0:00:02') or True
    configure_execution(programmer, action)
    assert programmer.execute_current_action(log_mode=ExecutionLog.LOG_AT_END)
    paths = list(tmp_path.rglob('*.json'))
    assert len(paths) == 1
    exported = json.loads(paths[0].read_text())['action_log']
    assert len(exported) == 1
    assert json.loads(latest_logs(programmer)[0].log_json) == exported[0]['0_Move']
    assert programmer.action_log == exported


def test_exports_never_overwrite_or_consume_history(programmer, tmp_path, monkeypatch):
    import importlib
    module = importlib.import_module(RosSequentialActionProgrammer.__module__)
    clock = Mock()
    clock.now.return_value.strftime.return_value = 'same_timestamp'
    monkeypatch.setattr(module, 'datetime', clock)
    programmer.rsap_file_manager.get_folder_path.return_value = str(tmp_path)
    programmer.append_action_log(0, 'Move', {'success': True})
    assert programmer.save_action_sequence_log()
    assert programmer.save_action_sequence_log()
    paths = list(tmp_path.rglob('*.json'))
    assert len(paths) == 2
    assert all(len(json.loads(path.read_text())['action_log']) == 1 for path in paths)
    assert len(programmer.action_log) == 1


def test_repeated_actions_keep_independent_history_and_clear_everything(programmer):
    log = {'success': True, 'response': {'values': [1]}}
    programmer.append_action_log(0, 'Move', log)
    log['response']['values'].append(2)
    programmer.append_action_log(0, 'Move', log)
    programmer.publish_action_logs()
    records = [json.loads(entry.log_json) for entry in latest_logs(programmer)]
    assert [record['response']['values'] for record in records] == [[1], [1, 2]]
    assert records[0]['execution_id'] != records[1]['execution_id']
    programmer._update_action_sequence_log()
    programmer.clear_all_log_entries()
    assert programmer.action_log == []
    assert programmer.action_sequence_log == {}
    assert latest_logs(programmer) == []


def test_exception_without_action_log_still_records_failed_attempt(programmer):
    action = make_action('Move', {})
    action.execute.side_effect = RuntimeError('Interrupted')
    configure_execution(programmer, action)
    assert not programmer.execute_current_action()
    entry = json.loads(latest_logs(programmer)[0].log_json)
    assert entry['success'] is False
    assert 'execution_time' in entry


@pytest.mark.parametrize('ending', ['complete', 'failure', 'inactive_last', 'stop', 'breakpoint'])
def test_sequence_exports_complete_or_partial_history(programmer, tmp_path, monkeypatch, ending):
    from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import ExecutionLog
    monkeypatch.setattr('time.sleep', lambda _: None)
    programmer.rsap_file_manager.get_folder_path.return_value = str(tmp_path)
    first_log, second_log = {}, {}
    first, second = make_action('Move', first_log), make_action('Inspect', second_log)
    configure_execution(programmer, first)
    programmer.action_list = [first, second]
    programmer.config.execution_behavior.get_value.return_value = False
    programmer.signal_current_action = SimpleNamespace(signal=Mock())
    programmer.has_current_action_breakpoint = lambda: ending == 'breakpoint' and programmer.current_action_index == 1
    second.is_active.return_value = ending != 'inactive_last'

    def execute_first(_interrupt):
        first_log.update(success=ending != 'failure', execution_time='0:00:02')
        if ending == 'stop':
            programmer.set_stop_execution()
        return ending != 'failure'

    first.execute.side_effect = execute_first
    second.execute.side_effect = lambda _: second_log.update(success=True, execution_time='0:00:03') or True
    success, _ = programmer.execute_action_list(0, log_mode=ExecutionLog.LOG_AT_END)
    assert success is (ending != 'failure')
    paths = list(tmp_path.rglob('*.json'))
    assert len(paths) == 1
    history = json.loads(paths[0].read_text())['action_log']
    assert len(history) == (2 if ending == 'complete' else 1)
    assert [json.loads(record.log_json) for record in latest_logs(programmer)] == [
        next(iter(record.values())) for record in history]


def test_reexecution_keeps_original_index_and_prior_results(programmer):
    log = {}
    action = make_action('Move', log)
    configure_execution(programmer, action)
    programmer.action_list.append(make_action('Unused', {}))
    programmer.config.step_execution_behavior.get_value.return_value = True
    durations = iter(['0:00:02', '0:00:04'])
    action.execute.side_effect = lambda _: log.update(success=True, execution_time=next(durations)) or True
    for _ in range(2):
        programmer.current_action_index = 0
        assert programmer.execute_current_action()
        assert programmer.current_action_index == 1
    records = latest_logs(programmer)
    assert [entry.action_index for entry in records] == [0, 0]
    assert [json.loads(entry.log_json)['execution_time'] for entry in records] == ['0:00:02', '0:00:04']


@pytest.mark.parametrize('pause', ['breakpoint', 'stop', 'failure'])
def test_resuming_run_preserves_history_in_topic_and_export(programmer, tmp_path, monkeypatch, pause):
    from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import ExecutionLog
    monkeypatch.setattr('time.sleep', lambda _: None)
    programmer.rsap_file_manager.get_folder_path.return_value = str(tmp_path)
    actions = []
    paused = False

    for index in range(7):
        log = {}
        action = make_action(f'Action {index}', log)

        def execute(_interrupt, index=index, log=log):
            nonlocal paused
            success = not (pause == 'failure' and index == 3 and not paused)
            log.update(success=success, execution_time='0:00:01')
            if index == 3 and pause in ('stop', 'failure') and not paused:
                paused = True
                if pause == 'stop':
                    programmer.set_stop_execution()
            return success

        action.execute.side_effect = execute
        actions.append(action)

    configure_execution(programmer, actions[0])
    programmer.action_list = actions
    programmer.config.execution_behavior.get_value.return_value = False
    programmer.signal_current_action = SimpleNamespace(signal=Mock())
    programmer.has_current_action_breakpoint = lambda: (
        pause == 'breakpoint' and programmer.current_action_index == 4)

    success, _ = programmer.execute_action_list(0, log_mode=ExecutionLog.LOG_AT_END)
    assert success is (pause != 'failure')
    assert len(programmer.action_log) == 4
    initial_ids = [json.loads(entry.log_json)['execution_id'] for entry in latest_logs(programmer)]

    # Run again from the breakpoint, next action after stop, or failed action.
    resume_index = 3 if pause == 'failure' else 4
    success, _ = programmer.execute_action_list(resume_index, log_mode=ExecutionLog.LOG_AT_END)
    assert success
    expected_indices = [0, 1, 2, 3] + list(range(resume_index, 7))
    logs = latest_logs(programmer)
    assert [entry.action_index for entry in logs] == expected_indices
    assert [json.loads(entry.log_json)['execution_id'] for entry in logs[:4]] == initial_ids
    exports = [json.loads(path.read_text())['action_log'] for path in tmp_path.rglob('*.json')]
    assert sorted(len(records) for records in exports) == [4, len(expected_indices)]
    complete = max(exports, key=len)
    assert [next(iter(record.values())) for record in complete] == [
        json.loads(entry.log_json) for entry in logs]

    programmer.clear_all_log_entries()
    assert programmer.action_log == []
    assert latest_logs(programmer) == []
