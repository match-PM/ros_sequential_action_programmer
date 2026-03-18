from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer, SET_IMPLICIT_SRV_DICT
from ros_sequential_action_programmer.submodules.action_classes.ActionBaseClass import ActionBaseClass
from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import ExecutionLog

class RsapExecutionWorkerSignals(QObject):
    signal = pyqtSignal()

class RsapExecutionWorker(QRunnable):
    def __init__(self, action_sequence_builder:RosSequentialActionProgrammer):
        super().__init__()
        self.action_sequence_builder = action_sequence_builder
        self.signals = RsapExecutionWorkerSignals()

    @pyqtSlot()
    def run(self):
        success = self.action_sequence_builder.execute_current_action(log_mode=self.action_sequence_builder.config.execution_log.get_execution_log_mode(),
                                                                      shift_action=True)
        self.signals.signal.emit()

class RsapExecutionRunWorker(QRunnable):
    def __init__(self, action_sequence_builder:RosSequentialActionProgrammer, index:int):
        super().__init__()
        self.action_sequence_builder = action_sequence_builder
        self.signals = RsapExecutionWorkerSignals()
        self.index  = index

    @pyqtSlot()
    def run(self):
        success, index = self.action_sequence_builder.execute_action_list(index_start=self.index,
                                                                          log_mode=self.action_sequence_builder.config.execution_log.get_execution_log_mode())
        self.signals.signal.emit()