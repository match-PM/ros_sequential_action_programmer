from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer, SET_IMPLICIT_SRV_DICT, LOG_AT_END, LOG_NEVER


class RsapExecutionWorkerSignals(QObject):
    signal = pyqtSignal()

class RsapExecutionWorker(QRunnable):
    def __init__(self, action_sequence_builder:RosSequentialActionProgrammer):
        super().__init__()
        self.action_sequence_builder = action_sequence_builder
        self.signals = RsapExecutionWorkerSignals()

    @pyqtSlot()
    def run(self):
        success = self.action_sequence_builder.execute_current_action(log_mode=LOG_AT_END,
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
        success, index = self.action_sequence_builder.execute_action_list(index_start=self.index)
        self.signals.signal.emit()