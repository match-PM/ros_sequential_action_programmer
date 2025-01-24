from PyQt6.QtCore import Qt, QByteArray, pyqtSignal, QObject

class IncomingLogSignal(QObject):
    """Signal to notify that a new log has been received."""
    signal = pyqtSignal(str,int)


class ExecutionStatusSignal(QObject):
    """Signal to notify that the execution of the sequence has been started."""
    signal = pyqtSignal(bool)

class CurrentActionSignal(QObject):
    """Signal to notify that the execution of the sequence has been started."""
    signal = pyqtSignal(int)