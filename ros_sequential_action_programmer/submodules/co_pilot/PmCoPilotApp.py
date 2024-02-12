import sys
import time
import json
from rclpy.node import Node

from PyQt6.QtGui import QIcon, QFont, QPalette, QColor, QTextCursor, QTextBlockFormat, QTextCharFormat
from PyQt6.QtWidgets import QLabel, QDialog, QApplication, QMainWindow, QWidget, QVBoxLayout, QTextEdit, QPushButton, QStyleFactory
from PyQt6.QtCore import QEvent, QObject, pyqtSignal, QThread, QSize, QRect, QPoint
from openai import OpenAI
from ros_sequential_action_programmer.submodules.co_pilot.AssistantAPI import AssistantAPI

class InitializationWorker(QObject):
    initializationComplete = pyqtSignal(object)  # Signal to emit when initialization is complete, passing the initialized assistantAPI

    def __init__(self, service_node: Node):
        super().__init__()
        self.service_node = service_node

    def run(self):
        # Initialize assistantAPI here
        assistantAPI = AssistantAPI(self.service_node)
        # Emit signal once initialization is complete
        self.initializationComplete.emit(assistantAPI)

class MessageWorker(QObject):
    finished = pyqtSignal(str)  # Signal to emit the response
    update_status = pyqtSignal(str)  # Signal to emit status updates

    def __init__(self, assistantAPI, message):
        super().__init__()
        self.assistantAPI = assistantAPI
        self.message = message

    def run(self):
        # Process the message
        self.assistantAPI.add_message(self.message)
        self.assistantAPI.run_assistant()

        while self.assistantAPI.retrieve_status() not in ["completed", "failed", "requires_action"]:
            status = self.assistantAPI.retrieve_status()
            self.update_status.emit(status)
            time.sleep(1)

        self.update_status.emit(self.assistantAPI.retrieve_status())

        if self.assistantAPI.retrieve_status() == "requires_action":
            print("Function call")
            self.assistantAPI.execute_function()
            self.assistantAPI.output_function_to_assistant()
        else:
            print("No function call")

        while self.assistantAPI.retrieve_status() not in ["completed", "failed", "requires_action"]:
            self.update_status.emit(self.assistantAPI.retrieve_status())
            time.sleep(1)

        self.update_status.emit(self.assistantAPI.retrieve_status())

        response = self.assistantAPI.retrieve_response()
        self.finished.emit(response)

class ChatDisplay(QTextEdit):
    def __init__(self, parent=None):
        super(ChatDisplay, self).__init__(parent)
        self.setReadOnly(True)
        self.setFont(QFont("Helvetica", 12))

    def append_question(self, speaker, text):
        self._append_text(speaker, text, QColor("#FFA07A"))  # Light Salmon color for questions

    def append_reply(self, speaker, text):
        self._append_text(speaker, text, QColor("#7FFFD4"))  # Aquamarine color for replies

    def _append_text(self,speaker, text, color):
        cursor = self.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        format = QTextCharFormat()
        format.setForeground(color)
        cursor.setCharFormat(format)
        cursor.insertText(speaker + text + '\n')
        self.setTextCursor(cursor)

class ConfirmationDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Delete Assistant")
        self.setFixedSize(200, 100) 

        # Layout
        layout = QVBoxLayout(self)

        # Add a label
        label = QLabel("Do you want to delete the current assistant?")
        layout.addWidget(label)

        # Yes and No buttons
        self.yes_button = QPushButton("Yes", self)
        self.yes_button.clicked.connect(self.accept)
        layout.addWidget(self.yes_button)

        self.no_button = QPushButton("No", self)
        self.no_button.clicked.connect(self.reject)
        layout.addWidget(self.no_button)
        

class PmCoPilotApp(QMainWindow):
    def __init__(self, service_node:Node):
        super().__init__()

        self.setupUI()
        self.apply_style()

        self.update_status_display("Initialization of Assisstant")

        self.service_node = service_node

        self.init_thread = QThread()
        self.init_worker = InitializationWorker(service_node)
        self.init_worker.moveToThread(self.init_thread)

        self.init_worker.initializationComplete.connect(self.onInitializationComplete)
        self.init_thread.started.connect(self.init_worker.run)
        
        # Start the thread
        self.init_thread.start()


    def onInitializationComplete(self, assistantAPI):
        # Once initialization is complete, you can use assistantAPI and continue with the setup
        self.assistantAPI = assistantAPI
        self.init_thread.quit()
        self.init_thread.wait()

        self.update_status_display("Assistant ready for your input!")

    def setupUI(self):
        # Apply a style suitable for dark mode
        self.setStyle(QStyleFactory.create("Fusion"))

        # Set the main window's properties
        self.setWindowTitle("PmCoPilot")
        self.setGeometry(100, 100, 800, 600)
        # self.setWindowIcon(QIcon("path_to_icon.png"))  # Set an Apple-style icon

        # Apply a palette for dark mode
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor("#1e1e1e"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#ffffff"))
        palette.setColor(QPalette.ColorRole.Base, QColor("#2e2e2e"))
        palette.setColor(QPalette.ColorRole.Text, QColor("#ffffff"))
        self.setPalette(palette)

        # Create the central widget and set the layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Create the chat history display
        self.chat_history = ChatDisplay()
        self.layout.addWidget(self.chat_history)

        # Add a label for displaying status
        self.status_label = QLabel("Status: Ready")
        self.status_label.setFont(QFont("Helvetica", 12))
        self.layout.addWidget(self.status_label)

        # Add a text edit for typing messages
        self.message_input = QTextEdit()
        self.message_input.setFont(QFont("Helvetica", 12))
        self.layout.addWidget(self.message_input)

        # Add a send button
        self.send_button = QPushButton("Send")
        self.send_button.setFont(QFont("Helvetica", 12))
        # self.send_button.setIcon(QIcon("path_to_send_icon.png"))  # Optional: Set an icon for the button
        self.send_button.clicked.connect(self.on_send_clicked)
        self.layout.addWidget(self.send_button)

        # Set the minimum size for the window
        self.setMinimumSize(QSize(800, 600))

    def apply_style(self):
        style_sheet = """
            QTextEdit {
                border: 1px solid #444;
                border-radius: 5px;
                padding: 5px;
                background-color: #2e2e2e;
                color: #ffffff;
            }
            QPushButton {
                border: 1px solid #444;
                border-radius: 5px;
                padding: 5px;
                background-color: #5e5e5e;
                color: #ffffff;
            }
            QPushButton:hover {
                background-color: #3e3e3e;
            }
            QLabel {
                color: #ffffff;
            }
        """
        self.setStyleSheet(style_sheet)


    def on_send_clicked(self):
        message = self.message_input.toPlainText()
        self.chat_history.append_question("User: ", message)
        self.start_message_processing_thread(message)
        self.message_input.clear()


    def start_message_processing_thread(self, message):
        self.thread = QThread()
        self.worker = MessageWorker(self.assistantAPI, message)
        self.worker.moveToThread(self.thread)

        # Connect signals and slots
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)

        # Update the GUI based on emitted signals
        self.worker.finished.connect(self.handle_processed_message)
        self.worker.update_status.connect(self.update_status_display)

        self.thread.start()

    def handle_processed_message(self, response):
        self.chat_history.append_reply("OpenAI: ", response)

    def update_status_display(self, status_message):
        self.status_label.setText(f"Status: {status_message}")


    def update_chat_history(self,speaker, message, background_color):
        # Update the chat history with the user's message and the bot's response
        self.chat_history.append(speaker + message)


    def closeEvent(self, event: QEvent):
        # Custom actions to perform when the window is closing
        self.cleanup()
        event.accept()  # Proceed with the window closing

    def cleanup(self):
        dialog = ConfirmationDialog(self)
        result = dialog.exec()

        self.service_node.get_logger().info(f"Deleting File: {self.assistantAPI.delete_files()}")
        
        if result == QDialog.DialogCode.Accepted:
            self.assistantAPI.delete_assistant()
            self.service_node.get_logger().info("Deleting Assistant before closing the application.")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = PmCoPilotApp()
    mainWin.show()
    sys.exit(app.exec())