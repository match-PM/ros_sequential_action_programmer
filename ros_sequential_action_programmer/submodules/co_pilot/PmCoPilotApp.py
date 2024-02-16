import sys
import time
import os
from rclpy.node import Node

import speech_recognition as sr
import pyttsx3
from pydub import AudioSegment
from pydub.playback import play
import io
import tempfile
from openai import OpenAI

from PyQt6.QtGui import QIcon, QAction, QFont, QPalette, QColor, QTextCursor, QTextBlockFormat, QTextCharFormat
from PyQt6.QtWidgets import QLabel, QWidgetAction, QMenuBar, QDialog, QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton, QStyleFactory
from PyQt6.QtCore import QEvent, QObject, pyqtSignal, QThread, QSize, QRect, QPoint

from ros_sequential_action_programmer.submodules.co_pilot.AssistantAPI import AssistantAPI


class SpeechWorker(QObject):
    
    # textReceived = pyqtSignal(str)
    errorOccurred = pyqtSignal(str)
    finished = pyqtSignal(str)

    def __init__(self, service_node: Node):
        super().__init__()
        self.service_node = service_node
        
        self.client = OpenAI()
        self.client.api_key = os.environ["OPENAI_API_KEY"]

    def run(self):
        # Initialize the recognizer
        r = sr.Recognizer()
        with sr.Microphone() as source:
            self.service_node.get_logger().info("Speech Recognition initialized!")
            try:
                audio = r.listen(source,timeout=2,phrase_time_limit=20)
                
                # Convert the audio data to an audio file format (e.g., WAV)
                audio_data = audio.get_wav_data()
                # Use io.BytesIO to create a file-like object from the byte data
                wav_audio = io.BytesIO(audio_data)

                # Load the audio file using pydub (interprets the BytesIO object as a WAV file)
                sound = AudioSegment.from_file(wav_audio, format="wav")

                # Convert the audio to MP3 and save to an in-memory file
                mp3_audio = io.BytesIO()
                sound.export(mp3_audio, format="mp3")
                mp3_audio.seek(0) 

                with open("output.mp3", "wb") as mp3_file:
                    mp3_file.write(mp3_audio.getvalue())

                audio_file = open("output.mp3", "rb")

                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file,
                    language="en"
                )

                # Use pydub to play back the audio file
                # sound = AudioSegment.from_file(audio_file, format="wav")
                # play(sound)  # This plays the sound to the user

                self.service_node.get_logger().info(f"Audio recorded {transcript}")

                # text = r.recognize_whisper(audio)
                # self.textReceived.emit(transcript.text)

            except sr.UnknownValueError:
                self.errorOccurred.emit("Speech Recognition could not understand audio")
            except sr.RequestError as e:
                self.errorOccurred.emit(f"Could not request results from Speech Recognition service; {e}")

        self.finished.emit(transcript.text)
        
class MessageWorker(QObject):
    finished = pyqtSignal(str)  # Signal to emit the response
    update_status = pyqtSignal(str)  # Signal to emit status updates

    def __init__(self, assistantAPI, message, service_node):
        super().__init__()
        self.assistantAPI = assistantAPI
        self.message = message
        self.service_node = service_node

    def run(self):
        # Process the message
        self.assistantAPI.add_message(self.message)
        self.assistantAPI.run_assistant()

        dot_state = 0
        animation_sequence = ["", ".", "..", "..."]

        while self.assistantAPI.retrieve_status() not in ["completed", "failed", "requires_action"]:
            status = self.assistantAPI.retrieve_status()
            # self.service_node.get_logger().info(f"Status 1 {status}")            
            animated_status = status + animation_sequence[dot_state]
            self.update_status.emit(animated_status)
            
            # Cycle through the animation sequence
            dot_state = (dot_state + 1) % len(animation_sequence)

            time.sleep(1)

        self.update_status.emit(self.assistantAPI.retrieve_status())

        if self.assistantAPI.retrieve_status() == "requires_action":
            # self.service_node.get_logger().info("Function call")
            self.assistantAPI.execute_function()
            self.assistantAPI.output_function_to_assistant()
        else:
            self.service_node.get_logger().info("No function call")

        while self.assistantAPI.retrieve_status() not in ["completed", "failed", "requires_action"]:
            # self.service_node.get_logger().info(f"Status 2 {status}")
            status = self.assistantAPI.retrieve_status()

            animated_status = status + animation_sequence[dot_state]
            self.update_status.emit(animated_status)
            
            # Cycle through the animation sequence
            dot_state = (dot_state + 1) % len(animation_sequence)
            time.sleep(1)

        self.update_status.emit(self.assistantAPI.retrieve_status())
        try:
            response = self.assistantAPI.retrieve_response()
        except:
            response = "No message received!"
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

    def clear(self) -> None:
        return super().clear()
    
class ConfirmationDialog(QDialog):
    def __init__(self, question, parent=None):
        super().__init__(parent)

        self.setWindowTitle("User Confirmation")
        self.setFixedSize(200, 100) 

        # Layout
        layout = QVBoxLayout(self)

        # Add a label
        label = QLabel(question)
        layout.addWidget(label)

        # Yes and No buttons
        self.yes_button = QPushButton("Yes", self)
        self.yes_button.clicked.connect(self.accept)
        layout.addWidget(self.yes_button)

        self.no_button = QPushButton("No", self)
        self.no_button.clicked.connect(self.reject)
        layout.addWidget(self.no_button)

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

class PmCoPilotApp(QMainWindow):
    def __init__(self, service_node:Node):
        super().__init__()

        self.setupUI()
        self.apply_style()

        self.update_status_display("Initialization of Assisstant")

        self.speech_engine = pyttsx3.init()

        self.service_node = service_node

        self.assistantAPI: AssistantAPI = None

        self.init_thread = QThread()
        self.init_worker = InitializationWorker(service_node)
        self.init_worker.moveToThread(self.init_thread)

        self.init_worker.initializationComplete.connect(self.onInitializationComplete)
        self.init_thread.started.connect(self.init_worker.run)
        
        # Start the thread
        self.init_thread.start()

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

        # Add a listen button
        self.listen_button = QPushButton("Listen")
        self.listen_button.setFont(QFont("Helvetica", 12))
        self.listen_button.clicked.connect(self.startSpeechRecognition)
        self.layout.addWidget(self.listen_button)

        # Add menu bar
        menu_bar = self.menuBar()  # This will create a menu bar

        # Create an "Edit" menu
        edit_menu = menu_bar.addMenu("Edit")

        # Add "New Thread" action
        new_thread_action = QAction("New Thread", self)
        new_thread_action.triggered.connect(self.start_new_thread)  # You need to define this method
        edit_menu.addAction(new_thread_action)

        # Add "New Assistant" action
        new_assistant_action = QAction("Update Assistant", self)
        new_assistant_action.triggered.connect(self.update_assistant)  # You need to define this method
        edit_menu.addAction(new_assistant_action)


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

    def onInitializationComplete(self, assistantAPI):
        # Once initialization is complete, you can use assistantAPI and continue with the setup
        self.assistantAPI = assistantAPI
        self.init_thread.quit()
        self.init_thread.wait()

        self.update_status_display("Assistant ready for your input!")

    def start_new_thread(self):
        self.service_node.get_logger().info("New thread will be started!")
        try:
            self.assistantAPI.create_new_thread()
        except Exception as e:
            self.service_node.get_logger().error(f"New thread could not be created: Error {e}!")
        
        self.chat_history.clear()
        self.update_status_display("New thread started! Assistant ready for your input!")
        self.service_node.get_logger().info("New thread started!")

    def update_assistant(self):
        self.service_node.get_logger().info("Assistant will be updated with new file")
        try:
            self.assistantAPI.update_assistant()
        except Exception as e:
            self.service_node.get_logger().error(f"Assistant could not be updated: Error {e}!")
        
        self.chat_history.clear()
        self.update_status_display("New file added to the assistant! Assistant ready for your input!")
        self.service_node.get_logger().info("Assistant updated!")

    def startSpeechRecognition(self):
        self.service_node.get_logger().info(f"start speech recognition")
        self.thread = QThread()
        self.worker = SpeechWorker(self.service_node)
        self.worker.moveToThread(self.thread)

        # Connect signals
        self.thread.started.connect(self.worker.run)

        # self.worker.textReceived.connect(self.handleSpeechInput)
        self.worker.errorOccurred.connect(self.handleError)
        self.worker.finished.connect(self.thread.quit)  # Ensure worker emits a finished signal when done
        self.worker.finished.connect(self.worker.deleteLater)  # Cleanup worker after finishing
        self.thread.finished.connect(self.thread.deleteLater)  # Cleanup thread after it finishes

        self.worker.finished.connect(self.handleSpeechInput)

        self.thread.start()

    def handleSpeechInput(self, message):
        # Process the recognized text as input
        self.service_node.get_logger().info(f"Recognized text: {message}")
        self.chat_history.append_question("User: ", message)
        self.start_message_processing_thread(message)
        self.message_input.clear()

    def handleError(self, error_message):
        # Handle errors here
        self.service_node.get_logger().info(f"Error: {error_message}")

    def onProcessedMessage(self, response):
        self.client = OpenAI()
        self.client.api_key = os.environ["OPENAI_API_KEY"]
        
        speech_repsonse = self.client.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=response
        )
        # Save the speech response to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as tmpfile:
            tmpfile.write(speech_repsonse.content)
            tmp_filename = tmpfile.name

        # Load the temporary file into an AudioSegment
        sound = AudioSegment.from_file(tmp_filename)

        # Play the audio
        play(sound)
        os.remove(tmp_filename)
        # Use pyttsx3 to speak out the response
        # self.speech_engine.say(response)
        # self.speech_engine.runAndWait()

    def on_send_clicked(self):
        message = self.message_input.toPlainText()
        self.chat_history.append_question("User: ", message)
        self.start_message_processing_thread(message)
        self.message_input.clear()


    def start_message_processing_thread(self, message):
        self.thread = QThread()
        self.worker = MessageWorker(self.assistantAPI, message, self.service_node)
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
        # self.onProcessedMessage(response=response)

    def update_status_display(self, status_message):
        self.status_label.setText(f"Status: {status_message}")


    def closeEvent(self, event: QEvent):
        # Custom actions to perform when the window is closing
        self.cleanup()
        # if self.thread is not None and self.thread.isRunning():
        #     self.thread.quit()  # Request the thread to quit
        #     self.thread.wait()  # Wait for the thread to finish
        event.accept()  # Proceed with the window closing

    def cleanup(self):
        dialog = ConfirmationDialog(question="Do you want to delete the current assistant?")
        result = dialog.exec()

        #self.service_node.get_logger().info(f"Deleting File: {self.assistantAPI.delete_files()}")
        
        if result == QDialog.DialogCode.Accepted:
            self.assistantAPI.delete_assistant()
            self.service_node.get_logger().info("Deleting Assistant before closing the application.")

        self.service_node.get_logger().info("Application closed.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = PmCoPilotApp()
    mainWin.show()
    sys.exit(app.exec())