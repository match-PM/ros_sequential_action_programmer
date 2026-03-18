from PyQt6.QtWidgets import QFileDialog, QMessageBox
import os
from ros_sequential_action_programmer.submodules.rsap_modules.SeqParamterManager import SeqParameterManager

from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer
from ros_sequential_action_programmer.submodules.RsapApp_submodules.SequenceInfoWidget import SequenceInfoWidget
from rclpy.node import Node


class GuiParameterFileActions:
    def __init__(self, parent, 
                 node: Node, 
                 action_sequence_builder: RosSequentialActionProgrammer,
                 sequence_info_widget: SequenceInfoWidget) -> None:
        self.parent = parent
        self.service_node = node
        self.action_sequence_builder = action_sequence_builder
        self.sequence_info_widget = sequence_info_widget

    def load_sequence_parameters_file(self) -> None:
        # log
        self.service_node.get_logger().info("Loading sequence parameters file...")
        file_path, _ = QFileDialog.getOpenFileName(
            self.parent, 
            "Open Sequence Parameters File",
            "",
            f"RSApp Parameter Files (*{SeqParameterManager.FILE_ENDING})"
        )
        self.service_node.get_logger().info(f"Loading sequence parameters from: {file_path}")
        self.action_sequence_builder.rsap_file_manager.seq_parameter_manager.load_file(file_path)
        self.sequence_info_widget.init_values()
        self.action_sequence_builder.reinit_sequence_parameter_values()

    def create_new_sequence_parameters_file(self) -> None:
        """
        Opens a save file dialog to create a new .rsapp.json sequence parameters file.
        Automatically appends the .rsapp.json extension and initializes it with metadata.
        """
        file_path, _ = QFileDialog.getSaveFileName(
            self.parent,
            "Create New Sequence Parameters File",
            "",
            f"RSApp Parameter Files (*{SeqParameterManager.FILE_ENDING})"
        )

        if not file_path:
            return  # user cancelled

        # Ensure correct extension
        if not file_path.endswith(SeqParameterManager.FILE_ENDING):
            file_path += SeqParameterManager.FILE_ENDING

        # Check if file already exists
        if os.path.exists(file_path):
            reply = QMessageBox.question(
                self.parent,
                "Overwrite File?",
                f"The file '{os.path.basename(file_path)}' already exists. Overwrite it?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if reply == QMessageBox.StandardButton.No:
                return
        
        self.service_node.get_logger().info(f"Creating new sequence parameters file at: {file_path}")
        self.action_sequence_builder.rsap_file_manager.reset_sequence_parameter_manager()
        self.action_sequence_builder.rsap_file_manager.seq_parameter_manager.set_file_dir(os.path.dirname(file_path))
        self.action_sequence_builder.rsap_file_manager.seq_parameter_manager.set_file_name(os.path.basename(file_path))
        self.action_sequence_builder.rsap_file_manager.seq_parameter_manager.save_to_file()
        self.sequence_info_widget.init_values()
        self.action_sequence_builder.reinit_sequence_parameter_values()

    def reset_sequence_parameter_manager(self) -> None:
        self.action_sequence_builder.rsap_file_manager.reset_sequence_parameter_manager()
        self.sequence_info_widget.init_values()
        self.action_sequence_builder.reinit_sequence_parameter_values()

    def save_sequence_parameter_manager(self) -> None:
        if self.action_sequence_builder.rsap_file_manager.seq_parameter_manager.get_is_initialized():
            self.action_sequence_builder.rsap_file_manager.seq_parameter_manager.save_to_file()
        else:
            self.create_new_sequence_parameters_file()

        self.sequence_info_widget.init_values()
