    


import yaml
from ament_index_python import get_package_share_directory
from ros_sequential_action_programmer.submodules import RosSequentialActionProgrammer
from pathlib import Path

# class RecentFilesManager:    
#     """
#     Manages recent files for the ROS Sequential Action Programmer application.
#     """
#     def __init__(self, rasp: RosSequentialActionProgrammer):
#         self.rasp = rasp
#         self.logger = rasp.node.get_logger()
#         self.recent_file = None


#     def load_recent_file(self):
#             """
#             Loads the last opened file from the yaml file.
#             """
#             path = get_package_share_directory('ros_sequential_action_programmer')

#             # Specify the path to your YAML file
#             yaml_file_path = f"{path}/recent_file.yaml"
#             try:
#                 # Read the content of the YAML file
#                 with open(yaml_file_path, 'r') as file:
#                     yaml_content = yaml.safe_load(file)

#                 process_file_path = yaml_content['recent_file'] 
#                 self.rasp.rsap_file_manager.load_from_JSON(process_file_path)
#                 self.recent_file = process_file_path
#                 self.logger.info(f"Loaded recent file: {process_file_path}")
#             except Exception as e:
#                 self.logger.error(f"Error loading recent file: {e}")
#                 self.logger.warn("No recent file found! Skipping loading of recent file!")

#     def set_recent_file(self):
#         """
#         Saves the last opened file to the yaml file.
#         """
#         recent_file_dict = {}
#         recent_file_dict['recent_file'] = self.rasp.rsap_file_manager.get_action_sequence_file_path()
#         path = get_package_share_directory('ros_sequential_action_programmer')
#         # Specify the path to your YAML file
#         yaml_file_path = f"{path}/recent_file.yaml"
#         with open(yaml_file_path, 'w') as file:
#             yaml.dump(recent_file_dict, file, default_flow_style=False)


class RecentFilesManager:    
    """
    Manages recent files for the ROS Sequential Action Programmer application.
    Keeps a list of the last 10 recent files in a YAML file.
    """
    MAX_RECENT_FILES = 10

    def __init__(self, rasp: RosSequentialActionProgrammer):
        self.rasp = rasp
        self.logger = rasp.node.get_logger()
        self.recent_files: list[str] = []  # list of last recent files

        # Path to YAML file
        package_path = get_package_share_directory('ros_sequential_action_programmer')
        self.yaml_file_path = Path(package_path) / "recent_files.yaml"

        # Load existing recent files
        self.load_recent_files()

    # --------------------------
    # Load recent files from YAML
    # --------------------------
    def load_recent_files(self):
        """
        Loads the recent files list from YAML.
        """
        if not self.yaml_file_path.exists():
            self.logger.warn("No recent files YAML found. Starting empty.")
            self.recent_files = []
            return

        try:
            with open(self.yaml_file_path, 'r') as file:
                yaml_content = yaml.safe_load(file)
                if yaml_content is None:
                    self.recent_files = []
                else:
                    self.recent_files = yaml_content.get('recent_files', [])
            self.logger.info(f"Loaded recent files: {self.recent_files}")

        except Exception as e:
            self.logger.error(f"Error loading recent files: {e}")
            self.recent_files = []

    def load_recent_file(self):
        """
        Loads the most recent file from the recent files list.
        """

        if self.recent_files:
            most_recent = self.recent_files[0]
            self.rasp.rsap_file_manager.load_from_JSON(most_recent)
            self.logger.info(f"Automatically loaded most recent file: {most_recent}")

    def load_recent_file_by_index(self, index: int):
        """
        Loads a recent file by its index in the recent files list.
        """
        if 0 <= index < len(self.recent_files):
            file_to_load = self.recent_files[index]
            self.rasp.rsap_file_manager.load_from_JSON(file_to_load)
            self.logger.info(f"Loaded recent file: {file_to_load}")
        else:
            self.logger.error(f"Invalid recent file index: {index}")
    # --------------------------
    # Save recent files to YAML
    # --------------------------
    def save_recent_files(self):
        """
        Saves the recent files list to YAML.
        """
        try:
            with open(self.yaml_file_path, 'w') as file:
                yaml.dump({'recent_files': self.recent_files}, file, default_flow_style=False)
            self.logger.info("Recent files saved to YAML.")
        except Exception as e:
            self.logger.error(f"Error saving recent files: {e}")

    # --------------------------
    # Add a file to recent files
    # --------------------------
    def add_recent_file(self, file_path: str):
        """
        Adds a file path to the recent files list.
        Moves it to the top if it already exists.
        """
        if file_path in self.recent_files:
            self.recent_files.remove(file_path)
        self.recent_files.insert(0, file_path)

        # Keep only MAX_RECENT_FILES
        self.recent_files = self.recent_files[:self.MAX_RECENT_FILES]

        # Save to YAML
        self.save_recent_files()

    def set_recent_file(self):
        """
        Saves the last opened file to the yaml file.
        """
        current_file = self.rasp.rsap_file_manager.get_action_sequence_file_path()
        self.add_recent_file(current_file)

    def get_recent_files(self)-> list[str]:
        """
        Returns the list of recent files.
        """
        return self.recent_files