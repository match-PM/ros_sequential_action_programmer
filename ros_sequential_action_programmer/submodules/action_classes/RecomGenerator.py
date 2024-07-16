from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml
import os
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
# import packagenotfounderror
from ament_index_python.packages import PackageNotFoundError
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


try:
    from assembly_manager_interfaces.msg import ObjectScene, Object, RefFrame
    AM_INTERFACES_AVAILABLE = True
except ImportError:
    AM_INTERFACES_AVAILABLE = False
    ObjectScene = None

class RecomGenerator():
    def __init__(self,ros_node:Node) -> None:
        self.recommendations=[]
        
        self.ros_node = ros_node
        #self.tf_subscription = self.ros_node.create_subscription(TFMessage,
        #                                                         '/tf',
        #                                                         self.tf_callback,
        #                                                         10)
        
        #self.tf_subscription_2 = self.ros_node.create_subscription(TFMessage,
        #                                                '/tf_static',
        #                                                self.tf_static_callback,
        #                                                10)
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.ros_node, spin_thread=True)
        
        # subscribe to assembly manager
        if AM_INTERFACES_AVAILABLE:
            self.assembly_manager_subscription = self.ros_node.create_subscription(
                ObjectScene, "assembly_manager/scene", self.am_callback, 10, callback_group=self.callback_group
            )

        self.update_recommendations()
        #self.timer = self.ros_node.create_timer(1.0, self.on_timer)

    def on_timer(self):
        pass

    def am_callback(self, msg: ObjectScene):
        if not AM_INTERFACES_AVAILABLE:
            return

        try:
            class_string = 'objects_in_scene'
            class_string_frames = 'object_frames_in_scene'
            components_in_scene = []
            object_frames_in_scene = []
            for component in msg.objects_in_scene:
                components_in_scene.append(component.obj_name)
                for frame in component.ref_frames:
                    object_frames_in_scene.append(frame.frame_name)

            self.delete_from_recommendation(class_string_frames)
            self.delete_from_recommendation(class_string)
            self.append_to_recommendation({class_string: components_in_scene})
            self.append_to_recommendation({class_string_frames: object_frames_in_scene})
        except Exception as e:
            # Log or handle the exception if necessary
            pass

    def tf_callback(self, msg: TFMessage):
        frames_list = []
        for transform in msg.transforms:
            
            # Access transform information
            frames_list.append(transform.child_frame_id)
            # child_frame_id = transform.
            # header = transform.header
            # transform_data = transform.transform

            # # Accessing translation and rotation components
            # translation = transform_data.translation
            # rotation = transform_data.rotation
        self.append_to_recommendation({'TF':frames_list})

    def tf_static_callback(self, msg: TFMessage):
        frames_list = []
        print("Test")
        for transform in msg.transforms:
            
            # Access transform information
            frames_list.append(transform.child_frame_id)
            # child_frame_id = transform.
            # header = transform.header
            # transform_data = transform.transform

            # # Accessing translation and rotation components
            # translation = transform_data.translation
            # rotation = transform_data.rotation
        self.append_to_recommendation({'TF_static':frames_list})

    def vision_init(self):
        try:
            package_share_directory = get_package_share_directory('pm_vision_manager')
            path_config_path = package_share_directory + '/vision_assistant_path_config.yaml'
            with open(path_config_path, 'r') as file:
                FileData = yaml.safe_load(file)
                config = FileData["vision_assistant_path_config"]
                process_library_path=config["process_library_path"]
                camera_config_path=config["camera_config_path"]
                #self.vision_database_path=config["vision_database_path"]
                #self.vision_assistant_config=config["vision_assistant_config"]
            vision_processes = self.get_files_in_dir(directory=process_library_path,file_end='.json', exclude_str=['results'])
            vision_cameras = self.get_files_in_dir(directory=camera_config_path,file_end='.yaml', exclude_str=['vision_assistant'])
            self.append_to_recommendation({'Vision-Processes': vision_processes})
            self.append_to_recommendation({'Vision-Cameras': vision_cameras})
        except PackageNotFoundError as e:
            print("Opening package failed! Package 'pm_vision_manager' not found!")
    
    def update_frame_list(self):
        frames = []
        frame_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        # extract frames from dict
        if frame_dict == []:
            frames = []
        else:
            frames = list(frame_dict.keys())
        self.delete_from_recommendation('TF_frames')
        self.append_to_recommendation({'TF_frames':frames})

    def update_recommendations(self):
        self.update_frame_list()
        self.vision_init()

    def get_recommendations(self)->list:
        self.update_recommendations()
        return self.recommendations
    
    def delete_from_recommendation(self, key:str):
        for recom in self.recommendations:
            for recom_key in recom.keys():
                if recom_key == key:
                    self.recommendations.remove(recom)
                    return
        return
    
    def append_to_recommendation(self,input_recom_dict:dict):
        # for input_key, input_value in input_recom_dict.items():
        #     for recom in self.recommendations:
        #         for key, value in recom.items():
        #             if key == input_key:
        #                 value = input_value
        #                 return
        # self.recommendations.append(input_recom_dict)

        for recom in self.recommendations:
            for key, value in recom.items():
                if key in input_recom_dict:
                    recom[key] = input_recom_dict[key]
                    return
        self.recommendations.append(input_recom_dict)

    # @staticmethod
    # def get_files_in_dir(directory: str, file_end: str, exclude_str:list[str] = []):
    #     """
    #     This function returns a list of files in the directory with the given file_end.
    #     Parameters:
    #     param: directory: str
    #     file_end: str
    #     exclude_str: list[str]
    #     """
    #     files = []
    #     for foldername, subfolders, filenames in os.walk(directory):
    #         for filename in filenames:
    #             if filename.endswith(file_end):
    #                 valid_file = True
    #                 for string in exclude_str:
    #                     if string in filename:
    #                         valid_file = False
    #                 if valid_file:    
    #                     files.append(os.path.join(foldername, filename).replace(directory, ''))
    #     return files

    @staticmethod
    def get_files_in_dir(directory: str, file_end: str, exclude_str: list[str] = []):
        """
        This function returns a list of files in the directory with the given file_end.
        Parameters:
        param: directory: str
        file_end: str
        exclude_str: list[str]
        """
        files = []
        for foldername, subfolders, filenames in os.walk(directory):
            for filename in filenames:
                if filename.endswith(file_end):
                    valid_file = True
                    for string in exclude_str:
                        if string in filename:
                            valid_file = False
                            break  # No need to continue checking once a match is found
                    if valid_file:    
                        files.append(os.path.relpath(os.path.join(foldername, filename), directory))
        return files

if __name__ == "__main__":
    recommendations = RecomGenerator()
    print(f"Recommendations: {recommendations.get_recommendations()}")