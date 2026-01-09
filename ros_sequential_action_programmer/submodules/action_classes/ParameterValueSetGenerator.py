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
from ros_sequential_action_programmer.submodules.action_classes.compatibility_mapping import COMPATIBLE_TYPES

try:
    from assembly_manager_interfaces.msg import ObjectScene, Object, RefFrame
    AM_INTERFACES_AVAILABLE = True
except ImportError:
    AM_INTERFACES_AVAILABLE = False
    ObjectScene = None
    
    
class ValuesSet():
    def __init__(self,value_set_type:str, set_name:str) -> None:
        self.value_set_type = value_set_type
        self._values_list:list[any] = []
        self.value_set_name = set_name
        
    def set_value_list(self, new_list:list[any]):
        self._values_list = new_list
    
    def get_values_list(self)->list[any]:
        return self._values_list

class ValueSets():
    def __init__(self) -> None:
        self._value_sets:list[ValuesSet] = []

    # def get_all_value_set_names(self,type:str = None)->list[str]:
    #     names_list = []
    #     for sets in self._value_sets:
    #         _name = sets.value_set_name
    #         _type = sets.value_set_type
    #         if type is None:
    #             names_list.append(_name)
    #         elif type == _type:
    #             names_list.append(_name)
                
    #     return names_list
    
    def append_set(self, v_set:ValuesSet):
        # check value set name does not exist
        for existing_set in self._value_sets:
            if existing_set.value_set_name == v_set.value_set_name:
                raise ValueError(f"Set with name '{v_set.value_set_name}' already exists!")
            
        if isinstance(v_set, ValuesSet):
            self._value_sets.append(v_set)
        else:
            raise ValueError("Is not a set!")
    
    # def get_value_sets_for_type(self,set_type:str):
    #     new_set = ValueSets()
        
    #     for _set in self._value_sets:
    #         if _set.value_set_type == set_type:
    #             new_set.append_set(_set)
        
    #     return new_set
    
    def get_set_for_set_name(self, name:str):
        for _set in self._value_sets:
            if _set.value_set_name == name:
                return _set
        raise ValueError(f"Set with name '{name}' not found!")
    

    def get_all_value_set_names(self, type: str = None) -> list[str]:
        names_list = []
        for sets in self._value_sets:
            _name = sets.value_set_name
            _type = sets.value_set_type
            if type is None:
                names_list.append(_name)
            else:
                # include sets that are compatible with the requested type
                compatible_types = COMPATIBLE_TYPES.get(type, [type])
                if _type in compatible_types:
                    names_list.append(_name)
        return names_list

    def get_value_sets_for_type(self, set_type: str):
        new_set = ValueSets()
        compatible_types = COMPATIBLE_TYPES.get(set_type, [set_type])
        
        for _set in self._value_sets:
            if _set.value_set_type in compatible_types:
                new_set.append_set(_set)
        return new_set
class ParameterValueSetGenerator():
    def __init__(self,ros_node:Node) -> None:
        self.value_sets = ValueSets()
        
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
            self.as_object_in_scene_set = ValuesSet(value_set_type='str',
                                                    set_name='components_in_scene')
            
            self.as_frames_in_scene_set = ValuesSet(value_set_type='str',
                                                    set_name='frames_in_scene')
            
            self.as_instructions_in_scene_set = ValuesSet(value_set_type='str',
                                                        set_name='instructions_in_scene')
            
            self.value_sets.append_set(self.as_object_in_scene_set)
            self.value_sets.append_set(self.as_frames_in_scene_set)
            self.value_sets.append_set(self.as_instructions_in_scene_set)
            
            self.assembly_manager_subscription = self.ros_node.create_subscription(
                ObjectScene, "assembly_manager/scene", self.am_callback, 10, callback_group=self.callback_group)

        self.tf_frames_set = ValuesSet(value_set_type='str',
                                        set_name='tf_frames')
        
        self.vision_cameras_set = ValuesSet(value_set_type='str',
                                set_name='vision_cameras')
        
        self.vision_processes_set = ValuesSet(value_set_type='str',
                        set_name='vision_processes')
        
        self.value_sets.append_set(self.tf_frames_set)
        self.value_sets.append_set(self.vision_cameras_set)
        self.value_sets.append_set(self.vision_processes_set)
        
        self._apply_test_sets()
        
        self.update()
        #self.timer = self.ros_node.create_timer(1.0, self.on_timer)

    def on_timer(self):
        pass

    def _apply_test_sets(self):
        test_set_1 = ValuesSet(value_set_type='string',
                        set_name='test_set_1')
        
        test_set_1.set_value_list(["1","2","3"])
        
        self.value_sets.append_set(test_set_1)
        
        test_set_2 = ValuesSet(value_set_type='string',
                        set_name='test_set_2')
        
        test_set_2.set_value_list(["5","6","7"])
        
        self.value_sets.append_set(test_set_2)

        test_set_3 = ValuesSet(value_set_type='uint32',
                        set_name='test_set_3')
        
        test_set_3.set_value_list([5, 8, 9])
        
        self.value_sets.append_set(test_set_3)

        test_set_4 = ValuesSet(value_set_type='double',
                        set_name='test_set_4')
        
        test_set_4.set_value_list([5, 8, 9])
        
        self.value_sets.append_set(test_set_4)


    def am_callback(self, msg: ObjectScene):
        if not AM_INTERFACES_AVAILABLE:
            return

        try:
            components_in_scene = []
            object_frames_in_scene = []
            for component in msg.objects_in_scene:
                components_in_scene.append(component.obj_name)
                for frame in component.ref_frames:
                    object_frames_in_scene.append(frame.frame_name)

            self.as_object_in_scene_set.set_value_list(components_in_scene)
            self.as_frames_in_scene_set.set_value_list(object_frames_in_scene)
            
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
        

    def tf_static_callback(self, msg: TFMessage):
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
            
            self.vision_cameras_set.set_value_list(vision_cameras)
            self.vision_processes_set.set_value_list(vision_processes)
        
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
            
        self.tf_frames_set.set_value_list(frames)

    def update(self):
        self.update_frame_list()
        self.vision_init()

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
