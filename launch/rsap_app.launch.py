import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments
    my_param = DeclareLaunchArgument('use_sim_time', default_value='False', description='An example parameter')

    # Define a ROS 2 node
    rsap_app_node = Node(
        package='ros_sequential_action_programmer',
        executable='ros_sequential_action_programmer',
        name='RSAP_App',
        #parameters=[{'param_name': LaunchConfiguration('my_param')}]
        emulate_tty=True
    )
    
    ld = LaunchDescription()
    ld.add_action(rsap_app_node)

    return ld