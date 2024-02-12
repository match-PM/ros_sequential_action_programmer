from setuptools import find_packages, setup

package_name = 'ros_sequential_action_programmer'
submodules = 'ros_sequential_action_programmer/submodules'
submodules_action_classes = 'ros_sequential_action_programmer/submodules/action_classes'
submodules_obj_dict_classes = 'ros_sequential_action_programmer/submodules/obj_dict_modules'
submodules_app = 'ros_sequential_action_programmer/submodules/RsapApp_submodules'
pm_robot_modules = 'ros_sequential_action_programmer/submodules/pm_robot_modules'
co_pilot = 'ros_sequential_action_programmer/submodules/co_pilot'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    #packages=find_packages(package_name,submodules),   
    packages=(package_name,submodules, submodules_action_classes, submodules_app, submodules_obj_dict_classes, pm_robot_modules, co_pilot),   
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/blacklist.yaml']),
        ('share/' + package_name, ['config/whitelist.yaml']),
        ('share/' + package_name, ['config/OpenAI_config.yaml']),
        ('share/' + package_name, ['config/path_definitions.yaml']),
        ('share/' + package_name + '/launch', ['launch/rsap_app.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niklas Terei',
    maintainer_email='terei@match.uni-hannover.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_sequential_action_programmer = ros_sequential_action_programmer.ros_sequential_action_programmer:main'
        ],
    },
)
