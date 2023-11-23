from setuptools import find_packages, setup

package_name = 'ros_sequential_action_programmer'
submodules = 'ros_sequential_action_programmer/submodules'
submodules_action_classes = 'ros_sequential_action_programmer/submodules/action_classes'
submodules_app = 'ros_sequential_action_programmer/submodules/RsapApp_submodules'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    #packages=find_packages(package_name,submodules),   
    packages=(package_name,submodules, submodules_action_classes, submodules_app),   
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/blacklist.yaml']),
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
