from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_description = LaunchDescription()
    """Define"""
    fun5_bringup_pkg = 'fun5_bringup'

    contoller_node = Node(
            package=fun5_bringup_pkg,
            namespace='',
            executable='controller_node.py',
            name='controller_node'
        )
    
    launch_description.add_action(contoller_node)

    randomtarget_node = Node(
            package=fun5_bringup_pkg,
            namespace='',
            executable='randomtarget_node.py',
            name='randomtarget_node'
        )
    
    launch_description.add_action(randomtarget_node)
    
    return launch_description