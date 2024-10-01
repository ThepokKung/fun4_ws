from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('fun4_bringup')
    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)

    """Define"""
    fun4_bringup_pkg = 'fun4_bringup'

    kinematicsolve_node = Node(
            package=fun4_bringup_pkg,
            namespace='',
            executable='kinematicsolve_node.py',
            name='kinematicsolve_node'
        )
    
    launch_description.add_action(kinematicsolve_node)

    robotcontroller_node = Node(
            package=fun4_bringup_pkg,
            namespace='',
            executable='robotcontroller_node.py',
            name='robotcontroller_node'
        )
    launch_description.add_action(robotcontroller_node)
    
    return launch_description