from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(
        package='uwb_locate',
        executable='uwb_location',
        name='uwb_location_1',
	    output='screen',
        parameters=[{'label_name': "px4_1"}]
    )
    ld.add_action(node1)


    return ld
