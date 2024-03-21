from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


    gz_forwarder_1 = Node(
        package='uwb_locate',
        executable='gz_forwarder',
        name='gz_forwarder_1',
	    output='screen',
        parameters=[{'gz_odometry_topic': "/model/x500_1/odometry"}]
    )
    ld.add_action(gz_forwarder_1)

    gz_forwarder_2 = Node(
        package='uwb_locate',
        executable='gz_forwarder',
        name='gz_forwarder_2',
	    output='screen',
        parameters=[{'gz_odometry_topic': "/model/x500_2/odometry"}]
    )
    ld.add_action(gz_forwarder_2)

    gz_forwarder_3 = Node(
        package='uwb_locate',
        executable='gz_forwarder',
        name='gz_forwarder_3',
	    output='screen',
        parameters=[{'gz_odometry_topic': "/model/x500_3/odometry"}]
    )
    ld.add_action(gz_forwarder_3) 

    gz_forwarder_4 = Node(
        package='uwb_locate',
        executable='gz_forwarder',
        name='gz_forwarder_4',
	    output='screen',
        parameters=[{'gz_odometry_topic': "/model/x500_4/odometry"}]
    )
    ld.add_action(gz_forwarder_4)

    locate_1 = Node(
        package='uwb_locate',
        executable='locate.py',
        name='locate_1',
	    output='screen',
        parameters=[{'distance_matrix_topic_name': "/uwbData/px4_1",
                     'reference_odometry_topic_name_1': "/model/x500_1/odometry",
                     'reference_odometry_topic_name_2': "/model/x500_2/odometry",
                     'reference_odometry_topic_name_3': "/model/x500_3/odometry",
                     'reference_odometry_topic_name_4': "/model/x500_4/odometry",
                     }]
    )
    ld.add_action(locate_1)

    return ld
