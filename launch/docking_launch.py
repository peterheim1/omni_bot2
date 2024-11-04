import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the apriltag_ros config file
    apriltag_config = os.path.join(
        get_package_share_directory('omni_bot2'), 'config', 'tags_36h11.yaml'
    )
    docking_config = os.path.join(
        get_package_share_directory('omni_bot2'), 'config', 'docking.yaml')
    
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[docking_config],
    )

    docking_pose_publisher = Node(
        package='omni_bot2',
        executable='dock_pose_pub.py',
        name='dock_pose_pub',
        output='screen',
        
    )

    docking_controller = Node(
        package='omni_bot2',
        executable='docking_controller.py',
        name='docking_controller',
        output='screen',
        
    )
    
    

    # Define the image rectification node
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify',
        namespace='',
        remappings=[('image', '/rear_camera/color/image_raw'),
                    ('camera_info', '/rear_camera/color/camera_info')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Define the AprilTag node for tag detection
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag',
        namespace='apriltag',
        remappings=[
            ('/apriltag/image_rect', '/rear_camera/color/image_raw'),
            ('/apriltag/camera_info', '/rear_camera/color/camera_info')
        ],
        parameters=[apriltag_config],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Create the container that will host the composable nodes (Rectify and AprilTag)
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node
        ],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    # Return the LaunchDescription with the RealSense node and the container
    return LaunchDescription([
        docking_server,
        docking_pose_publisher,
        docking_controller,
        container,
        lifecycle_manager

    ])
