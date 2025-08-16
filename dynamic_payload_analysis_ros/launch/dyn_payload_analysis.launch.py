from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    advanced_mode_arg = DeclareLaunchArgument(
        'advanced_mode',
        default_value='false',
        description='Enable advanced mode for enabling payload in every link of the selected kinematic tree'
    )
    
    resolution_ik_arg = DeclareLaunchArgument(
        'resolution_ik',
        default_value='0.20',
        description='Resolution for IK calculations'
    )
    
    workspace_range_arg = DeclareLaunchArgument(
        'workspace_range',
        default_value='2.0',
        description='Range of the workspace area for the IK solver'
    )

    # Define the node
    rviz_visualization_menu_node = Node(
        package='dynamic_payload_analysis_ros',
        executable='node_rviz_visualization_menu',
        name='node_rviz_visualization',
        output='screen',
        parameters=[{
            'advanced_mode': LaunchConfiguration('advanced_mode'),
            'resolution_ik': LaunchConfiguration('resolution_ik'),
            'workspace_range': LaunchConfiguration('workspace_range'),
        }],
    )

    return LaunchDescription([
        advanced_mode_arg,
        resolution_ik_arg,
        workspace_range_arg,
        rviz_visualization_menu_node,
    ])