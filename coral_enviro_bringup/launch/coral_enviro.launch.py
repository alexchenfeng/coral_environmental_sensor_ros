from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    ld = LaunchDescription()

    enviro_node = Node(
        package='coral_environmental_sensor_ros',
        executable='enviro_node',
        name='enviro_node',
        namespace='coral_enviro',
    )

    ld.add_action(enviro_node)

    return ld