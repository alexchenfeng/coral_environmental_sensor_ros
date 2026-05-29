from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()

    namespace = LaunchConfiguration('namespace', default='coral_enviro')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coral_enviro',
        description='The namespace to launch the nodes in.'
    )

    sensor_board_name = LaunchConfiguration('sensor_board_name', default='coral_enviro_board_1')
    sensor_board_name_arg = DeclareLaunchArgument(
        'sensor_board_name',
        default_value='coral_enviro_board_1',
        description='The name of the sensor board.'
    )   

    enviro_node = Node(
        package='coral_environmental_sensor_ros',
        executable='enviro_node',
        name=sensor_board_name,
        namespace=namespace,
    )

    ld.add_action(enviro_node)
    ld.add_action(namespace_arg)
    ld.add_action(sensor_board_name_arg)

    return ld