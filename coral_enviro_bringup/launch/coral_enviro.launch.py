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

    enable_oled_display = LaunchConfiguration('enable_oled_display', default='true')
    enable_oled_display_arg = DeclareLaunchArgument(
        'enable_oled_display',
        default_value='true',
        description='Whether to enable the OLED display on the Enviro board.'
    )

    enviro_node = Node(
        package='coral_environmental_sensor_ros',
        executable='enviro_node',
        name=sensor_board_name,
        namespace=namespace,
        parameters=[{
            'enable_oled_display': enable_oled_display
        }]
    )

    ld.add_action(enviro_node)
    ld.add_action(namespace_arg)
    ld.add_action(sensor_board_name_arg)
    ld.add_action(enable_oled_display_arg)

    return ld