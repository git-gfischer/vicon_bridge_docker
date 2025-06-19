from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Declare arguments
    vicon_ip_arg = DeclareLaunchArgument(
        'vicon_ip',
        default_value=TextSubstitution(text='192.168.1.115'),
        description='IP address of the Vicon computer'
    )

    port_arg = DeclareLaunchArgument(
        'port_number',
        default_value=TextSubstitution(text='801'),
        description='Port number for the Vicon connection'
    )
    host_name = f"{vicon_ip_arg}:{port_arg}"

    return LaunchDescription([
        Node(
            package='vicon_bridge',
            executable='vicon_bridge',
            name='vicon_bridge',
            parameters = [
                {"host_name": host_name},
                {"stream_mode": "ServerPush"}, # or "ClientPull"
                {"update_rate_hz": 250.0},
                {"publish_specific_segment": False},
                {"world_frame_id": "world"},
                {"tf_namespace": "vicon"}
            ]
        ),
    ])