from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # ============================================================================
    # Declare launch arguments
    # ============================================================================
    server_address_arg = DeclareLaunchArgument(
        'server_address',
        default_value='192.168.1.113',
        description='OptiTrack/Motive server IP address'
    )
    local_address_arg = DeclareLaunchArgument(
        'local_address',
        default_value='192.168.1.200',
        description='Local machine IP address'
    )
    connection_type_arg = DeclareLaunchArgument(
        'connection_type',
        default_value='Unicast',
        description='Connection type: Unicast or Multicast'
    )
    multicast_address_arg = DeclareLaunchArgument(
        'multicast_address',
        default_value='239.255.42.99',
        description='Multicast group address (only used in Multicast mode)'
    )
    server_command_port_arg = DeclareLaunchArgument(
        'server_command_port',
        default_value='1510',
        description='NatNet command port'
    )
    server_data_port_arg = DeclareLaunchArgument(
        'server_data_port',
        default_value='1511',
        description='NatNet data port'
    )
    topic_namespace_arg = DeclareLaunchArgument(
        'topic_namespace',
        default_value='optitrack',
        description='Topic namespace for OptiTrack messages'
    )
    rigid_body_name_arg = DeclareLaunchArgument(
        'rigid_body_name',
        default_value='drone',
        description='Name of the rigid body to relay to PX4'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='World frame for tf2 transformations'
    )
    optitrack_frame_arg = DeclareLaunchArgument(
        'optitrack_frame',
        default_value='optitrack',
        description='OptiTrack frame for tf2 transformations'
    )
    map_xyz_arg = DeclareLaunchArgument(
        'map_xyz',
        default_value='[0.0, 0.0, 0.0]',
        description='XYZ translation for coordinate frame mapping'
    )
    map_rpy_arg = DeclareLaunchArgument(
        'map_rpy',
        default_value='[0.0, 0.0, 0.0]',
        description='RPY rotation for coordinate frame mapping'
    )
    map_rpy_in_degrees_arg = DeclareLaunchArgument(
        'map_rpy_in_degrees',
        default_value='false',
        description='Whether RPY values are in degrees (true) or radians (false)'
    )

    # Add all launch arguments
    ld.add_action(server_address_arg)
    ld.add_action(local_address_arg)
    ld.add_action(connection_type_arg)
    ld.add_action(multicast_address_arg)
    ld.add_action(server_command_port_arg)
    ld.add_action(server_data_port_arg)
    ld.add_action(topic_namespace_arg)
    ld.add_action(rigid_body_name_arg)
    ld.add_action(world_frame_arg)
    ld.add_action(optitrack_frame_arg)
    ld.add_action(map_xyz_arg)
    ld.add_action(map_rpy_arg)
    ld.add_action(map_rpy_in_degrees_arg)

    # ============================================================================
    # Get launch configuration values
    # ============================================================================
    server_address = LaunchConfiguration('server_address')
    local_address = LaunchConfiguration('local_address')
    connection_type = LaunchConfiguration('connection_type')
    multicast_address = LaunchConfiguration('multicast_address')
    server_command_port = LaunchConfiguration('server_command_port')
    server_data_port = LaunchConfiguration('server_data_port')
    topic_namespace = LaunchConfiguration('topic_namespace')
    rigid_body_name = LaunchConfiguration('rigid_body_name')
    world_frame = LaunchConfiguration('world_frame')
    optitrack_frame = LaunchConfiguration('optitrack_frame')
    map_xyz = LaunchConfiguration('map_xyz')
    map_rpy = LaunchConfiguration('map_rpy')
    map_rpy_in_degrees = LaunchConfiguration('map_rpy_in_degrees')

    # ============================================================================
    # Define nodes
    # ============================================================================

    # Node 1: OptiTrack client
    optitrack_client = Node(
        package='optitrack4px4',
        executable='optitrack_client',
        output='screen',
        parameters=[{
            'connection_type': connection_type,
            'server_address': server_address,
            'local_address': local_address,
            'multicast_address': multicast_address,
            'server_command_port': server_command_port,
            'server_data_port': server_data_port,
            'namespace': topic_namespace,
            'world_frame': world_frame,
            'optitrack_frame': optitrack_frame,
            'map_xyz': map_xyz,
            'map_rpy': map_rpy,
            'map_rpy_in_degrees': map_rpy_in_degrees
        }]
    )

    # Node 2: Visual odometry relay (from mocap_px4_relays)
    # Remap the relay's hardcoded /vicon/drone/drone subscription to
    # the configurable optitrack topic: /{namespace}/{rb_name}/{rb_name}
    visual_odometry_relay = Node(
        package='mocap_px4_relays',
        executable='visual_odometry_relay',
        output='screen',
        remappings=[
            ('/vicon/drone/drone',
             [topic_namespace, '/', rigid_body_name, '/', rigid_body_name])
        ]
    )

    # Node 3: Full state relay (from mocap_px4_relays)
    full_state_relay = Node(
        package='mocap_px4_relays',
        executable='full_state_relay',
        output='screen',
    )

    # Add nodes
    ld.add_action(optitrack_client)
    ld.add_action(visual_odometry_relay)
    ld.add_action(full_state_relay)

    return ld
