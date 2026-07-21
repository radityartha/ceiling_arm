"""Bring up 2 physical RealSense D455 cameras on the `rgbd` / `rgbd2` contract
that the perception pipeline (seg_router, object_localizer, depth_cloud,
seg_cloud, ...) expects:

    /<ns>/rgb            sensor_msgs/Image     (rgb8)
    /<ns>/depth          sensor_msgs/Image     (32FC1, meters)
    /<ns>/camera_info    sensor_msgs/CameraInfo

...plus a static TF `world -> <ns>_camera_optical` per camera (the pipeline
looks up exactly that frame name, see object_localizer/depth_cloud
`optical_frame_suffix`). The realsense2_camera driver publishes depth as
16UC1 millimeters aligned to color; `depth_image_proc`'s `convert_metric_node`
converts that to the 32FC1-meters contract this repo's nodes decode directly
(see depth_cloud.py `_decode`). Color image_raw and camera_info are relayed
1:1 onto the neutral topic names via `topic_tools relay` (cheap, no recompute).

Camera identity is pinned by USB serial number (stable across replug/USB
port), not port order.

    ros2 launch reachability_gng realsense_dual.launch.py
    ros2 launch reachability_gng realsense_dual.launch.py \\
        serial1:=234222303079 serial2:=241122302297

** world->camera TF is a PLACEHOLDER (identity) until calibrated. ** Override
per camera with tf1_x/y/z/roll/pitch/yaw (and tf2_*), in meters/radians, using
the ROS optical-frame convention (x right, y down, z forward out of the lens).
Until it's calibrated, depth_cloud/object_localizer will place points as if
the camera sat at the world origin looking down +Z -- fine for smoke-testing
YOLOE detection on /rgbd/seg/debug_image, NOT valid for reachability/grasping.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _camera_nodes(ns, serial, color_profile, depth_profile):
    """RealSense driver + mm->m depth convert + rgb/camera_info relay for one camera."""
    driver = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=ns,
        namespace=ns,
        parameters=[{
            'serial_no': ParameterValue(serial, value_type=str),
            'camera_name': ns,
            'camera_namespace': ns,
            'align_depth.enable': True,
            'pointcloud.enable': False,
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'rgb_camera.color_profile': color_profile,
            'depth_module.depth_profile': depth_profile,
        }],
        output='screen',
    )
    convert_metric = Node(
        package='depth_image_proc',
        executable='convert_metric_node',
        name=f'{ns}_convert_metric',
        remappings=[
            ('image_raw', f'/{ns}/{ns}/aligned_depth_to_color/image_raw'),
            ('image', f'/{ns}/depth'),
        ],
    )
    relay_rgb = Node(
        package='topic_tools', executable='relay', name=f'{ns}_relay_rgb',
        arguments=[f'/{ns}/{ns}/color/image_raw', f'/{ns}/rgb'],
    )
    relay_info = Node(
        package='topic_tools', executable='relay', name=f'{ns}_relay_info',
        arguments=[f'/{ns}/{ns}/color/camera_info', f'/{ns}/camera_info'],
    )
    return [driver, convert_metric, relay_rgb, relay_info]


def _static_tf(ns, x, y, z, roll, pitch, yaw):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{ns}_world_tf',
        arguments=['--x', x, '--y', y, '--z', z,
                   '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                   '--frame-id', 'world', '--child-frame-id', f'{ns}_camera_optical'],
    )


def generate_launch_description():
    serial1 = LaunchConfiguration('serial1')
    serial2 = LaunchConfiguration('serial2')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    return LaunchDescription([
        DeclareLaunchArgument('serial1', default_value='234222303079'),
        DeclareLaunchArgument('serial2', default_value='241122302297'),
        DeclareLaunchArgument('color_profile', default_value='1280x720x30'),
        DeclareLaunchArgument('depth_profile', default_value='848x480x30'),
        # PLACEHOLDER world->camera extrinsics -- calibrate then override, see
        # module docstring above.
        DeclareLaunchArgument('tf1_x', default_value='0.0'),
        DeclareLaunchArgument('tf1_y', default_value='0.0'),
        DeclareLaunchArgument('tf1_z', default_value='2.0'),
        DeclareLaunchArgument('tf1_roll', default_value='-1.5708'),
        DeclareLaunchArgument('tf1_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf1_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('tf2_x', default_value='0.0'),
        DeclareLaunchArgument('tf2_y', default_value='0.0'),
        DeclareLaunchArgument('tf2_z', default_value='2.0'),
        DeclareLaunchArgument('tf2_roll', default_value='-1.5708'),
        DeclareLaunchArgument('tf2_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf2_yaw', default_value='-1.5708'),

        *_camera_nodes('rgbd', serial1, color_profile, depth_profile),
        *_camera_nodes('rgbd2', serial2, color_profile, depth_profile),
        _static_tf('rgbd', LaunchConfiguration('tf1_x'), LaunchConfiguration('tf1_y'),
                  LaunchConfiguration('tf1_z'), LaunchConfiguration('tf1_roll'),
                  LaunchConfiguration('tf1_pitch'), LaunchConfiguration('tf1_yaw')),
        _static_tf('rgbd2', LaunchConfiguration('tf2_x'), LaunchConfiguration('tf2_y'),
                  LaunchConfiguration('tf2_z'), LaunchConfiguration('tf2_roll'),
                  LaunchConfiguration('tf2_pitch'), LaunchConfiguration('tf2_yaw')),
    ])
