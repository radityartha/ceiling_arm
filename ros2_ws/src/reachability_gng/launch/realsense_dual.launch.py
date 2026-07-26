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
port), not port order. `rgbd` (serial1) is the gantry_1-side (+Y) camera,
`rgbd2` (serial2) the gantry_2-side (-Y) one -- matches the Isaac twin's
convention (ros2_bridge_gui.py CAMERAS list), keep them in sync.

    ros2 launch reachability_gng realsense_dual.launch.py
    ros2 launch reachability_gng realsense_dual.launch.py \\
        serial1:=241122302297 serial2:=234222303079
    ros2 launch reachability_gng realsense_dual.launch.py enable1:=false  # cam1 cable bad

** enable1/enable2 (default true): fully skip a camera. ** A camera node that
never finds its serial retries enumeration forever, which contends for the
USB bus and can knock the OTHER camera offline ("Device or resource busy").
If one camera's cable/hardware is known bad, disable it here rather than
leaving it running -- otherwise the working camera degrades too.

** world->camera TF defaults come from calibrate_extrinsics.py (2026-07-25
first pass, see that script + charuco_common.py / generate_charuco_board.py
in this package). ** Override per camera with tf1_x/y/z/roll/pitch/yaw (and
tf2_*), in meters/radians, using the ROS optical-frame convention (x right,
y down, z forward out of the lens), if you re-run the calibration. rgbd2
(tf2)'s fit was marginal (~2.2px reprojection RMS, board was small/blurry in
that camera's view) -- good enough for a first pass, re-calibrate with the
board bigger/closer in rgbd2's frame for reachability/grasping precision.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
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
        # serial1/2 -> ns rgbd/rgbd2, matching the Isaac twin's layout
        # (ros2_bridge_gui.py CAMERAS) so sim and real agree. The two sit at
        # DIAGONALLY OPPOSITE corners of the work area (rail spans x: 0..2.0,
        # so the gantry START is x=0):
        #   rgbd  (serial1) = 241122302297 -> (+X, +Y), past the rail end,
        #                     gantry_1 side: ACROSS from the gantry start.
        #   rgbd2 (serial2) = 234222303079 -> (-X, -Y), before the rail start,
        #                     gantry_2 side: NEAR the gantry start.
        # Established 2026-07-26 by solving each camera's pose RELATIVE TO THE
        # BOARD (PnP only, independent of the broken world extrinsics) once
        # both cameras were finally on USB3 at 1280x720: 241122302297 came out
        # at board-frame Y=+0.69 (RMS 0.31px) = the +Y side, so it is `rgbd`.
        # The earlier opposite pairing was solved when one camera was stuck at
        # 640x480 and its fit was junk (RMS 2.2px) -- do not trust that run.
        # Verify by eye after any change (rqt_image_view on /rgbd/rgb and
        # /rgbd2/rgb) rather than trusting this comment -- and if the LAYOUT
        # looks wrong in RViz while the raw images look right, suspect stale
        # static_transform_publisher processes first (see below), not this.
        # INTRINSICS never need swapping: they ride with each device via
        # camera_info.
        DeclareLaunchArgument('serial1', default_value='241122302297'),
        DeclareLaunchArgument('serial2', default_value='234222303079'),
        # A camera node that never finds its serial keeps retrying enumeration
        # forever, which contends for the USB bus and can knock the OTHER
        # camera offline ("Device or resource busy"). Set enable1/2:=false to
        # fully skip a camera whose cable/hardware is known bad.
        DeclareLaunchArgument('enable1', default_value='true'),
        DeclareLaunchArgument('enable2', default_value='true'),
        DeclareLaunchArgument('color_profile', default_value='1280x720x30'),
        DeclareLaunchArgument('depth_profile', default_value='848x480x30'),
        # world->camera extrinsics, from calibrate_extrinsics.py (2026-07-25
        # first pass, ChArUco board pinned via t1_a1_tool_frame + tape-measure
        # deltas -- see memory/reachability_gng notes).
        #
        # ** THESE ARE KNOWN-INACCURATE, RE-CALIBRATION PENDING. ** A live
        # cross-camera overlap check (deproject both depth images to `world`,
        # symmetric nearest-neighbour on the shared floor/wall surfaces)
        # scored ~36 cm median disagreement -- the solve itself is off, so
        # do not trust these for reachability/grasping. Re-run
        # calibrate_extrinsics.py with the board large and sharp in BOTH
        # views (the 28 mm board is only ~18 px/square at the ~2 m ceiling
        # distance, too small for ArUco marker decoding -- print a bigger
        # one, constants live in charuco_common.py).
        #
        # Which solve belongs to which physical camera is NOT reliably known:
        # the original run happened while the serial->ns mapping was itself
        # wrong, so the two results cannot be attributed with confidence.
        # They are ordered here so each camera's frame lands on the side it
        # actually occupies -- rgbd (+Y, beside t1_base_link) and rgbd2 (-Y,
        # beside t2_base_link) -- confirmed both in RViz against the gantry
        # base frames and by the board-relative PnP (241122302297 solved to
        # board-frame Y=+0.69). Treat the numbers themselves as placeholders.
        DeclareLaunchArgument('tf1_x', default_value='0.9339'),
        DeclareLaunchArgument('tf1_y', default_value='1.9872'),
        DeclareLaunchArgument('tf1_z', default_value='2.2075'),
        DeclareLaunchArgument('tf1_roll', default_value='-2.4072'),
        DeclareLaunchArgument('tf1_pitch', default_value='0.3226'),
        DeclareLaunchArgument('tf1_yaw', default_value='2.4601'),
        DeclareLaunchArgument('tf2_x', default_value='0.0855'),
        DeclareLaunchArgument('tf2_y', default_value='-0.6396'),
        DeclareLaunchArgument('tf2_z', default_value='2.0485'),
        DeclareLaunchArgument('tf2_roll', default_value='-2.3216'),
        DeclareLaunchArgument('tf2_pitch', default_value='-0.1442'),
        DeclareLaunchArgument('tf2_yaw', default_value='-0.9436'),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable1')),
            actions=[
                *_camera_nodes('rgbd', serial1, color_profile, depth_profile),
                _static_tf('rgbd', LaunchConfiguration('tf1_x'), LaunchConfiguration('tf1_y'),
                          LaunchConfiguration('tf1_z'), LaunchConfiguration('tf1_roll'),
                          LaunchConfiguration('tf1_pitch'), LaunchConfiguration('tf1_yaw')),
            ]),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable2')),
            actions=[
                *_camera_nodes('rgbd2', serial2, color_profile, depth_profile),
                _static_tf('rgbd2', LaunchConfiguration('tf2_x'), LaunchConfiguration('tf2_y'),
                          LaunchConfiguration('tf2_z'), LaunchConfiguration('tf2_roll'),
                          LaunchConfiguration('tf2_pitch'), LaunchConfiguration('tf2_yaw')),
            ]),
    ])
