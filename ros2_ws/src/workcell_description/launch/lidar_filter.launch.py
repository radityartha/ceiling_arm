from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Define the CropBox filter node
    crop_box_node = ComposableNode(
        package="pcl_ros",
        plugin="pcl_ros::CropBox",  # The class name in the library
        name="filter_crop_box_node",
        parameters=[
            # Frame IDs
            {"input_frame": "world"},
            {"output_frame": "world"},
            # The Box Dimensions (Meters relative to World)
            # Crop area: Keep points INSIDE this box
            {"min_x": -2.0},
            {"max_x": 2.0},
            {"min_y": -2.0},
            {"max_y": 2.0},
            {"min_z": 0.1},
            {"max_z": 1.8},  # Cut floor (<0.1) and ceiling (>1.8)
            {"negative": False},  # False = Keep inside, True = Remove inside
        ],
        remappings=[("input", "/livox/lidar"), ("output", "/livox/lidar_filtered")],
    )

    # Create a Container to run the node
    container = ComposableNodeContainer(
        name="filter_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[crop_box_node],
        output="screen",
    )

    return LaunchDescription([container])
