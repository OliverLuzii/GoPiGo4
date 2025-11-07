from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf = FileContent(PathJoinSubstitution([FindPackageShare("blacknav"), "urdf", "gopigo.urdf"]))

    return LaunchDescription([
        Node(
            package="blacknav",
            executable="motor_controller",
        ),
        Node(
            package="blacknav",
            executable="odom_pub",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"robot_description": urdf}],
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            parameters=[{
                ".camera.depth.image_rect_raw.compressedDepth.format": "rvl",
                "enable_color": True,
                "enable_depth": True,
                "enable_sync": True,
                "align_depth.enable": True,
                "rgb_camera.color_profile": "640x480x6",
                "depth_module.depth_profile": "640x480x6",
                "rgb_camera.exposure": 83,
                "rgb_camera.gain" : 96,
            }]
        )
    ])