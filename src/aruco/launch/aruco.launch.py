# ROS 2 差异：本文件由 ROS 1 XML launch（launch/aruco.launch）转换而来。
# 节点名、可执行文件名、参数名与取值，以及节点对外的话题/消息类型均与 ROS 1 保持一致。
# ROS 1 的 <param name="x" value="y"/> 写在 <node> 内属于节点私有参数（/aruco/x），
# ROS 2 中节点参数本身就是节点私有的，因此直接放进 parameters 字典中。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco',
            executable='aruco',
            name='aruco',
            output='screen',
            parameters=[{
                # Detector
                'debug': False,
                'cosine_limit': 0.7,
                'theshold_block_size_min': 3,
                'theshold_block_size_max': 21,
                'max_error_quad': 0.035,
                'min_area': 100,

                # Markers SIZE_CM POS_XYZ ROT_XYZ
                'marker321': '0.2_0_0_0_0_0_0',
                'marker123': '0.2_0_0_0_0_0_0',

                # Camera
                'topic_camera': '/camera/rgb',
                'topic_camera_info': '/camera/info',
            }],
        ),
    ])
