import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    visual_servoing_share = get_package_share_directory("visual_servoing")
    yaml_dir = os.path.join(visual_servoing_share, "yaml")

    handeye_yaml_path = os.path.join(yaml_dir, "handeye_tf.yaml")
    #saved_goals_yaml_path = os.path.join(yaml_dir, "saved_tag_goals.yaml")
    saved_goals_yaml_path = "/home/woubraim/extender_ws/src/visual_servoing/yaml/saved_tag_goals.yaml"
    handeye_tf_publisher = Node(
        package="visual_servoing",
        executable="handeye_tf_publisher",
        name="handeye_tf_publisher",
        output="screen",
        parameters=[
            {
                "yaml_path": handeye_yaml_path,
                "ee_frame": "camera_explorer",
                "camera_frame": "oak_parent_frame",
            }
        ],
    )

    visual_servoing_node = Node(
        package="visual_servoing",
        executable="visual_servoing_node",
        name="visual_servoing_node",
        output="screen",
        parameters=[
            {
                "camera_type": "oak",
                "target_frame": "base_link",
                "tag_size": 0.024,
                "fullscreen_display": True,
            }
        ],
    )

    save_goal_manager = Node(
        package="visual_servoing",
        executable="save_goal_manager",
        name="save_goal_manager",
        output="screen",
        parameters=[
            {
                "detected_goals_topic": "/visual_servoing/detected_goals",
                "save_service_name": "/visual_servoing/save_current_tag_goal",
                "base_frame": "base_link",
                "ee_frame": "ft_frame",
                "yaml_path": saved_goals_yaml_path,
            }
        ],
    )

    shared_control_visualization_node = Node(
        package="visual_servoing",
        executable="shared_control_visualization_node",
        name="shared_control_visualization_node",
        output="screen",
        parameters=[
            {
                "goal_sphere_diameter": 0.10,
                "goal_label_height": 0.08,
                "marker_lifetime_sec": 0.2,
            }
        ],
    )

    return LaunchDescription([
        handeye_tf_publisher,
        visual_servoing_node,
        save_goal_manager,
        shared_control_visualization_node,
    ])
