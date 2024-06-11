from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mqtt_bridge = Node(
        package="vda5050_connector",
        executable="mqtt_bridge.py",
        name="mqtt_bridge"
    )

    vda5050_controller = Node(
        package="vda5050_connector",
        executable="vda5050_controller.py",
        name="vda5050_controller"
    )

    return LaunchDescription(
        [
            mqtt_bridge,
            # vda5050_controller,
        ]
    )
