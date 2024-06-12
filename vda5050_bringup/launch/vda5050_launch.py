from launch import LaunchDescription
from launch_ros.actions import Node


# TODO: Include launch files in vda5050_connector/launch instead
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

    luka_adapter = Node(
        package="luka_adapter",
        executable="luka_adapter",
        name="luka_adapter"
    )

    return LaunchDescription(
        [
            luka_adapter,
            mqtt_bridge,
            vda5050_controller,
        ]
    )
