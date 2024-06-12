import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction
from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import AGVPosition as VDAAGVPosition
from vda5050_msgs.msg import Velocity as VDAVelocity
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import CurrentAction as VDACurrentAction


class LukaAdapter(Node):
    def __init__(self) -> None:
        super().__init__('luka_adapter')

        self.get_logger().info(f"LUKA adapter starting")

        self._manufacturer_name = "robots"
        self._robot_name = "robot_1"

        base_interface_name = (
            f"uagv/{self._manufacturer_name}/{self._robot_name}/"
        )

        self.get_adapter_state_srv = self.create_service(
            srv_type=GetState,
            srv_name=base_interface_name + "adapter/get_state",
            callback=self.get_state_callback,
        )

        self.supported_actions_srv = self.create_service(
            SupportedActions,
            base_interface_name + "adapter/supported_actions",
            lambda _: self.get_logger().info("Supported actions request"),
        )

        nav_to_node_action_client_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav_to_node_ac_srv = ActionServer(
            node=self,
            action_type=NavigateToNode,
            action_name=base_interface_name + "adapter/nav_to_node",
            execute_callback=self.navigate_to_node_callback,
            callback_group=nav_to_node_action_client_cb_group,
        )

        self._process_vda_action_ac_srv = ActionServer(
            node=self,
            action_type=ProcessVDAAction,
            action_name=base_interface_name + "adapter/vda_action",
            execute_callback=self.process_vda_action_callback,
        )

        self.get_logger().info(f"LUKA adapter started")

    def on_shutdown(self):
        self.get_logger().info(f"LUKA adapter shutting down")

    def get_state_callback(self, request, response):
        self.get_logger().debug(f"Getting state: {request}")
        # order_state = VDAOrderState()
        # order_state.agv_position = self._agv_position
        # order_state.velocity = self._velocity
        # response.state = order_state
        return response

    def navigate_to_node_callback(self, goal_handle):
        node = goal_handle.request.node

        self.get_logger().info(
            f"Navigating to node '{goal_handle.request.node}', edge '{goal_handle.request.edge}'"
        )

        goal_handle.succeed()

        result = NavigateToNode.Result()
        return result

    def process_vda_action_callback(self, goal_handle):
        action = goal_handle.request.action
        result = ProcessVDAAction.Result()

        self.get_logger().info(f"Requested action: {action}")

        action_parameters = {}
        for action_parameter in action.action_parameters:
            action_parameters[action_parameter.key] = action_parameter.value

        result.result = VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=VDACurrentAction.FINISHED,
        )

        self.get_logger().info(f"Parsed action parameters: {action_parameters}")
        goal_handle.succeed()

        return result


def main():
    rclpy.init()
    print('Starting luka_adapter')

    adapter = LukaAdapter()

    try:
        rclpy.spin(adapter)
    except KeyboardInterrupt:
        pass
    except BaseException:
        raise
    finally:
        adapter.on_shutdown()
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
