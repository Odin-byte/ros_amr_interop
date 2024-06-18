import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

import rclpy.time
from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction
from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.action import MoveBase
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import AGVPosition as VDAAGVPosition
from vda5050_msgs.msg import Velocity as VDAVelocity
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import CurrentAction as VDACurrentAction


class LukaAdapter(Node):
    def __init__(self) -> None:
        super().__init__("luka_adapter")

        self.get_logger().info(f"LUKA adapter starting")

        self._manufacturer_name = "robots"
        self._robot_name = "robot_1"

        base_interface_name = f"uagv/{self._manufacturer_name}/{self._robot_name}/"

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
            execute_callback=self.navigate_to_node_execute_callback,
            goal_callback=self.navigate_to_node_goal_callback,
            cancel_callback=self.navigate_to_node_cancel_callback,
            callback_group=nav_to_node_action_client_cb_group,
        )

        self._process_vda_action_ac_srv = ActionServer(
            node=self,
            action_type=ProcessVDAAction,
            action_name=base_interface_name + "adapter/vda_action",
            execute_callback=self.process_vda_action_callback,
        )

        # Move Base Action Client
        self._move_base_client = ActionClient(
            node=self, action_type=MoveBase, action_name="/move_base"
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

    def navigate_to_node_goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: {goal_request}")
        return GoalResponse.ACCEPT

    def navigate_to_node_cancel_callback(self, goal_handle):
        self.get_logger().info(f"Received cancel request")
        return CancelResponse.ACCEPT

    def navigate_to_node_execute_callback(self, goal_handle):
        node = goal_handle.request.node

        self.get_logger().info(
            f"Navigating to node '{goal_handle.request.node}', edge '{goal_handle.request.edge}'"
        )

        self.get_logger().info(
            f"Got position: x = {goal_handle.request.node.node_position.x}, y = {goal_handle.request.node.node_position.y}"
        )

        # Populate the Pose msg
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"

        target_pose.pose.position.x = goal_handle.request.node.node_position.x
        target_pose.pose.position.y = goal_handle.request.node.node_position.y
        target_pose.pose.position.z = 0.0

        target_pose.pose.orientation.w = goal_handle.request.node.node_position.theta

        # Start periodic feedback
        self._feedback_timer = self.create_timer(
            1.0, lambda: self.send_feedback(goal_handle)
        )

        self.get_logger().info("Sending request")
        # Initialize success flag
        self._move_base_success = None

        # Send Move Base goal asynchronously
        self.send_move_base_goal(target_pose)

        # Wait for move_base_goal_response_callback to set _move_base_success
        while rclpy.ok() and self._move_base_success == None:
            if self._move_base_success != None:
                break

        if self._move_base_success:
            goal_handle.succeed()
            self.get_logger().info("Navigation to node succeeded")
        else:
            goal_handle.abort()
            self.get_logger().info("Navigation to node failed")

        result = NavigateToNode.Result()
        return result

    def send_feedback(self, goal_handle):
        if goal_handle.is_cancel_requested or goal_handle.status in [
            GoalResponse.STATUS_SUCCEEDED,
            GoalResponse.STATUS_ABORTED,
        ]:
            self._feedback_timer.cancel()
            return

        feedback_msg = NavigateToNode.Feedback()
        # Get current position of the AGV
        current_position = self.get_current_position()

        feedback_msg.position.position_initialized = True
        feedback_msg.position.localization_score = (
            0.5  # How good is the quality of the position estimate -> 1.0 = perfect
        )
        feedback_msg.position.deviation_range = 0.01  # in meters
        feedback_msg.position.x = current_position.pose.position.x
        feedback_msg.position.y = current_position.pose.position.y
        feedback_msg.position.theta = current_position.pose.orientation.w

        # Publish feedback
        goal_handle.publish_feedback(feedback_msg)

        # Log feedback
        self.get_logger().info(f"Sending feedback: {current_position}")

    def get_current_position(self):
        # Here you should retrieve the actual current position of your AGV.
        # For demonstration purposes, let's return a dummy position.
        current_position = PoseStamped()
        current_position.header.stamp = self.get_clock().now().to_msg()
        current_position.header.frame_id = "map"

        # Replace these with actual current position values
        current_position.pose.position.x = 1.0
        current_position.pose.position.y = 2.0
        current_position.pose.position.z = 0.0
        current_position.pose.orientation.w = 1.0

        return current_position

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

    # Move Base Action client functions
    def send_move_base_goal(self, target_pose):
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose = target_pose

        self.get_logger().info("Waiting for server")
        self._move_base_client.wait_for_server()

        self.get_logger().info("Sending goal request")
        self._send_goal_future = self._move_base_client.send_goal_async(
            goal_msg, feedback_callback=self.move_base_feedback_callback
        )

        self._send_goal_future.add_done_callback(self.move_base_goal_response_callback)

    def move_base_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            self._move_base_success = False
            return

        self.get_logger().info("Goal accepted!")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.move_base_get_result_callback)

    def move_base_feedback_callback(self, feedback):
        self.get_logger().info(
            "Received feedback: {0}".format(feedback.feedback.sequence)
        )

    def move_base_get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "Goal succeeded! Result: {0}".format(result.sequence)
            )
            self._move_base_success = True
        else:
            self.get_logger().info("Goal failed with status: {0}".format(status))
            self._move_base_success = False


def main():
    rclpy.init()
    print("Starting luka_adapter")

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


if __name__ == "__main__":
    main()
