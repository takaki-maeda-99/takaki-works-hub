import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from std_msgs.msg import Bool


class LifecycleController(Node):
    def __init__(self):
        super().__init__("controller")
        self.subscription = self.create_subscription(
            Bool, "lidar_condition", self.condition_callback, 10
        )
        self.client_1st = self.create_client(ChangeState, "/urg_node2_1st/change_state")
        self.client_2nd = self.create_client(ChangeState, "/urg_node2_2nd/change_state")
        self._before_state = State.PRIMARY_STATE_UNKNOWN

    async def condition_callback(self, msg):
        if self._before_state == State.PRIMARY_STATE_ACTIVE:
            return
        new_state = (
            State.PRIMARY_STATE_INACTIVE if not msg.data else State.PRIMARY_STATE_ACTIVE
        )
        await self.transition_node_state(new_state)
        self._before_state = new_state

    async def transition_node_state(self, new_state):
        while not self.client_1st.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        while not self.client_2nd.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        req = ChangeState.Request()
        req.transition.id = new_state
        future = self.client_1st.call_async(req)
        future2 = self.client_2nd.call_async(req)
        await future
        await future2


def main(args=None):
    rclpy.init(args=args)
    controller = LifecycleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
