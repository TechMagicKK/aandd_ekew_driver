import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from weight_scale_interfaces.msg import Weight
from weight_scale_interfaces.action import SetZero, GetWeight

class WeightAndScaleTestNode(Node):
    def __init__(self):
        super().__init__('aandd_ekew_test')
        self._weight_sub = self.create_subscription(Weight, '/aandd_ekew_node/weight', self.weight_callback, 10)
        self._set_zero_goal_handle = None
        self._set_zero_action = ActionClient(self, SetZero, '/aandd_ekew_node/set_zero')
        self._get_weight_goal_handle = None
        self._get_weight_action = ActionClient(self, GetWeight, '/aandd_ekew_node/get_weight')

    # ----------------------------------------------------------------------------------------------------------------------------
    async def weight_callback(self, msg):
        #self.get_logger().info(f'published_weight({msg.stamp.sec}.{msg.stamp.nanosec:09}, {msg.stable}, {msg.overload}, {msg.weight:.2f}[{msg.unit}])')
        pass

    # ----------------------------------------------------------------------------------------------------------------------------
    def cancel_set_zero(self):
        if self._set_zero_goal_handle is not None:
            future = self._set_zero_goal_handle.cancel_goal_async()
            response = future.result()

    async def set_zero_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        msg = feedback.weight
        self.get_logger().info(f'set_zero_feedback: {msg.stamp.sec}.{msg.stamp.nanosec:09}, {msg.stable}, {msg.overload}, {msg.weight:.2f}[{msg.unit}]')

    def set_zero(self, timeout_sec):
        self._set_zero_action.wait_for_server()
        goal = SetZero.Goal()
        goal.timeout = timeout_sec
        send_goal_future = self._set_zero_action.send_goal_async(
            goal,
            feedback_callback=self.set_zero_feedback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=20.0)
        self._set_zero_goal_handle = send_goal_future.result()
        if (self._set_zero_goal_handle is None) or (not self._set_zero_goal_handle.accepted):
            self.get_logger().info('exception: set_zero: goal')
            raise Exception('set_zero: goal')
        get_result_future = self._set_zero_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=20.0)
        result = get_result_future.result()
        if result is None:
            self.get_logger().info('exception: set_zero: result')
            raise Exception('set_zero: result')
        self._set_zero_goal_handle = None
        return result.result

    # ----------------------------------------------------------------------------------------------------------------------------
    def cancel_get_weight(self):
        if self._get_weight_goal_handle is not None:
            future = self._get_weight_goal_handle.cancel_goal_async()
            response = future.result()

    async def get_weight_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        msg = feedback.weight
        self.get_logger().info(f'get_weight_feedback: {msg.stamp.sec}.{msg.stamp.nanosec:09}, {msg.stable}, {msg.overload}, {msg.weight:.2f}[{msg.unit}]')

    def get_weight(self, timeout_sec):
        self._get_weight_action.wait_for_server()
        goal = GetWeight.Goal()
        goal.timeout = timeout_sec
        send_goal_future = self._get_weight_action.send_goal_async(
            goal,
            feedback_callback=self.get_weight_feedback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=20.0)
        self._get_weight_goal_handle = send_goal_future.result()
        if (self._get_weight_goal_handle is None) or (not self._get_weight_goal_handle.accepted):
            self.get_logger().info('exception: get_weight: goal')
            raise Exception('get_weight: goal')
        get_result_future = self._get_weight_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=20.0)
        result = get_result_future.result()
        if result is None:
            self.get_logger().info('exception: get_weight: result')
            raise Exception('get_weight: result')
        self._get_weight_goal_handle = None
        return result.result

# ----------------------------------------------------------------------------------------------------------------------------
def spin_sleep(executor, sec):
    future = executor._executor.submit(lambda: time.sleep(sec))
    executor.spin_until_future_complete(future)

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = WeightAndScaleTestNode()
    executor.add_node(node)
    try:
        res = node.set_zero(timeout_sec=1.0)
        node.get_logger().info(f'set_zero: success={res.success}, message={res.message}')
        while rclpy.ok():
            res = node.get_weight(timeout_sec=1.0)
            msg = res.weight
            node.get_logger().info(f'get_weight: {msg.stamp.sec}.{msg.stamp.nanosec:09}, {msg.stable}, {msg.overload}, {msg.weight:.2f}[{msg.unit}], {res.success}, {res.message}')
            spin_sleep(executor, 1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()

if __name__ == '__main__':
    main()
