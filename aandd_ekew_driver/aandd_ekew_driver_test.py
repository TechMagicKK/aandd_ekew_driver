import signal
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions 
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from action_msgs.msg import GoalStatus

from weight_scale_interfaces.msg import Weight
from weight_scale_interfaces.action import SetZero
from weight_scale_interfaces.action import GetWeight

class WeightAndScaleTestNode(Node):
    def __init__(self):
        super().__init__('aandd_ekew_driver_test')
        self._weight_sub = self.create_subscription(Weight, '/aandd_ekew_driver_node/weight', self.weight_callback, 10)
        self._set_zero_goal_handle = None
        self._set_zero_action = ActionClient(self, SetZero, '/aandd_ekew_driver_node/set_zero')
        self._get_weight_goal_handle = None
        self._get_weight_action = ActionClient(self, GetWeight, '/aandd_ekew_driver_node/get_weight')

    # ----------------------------------------------------------------------------------------------------------------------------
    async def weight_callback(self, msg):
        self.get_logger().info(f'published_weight({msg.stamp.sec}.{msg.stamp.nanosec:*<9}, {msg.stable}, {msg.weight}[{msg.unit}])')

    # ----------------------------------------------------------------------------------------------------------------------------
    def cancel_set_zero(self):
        if self._set_zero_goal_handle is not None:
            future = self._set_zero_goal_handle.cancel_goal_async()
            response = future.result()

    async def set_zero_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        msg = feedback.weight
        self.get_logger().info(f'set_zero_feedback: {msg.stamp.sec}.{msg.stamp.nanosec:*<9}, {msg.stable}, {msg.weight:.2f}[{msg.unit}]')

    def set_zero(self, timeout_sec):
        self._set_zero_action.wait_for_server()
        goal = SetZero.Goal()
        goal.timeout = timeout_sec
        send_goal_future = self._set_zero_action.send_goal_async(
            goal,
            feedback_callback=self.set_zero_feedback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        self._set_zero_goal_handle = send_goal_future.result()
        if (self._set_zero_goal_handle is None) or (not self._set_zero_goal_handle.accepted):
            self.get_logger().info('exception: set_zero: goal')
            raise Exception('set_zero: goal')
        get_result_future = self._set_zero_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=10.0)
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
        self.get_logger().info(f'get_weight_feedback: {msg.stamp.sec}.{msg.stamp.nanosec:*<9}, {msg.stable}, {msg.weight:.2f}[{msg.unit}]')

    def get_weight(self, timeout_sec):
        self._get_weight_action.wait_for_server()
        goal = GetWeight.Goal()
        goal.timeout = timeout_sec
        send_goal_future = self._get_weight_action.send_goal_async(
            goal,
            feedback_callback=self.get_weight_feedback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        self._get_weight_goal_handle = send_goal_future.result()
        if (self._get_weight_goal_handle is None) or (not self._get_weight_goal_handle.accepted):
            self.get_logger().info('exception: get_weight: goal')
            raise Exception('get_weight: goal')
        get_result_future = self._get_weight_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=10.0)
        result = get_result_future.result()
        if result is None:
            self.get_logger().info('exception: get_weight: result')
            raise Exception('get_weight: result')
        self._get_weight_goal_handle = None
        return result.result

# ----------------------------------------------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WeightAndScaleTestNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            res = node.set_zero(timeout_sec=1.0)
            node.get_logger().info(f'set_zero: success={res.success}, message={res.message}')
            time.sleep(5)
            res = node.get_weight(timeout_sec=1.0)
            msg = res.weight
            node.get_logger().info(f'get_weight: {msg.stamp.sec}.{msg.stamp.nanosec:*<9}, {msg.stable}, {msg.weight:.2f}[{msg.unit}], {res.success}, {res.message}')
            time.sleep(5)
    except Exception as e:
        node.get_logger().info(f'exception: {e}')
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()
