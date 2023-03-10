import time
import sys
import re
import threading
import random
import asyncio
from serial import Serial
from typing import Optional

import rclpy
from rclpy.timer import Timer
from rclpy.time import Time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSPresetProfiles

from weight_scale_interfaces.msg import Weight
from weight_scale_interfaces.action import SetZero
from weight_scale_interfaces.action import GetWeight

#---------------------------------------------------------------------------------------
class EKEWError(Exception):
    pass

class EKEW(object):
    def __init__(self, node):
        self._node = node
        self._client: Optional[Serial] = None
        self._lock = threading.Lock()

    def connect(self, device, baudrate, timeout):
        self._device = device
        self._baudrate = baudrate
        self._timeout = timeout
        self.disconnect()
        try:
            with self._lock:
                self._node.get_logger().info(f'Serial(port={device}, baudrate={baudrate}, timeout={timeout})')
                self._client = Serial(port=device, baudrate=baudrate, timeout=timeout)
        except Exception as e:
            raise EKEWError(f'connect: {e}')

    def disconnect(self):
        if self._client is not None:
            with self._lock:
                self._client.close()
                self._client = None
        return True

    def set_zero(self):
        try:
            with self._lock:
                self._client.write(b'Z\r\n')
                line = self._client.readline()
                if line is None:
                    raise Exception
                time.sleep(1.0)
        except Exception as e:
            raise EKEWError(f'set_zero: communication error')

    def get_weight(self) -> Weight():
        line = None
        try:
            msg = Weight()
            msg.stable = False
            msg.overload = False
            msg.weight = 0.0
            msg.unit = 'kg'
            msg.stamp = self._node.get_clock().now().to_msg()
            with self._lock:
                self._client.write(b'Q\r\n')
                line = self._client.readline().strip().decode('utf-8')
            if line is None:
                raise Exception
            m = re.search(r'(ST|QT|US|OL),([ 0-9\.\+\-]+)([ a-zA-Z\%]+)', line)
            if m.group(1) == 'OL':
                # OverLoad
                msg.overload = True
                msg.unit = m.group(3).lstrip()
            elif (m.group(1) == 'ST') or (m.group(1) == 'QT'):
                # STable
                msg.stable = True
                msg.weight = float(m.group(2))
                msg.unit = m.group(3).lstrip()
            else:
                # must be US(UnStable)
                msg.stable = False
                msg.weight = float(m.group(2))
                msg.unit = m.group(3).lstrip()
            return msg
        except Exception as e:
            raise EKEWError(f'get_weight: communication error')

class EKEWTest(object):
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._weight_str = 'ST,+01234.56  g'

    def connect(self, device, baudrate, timeout):
        pass

    def disconnect(self):
        pass

    def set_zero(self):
        with self._lock:
            self._weight_str = 'ST,+00000.00  g'

    def get_weight(self) -> Weight():
        try:
            msg = Weight()
            msg.stamp = self._node.get_clock().now().to_msg()
            msg.stable = True if random.random() < 0.1 else False
            msg.overload = False
            with self._lock:
                line = self._weight_str
            m = re.search(r'(ST|QT|US|OL),([ 0-9\.\+\-]+)([ a-zA-Z\%]+)', line)
            msg.weight = float(m.group(2)) + random.random()
            msg.unit = m.group(3).lstrip()
            return msg
        except Exception as e:
            raise EKEWError(f'get_weight: {e}')

#---------------------------------------------------------------------------------------
class WeightAndScaleNode(Node):
    def __init__(self, **kwargs):
        self._ekew: Optional[EKEW] = None
        self._weight_publisher: Optional[Publisher] = None
        self._weight_publish_timer: Optional[Timer] = None
        self._set_zero_action: Optional[ActionServer] = None
        self._get_weight_action: Optional[ActionServer] = None
        super().__init__('aandd_ekew_node', **kwargs)

    def params_changed(self, params):
        for param in params:
            if param.name == 'rate':
                rate = param.value
                if rate < 0.001:
                    rate = 0.001
                if rate != self._rate:
                    self.get_logger().info(f'change rate param from {self._rate:.2f}[Hz] to {rate:.2f}[Hz]')
                    self._rate = rate
                    if self._weight_publish_timer is not None:
                        self.destroy_timer(self._weight_publish_timer)
                        self._weight_publish_timer = self.create_timer(1.0/self._rate, self.publish_weight)
            
    def setup_params(self):
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('connect_timeout', 1.0)
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('use_fake', False)
        self._device = self.get_parameter('device').value
        self._baudrate = self.get_parameter('baudrate').value
        self._connect_timeout = self.get_parameter('connect_timeout').value
        self._rate = self.get_parameter('rate').value
        self._use_fake = self.get_parameter('use_fake').value
        self.get_logger().info(f'params: device={self._device}')
        self.get_logger().info(f'params: baudrate={self._baudrate}[bps]')
        self.get_logger().info(f'params: connect_timeout={self._connect_timeout}')
        self.get_logger().info(f'params: rate={self._rate}[Hz]')
        self.get_logger().info(f'params: use_fake={self._use_fake}')
        self.add_post_set_parameters_callback(self.params_changed)

    async def publish_weight(self):
        try:
            if (self._weight_publisher is not None) and self._weight_publisher.is_activated:
                msg = self._ekew.get_weight()
                self.get_logger().info(f'publish_weight({msg.stamp.sec}.{msg.stamp.nanosec:09}, {msg.stable}, {msg.overload}, {msg.weight:.2f}[{msg.unit}])')
                self._weight_publisher.publish(msg)
        except Exception as e:
            self.get_logger().info(f'publish_weight: {e}')

    #-----------------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_configure()')
        if self._use_fake:
            self._ekew = EKEWTest(self)
        else:
            self._ekew = EKEW(self)
        self._weight_publisher = self.create_lifecycle_publisher(Weight, "~/weight", 10);
        self.get_logger().info(f'create_timer({1.0/self._rate:.2f})')
        self._weight_publish_timer = self.create_timer(1.0/self._rate, self.publish_weight)
        self._set_zero_action = ActionServer(
            self,
            SetZero,
            '~/set_zero',
            execute_callback=self.set_zero_execute_callback,
            cancel_callback=self.set_zero_cancel_callback,
            result_timeout = 60
        )
        self._get_weight_action = ActionServer(
            self,
            GetWeight,
            '~/get_weight',
            execute_callback=self.get_weight_execute_callback,
            cancel_callback=self.get_weight_cancel_callback,
            result_timeout = 60
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_activate()')
        self.get_logger().info(f'connect(device="{self._device}", baudrate={self._baudrate}[bps], connect_timeout={self._connect_timeout}[s])')
        try:
            self._ekew.connect(self._device, self._baudrate, self._connect_timeout)
        except Exception as e:
            self.get_logger().error(f'exception: {e}')
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_deactivate()')
        if self._ekew is not None:
            self.get_logger().info('disconnect()')
            self._ekew.disconnect()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_cleanup()')
        self.manual_shutdown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_shutdown()')
        self.manual_shutdown()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_error()')
        self.manual_shutdown()
        return TransitionCallbackReturn.SUCCESS

    def manual_shutdown(self, debug_out=True): # after exception
        if self._weight_publish_timer is not None:
            if debug_out: self.get_logger().info('cleanup timer')
            self.destroy_timer(self._weight_publish_timer)
            self._weight_publish_timer = None
        if self._weight_publisher is not None:
            if debug_out: self.get_logger().info('destroy weight_publisher')
            self.destroy_publisher(self._weight_publisher)
            self._weight_publisher = None
        if self._set_zero_action is not None:
            if debug_out: self.get_logger().info('destroy set_zero_actions')
            self._set_zero_action.destroy()
            self._set_zero_action = None
        if self._get_weight_action is not None:
            if debug_out: self.get_logger().info('destroy get_weight_actions')
            self._get_weight_action.destroy()
            self._get_weight_action = None
        if self._ekew is not None:
            if debug_out: self.get_logger().info('disconnect()')
            self._ekew.disconnect()
            self._ekew = None

    #-----------------------------------------------------------------------------
    async def set_zero_cancel_callback(self, goal_handle):
        self.get_logger().info('cancel set_zero()')
        return CancelResponse.ACCEPT

    async def set_zero_execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = SetZero.Result()
        feedback = SetZero.Feedback()
        weight = Weight()
        self.get_logger().info(f'set_zero({goal.timeout:5.2f}[s])')

        try:
            timeout = self.get_clock().now() + rclpy.duration.Duration(seconds=goal.timeout)
            while True:
                weight = self._ekew.get_weight()
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('set_zero: canceled')
                    result.success = False
                    result.message = 'canceled'
                    goal_handle.canceled()
                    return result
                if weight.stable:
                    self.get_logger().info('set_zero: stable')
                    self._ekew.set_zero()
                    result.success = True
                    result.message = 'success'
                    goal_handle.succeed()
                    return result
                elif Time.from_msg(weight.stamp) > timeout:
                    self.get_logger().info('set_zero: timeout')
                    result.success = False
                    result.message = 'timeout'
                    goal_handle.succeed()
                    return result
                else:
                    self.get_logger().info('set_zero: feedback')
                    feedback.weight = weight
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.1)
        except EKEWError as e:
            result.success = False
            result.weight = weight
            result.message = 'rs232c communication error'
            goal_handle.succeed()
            return result

    #-----------------------------------------------------------------------------
    async def get_weight_cancel_callback(self, goal_handle):
        self.get_logger().info('cancel get_weight()')
        return CancelResponse.ACCEPT

    async def get_weight_execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = GetWeight.Result()
        feedback = GetWeight.Feedback()
        weight = Weight()
        self.get_logger().info(f'get_weight({goal.timeout:5.2f}[s])')

        try:
            timeout = self.get_clock().now() + rclpy.duration.Duration(seconds=goal.timeout)
            while True:
                weight = self._ekew.get_weight()
                if goal_handle.is_cancel_requested:
                    result.weight = weight
                    result.success = False
                    result.message = 'canceled'
                    goal_handle.canceled()
                    return result
                if weight.stable:
                    result.weight = weight
                    result.success = True
                    result.message = 'success'
                    goal_handle.succeed()
                    return result
                elif Time.from_msg(weight.stamp) > timeout:
                    result.weight = weight
                    result.success = False
                    result.message = 'timeout'
                    goal_handle.succeed()
                    return result
                else:
                    feedback.weight = weight
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.1)
        except EKEWError as e:
            result.success = False
            result.weight = weight
            result.message = 'rs232c communication error'
            goal_handle.succeed()
            return result

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    #executor = rclpy.executors.SingleThreadedExecutor()
    node = WeightAndScaleNode()
    executor.add_node(node)
    node.setup_params()
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.manual_shutdown(debug_out=False)
        node.destroy_node()

if __name__ == '__main__':
    main()
