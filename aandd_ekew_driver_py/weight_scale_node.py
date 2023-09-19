import time
import sys
import re
import threading
import random
import asyncio
from typing import Optional

import rclpy
from rclpy.timer import Timer
from rclpy.time import Time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rcl_interfaces.msg import SetParametersResult

from weight_scale_interfaces.msg import Weight
from weight_scale_interfaces.action import SetZero
from weight_scale_interfaces.action import GetWeight

#---------------------------------------------------------------------------------------
class WeightScaleError(Exception):
    pass

#---------------------------------------------------------------------------------------
class WeightScaleNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self._weight_publisher: Optional[Publisher] = None
        self._weight_publish_timer: Optional[Timer] = None
        self._set_zero_action: Optional[ActionServer] = None
        self._get_weight_action: Optional[ActionServer] = None
        self._lock = threading.Lock()
        self.declare_parameter('fake_weight', 13.57)
        self._weight = self.get_parameter('fake_weight').value
        self.get_logger().info(f'params: fake_weight={self._weight:.2f}[g]')
        self.declare_parameter('rate', 5.0)
        self._rate = self.get_parameter('rate').value
        self.get_logger().info(f'params: rate={self._rate}[Hz]')
        self.add_on_set_parameters_callback(self.params_changed)

    def params_changed(self, params) -> Optional[SetParametersResult]:
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
            elif param.name == 'fake_weight':
                weight = param.value
                if weight < 0.0:
                    weight = 0.0
                if weight > 900.0:
                    weight = 900.0
                if weight != self._weight:
                    self.get_logger().info(f'change weight param from {self._weight:.2f}[g] to {weight:.2f}[g]')
                    self._weight = weight
        return SetParametersResult(successful=True)

    #---------------------------------------------------------------------------------------
    def connect(self):
        self.get_logger().info('connect()')

    def disconnect(self):
        pass

    def set_zero(self):
        self._weight = 0.0

    def get_weight(self) -> Weight():
        try:

            msg = Weight()
            msg.stamp = self.get_clock().now().to_msg()
            msg.stable = True if random.random() < 0.1 else False
            msg.overload = False
            self._weight = self.get_parameter('fake_weight').value
            msg.weight = self._weight + random.random()
            msg.unit = 'g'
            return msg
        except Exception as e:
            raise WeightScaleError(f'get_weight: {e}')

    #---------------------------------------------------------------------------------------
    async def publish_weight(self):
        try:
            if (self._weight_publisher is not None) and (self._weight_publisher.is_activated):
                with self._lock:
                    msg = self.get_weight()
                # self.get_logger().info(f'publish_weight({msg.stamp.sec}.{msg.stamp.nanosec:09}, {msg.stable}, {msg.overload}, {msg.weight:.2f}[{msg.unit}])')
                self._weight_publisher.publish(msg)
        except Exception as e:
            self.get_logger().info(f'publish_weight: {e}')
            # change self state to error
            self.trigger_deactivate()
            
    #-----------------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_configure()')
        self._weight_publisher = self.create_lifecycle_publisher(Weight, "~/weight", 10)
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
        try:
            with self._lock:
                self.connect()
        except Exception as e:
            self.get_logger().error(f'exception: {e}')
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_deactivate()')
        with self._lock:
            self.get_logger().info('disconnect()')
            self.disconnect()
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
        with self._lock:
            if debug_out: self.get_logger().info('disconnect()')
            self.disconnect()

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
                with self._lock:
                    weight = self.get_weight()
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('set_zero: canceled')
                    result.success = False
                    result.message = 'canceled'
                    goal_handle.canceled()
                    return result
                if weight.stable:
                    self.get_logger().info('set_zero: stable')
                    with self._lock:
                        self.set_zero()
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
        except WeightScaleError as e:
            result.success = False
            result.weight = weight
            result.message = 'communication error'
            goal_handle.succeed()
            # change self state to error
            self.trigger_deactivate()
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
                with self._lock:
                    weight = self.get_weight()
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
        except WeightScaleError as e:
            result.success = False
            result.weight = weight
            result.message = 'communication error'
            goal_handle.succeed()
            # change self state to error
            self.trigger_deactivate()
            return result

