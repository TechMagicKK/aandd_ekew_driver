import time
import re
import threading
import random
from serial import Serial
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from weight_scale_interfaces.msg import Weight
from weight_scale_driver.weight_scale_node import WeightScaleNode, WeightScaleError

#---------------------------------------------------------------------------------------
class EKEWNode(WeightScaleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self._client: Optional[Serial] = None
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('connect_timeout', 1.0)
        self.declare_parameter('use_fake', False)
        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._connect_timeout = self.get_parameter('connect_timeout').value
        self._use_fake = self.get_parameter('use_fake').value
        self.get_logger().info(f'params: port={self._port}')
        self.get_logger().info(f'params: baudrate={self._baudrate}[bps]')
        self.get_logger().info(f'params: connect_timeout={self._connect_timeout}')
        self.get_logger().info(f'params: use_fake={self._use_fake}')

    def connect(self):
        if self._use_fake:
            super().connect(port, unit, baudrate, timeout)
            return
        try:
            self.disconnect()
            self.get_logger().info(f'connect(port={self._port}, baudrate={self._baudrate}, timeout={self._connect_timeout})')
            self._client = Serial(port=self._port, baudrate=self._baudrate, timeout=self._connect_timeout)
        except Exception as e:
            raise WeightScaleError(f'connect: {e}')

    def disconnect(self):
        if self._use_fake:
            super().disconnect()
            return
        if self._client is not None:
            self.get_logger().info('disconnect()')
            self._client.close()
            self._client = None
        return True

    def set_zero(self):
        if self._use_fake:
            super().set_zero()
            return
        try:
            self._client.write(b'Z\r\n')
            line = self._client.readline()
            if line is None:
                raise Exception
            time.sleep(1.0)
        except Exception as e:
            raise WeightScaleError(f'set_zero: communication error')

    def get_weight(self) -> Weight():
        if self._use_fake:
            return super().get_weight()
        try:
            msg = Weight()
            msg.stable = False
            msg.overload = False
            msg.weight = 0.0
            msg.unit = 'kg'
            msg.stamp = self.get_clock().now().to_msg()
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
            raise WeightScaleError(f'get_weight: communication error')

#---------------------------------------------------------------------------------------
def main(args=None):
    try:
        rclpy.init(args=args)
        executor = rclpy.executors.MultiThreadedExecutor()
        node = EKEWNode('aandd_ekew_node')
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.manual_shutdown(debug_out=False)
        node.destroy_node()

if __name__ == '__main__':
    main()
