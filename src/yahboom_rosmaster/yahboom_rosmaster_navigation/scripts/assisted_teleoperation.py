#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor


class AssistedTeleopNode(Node):

    def __init__(self):
        super().__init__('assisted_teleop_node')

        self.declare_parameter(
            'costmap_clear_frequency', 2.0,
            ParameterDescriptor(description='Frequency in Hz for costmap clearing')
        )
        param_value = self.get_parameter('costmap_clear_frequency').value
        clear_frequency: float = float(param_value) if param_value is not None else 2.0

        #self.navigator = BasicNavigator('assisted_teleop_navigator')
        self.navigator = BasicNavigator('assisted_teleop_navigator')
        self._nav_executor = MultiThreadedExecutor()
        self._nav_executor.add_node(self.navigator)
        self._nav_thread = threading.Thread(
            target=self._nav_executor.spin, daemon=True)
        self._nav_thread.start()
                
        self.navigator.lifecycleShutdown = lambda: None

        self._clear_local = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap')

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.cmd_vel_callback, 10)
        self.cancel_sub = self.create_subscription(
            Bool, '/cancel_assisted_teleop', self.cancel_callback, 10)

        self.assisted_teleop_active = False
        self.cancellation_requested = False

        period = 1.0 / clear_frequency
        self.clear_costmaps_timer = self.create_timer(period, self.clear_costmaps_callback)

        self.get_logger().info(
            f'Assisted Teleop Node initialized, costmap clear frequency: {clear_frequency} Hz')

        # Blokujące — OK bo rclpy.spin() jeszcze nie startował
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 active, ready for teleop.')

    def cmd_vel_callback(self, twist_msg: Twist) -> None:
        has_motion = (abs(twist_msg.linear.x) > 0.0 or
                      abs(twist_msg.linear.y) > 0.0 or
                      abs(twist_msg.angular.z) > 0.0)
        if has_motion:
            self.cancellation_requested = False
        if not self.assisted_teleop_active and not self.cancellation_requested and has_motion:
            self.start_assisted_teleop()

    def start_assisted_teleop(self) -> None:
        self.assisted_teleop_active = True
        self.cancellation_requested = False
        threading.Thread(
            target=self.navigator.assistedTeleop,
            kwargs={'time_allowance': 0},
            daemon=True
        ).start()
        self.get_logger().info('AssistedTeleop activated with indefinite duration')

    def cancel_callback(self, msg: Bool) -> None:
        if msg.data and self.assisted_teleop_active and not self.cancellation_requested:
            self.cancel_assisted_teleop()

    def cancel_assisted_teleop(self) -> None:
        if self.assisted_teleop_active:
            self.navigator.cancelTask()
            self.assisted_teleop_active = False
            self.cancellation_requested = True
            self.get_logger().info('AssistedTeleop cancelled')

    def clear_costmaps_callback(self) -> None:
        if not self.assisted_teleop_active:
            return
        if self._clear_local.service_is_ready():
            self._clear_local.call_async(ClearEntireCostmap.Request())
        if self._clear_global.service_is_ready():
            self._clear_global.call_async(ClearEntireCostmap.Request())
        self.get_logger().debug('Costmaps cleared')


def main():
    rclpy.init()
    node = None
    try:
        node = AssistedTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Node shutting down due to keyboard interrupt')
    except ROSInterruptException:
        if node:
            node.get_logger().info('Node shutting down due to ROS interrupt')
    finally:
        if node:
            node.cancel_assisted_teleop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
