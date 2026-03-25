#!/usr/bin/env python3
"""
Assisted Teleop Node for ROS 2 Navigation

This script implements an Assisted Teleop node that interfaces with the Nav2 stack.
It manages the lifecycle of the AssistedTeleop action, handles cancellation requests,
and periodically clears costmaps to remove temporary obstacles.

Subscription Topics:
    /cmd_vel_teleop (geometry_msgs/Twist): Velocity commands for assisted teleop
    /cancel_assisted_teleop (std_msgs/Bool): Cancellation requests for assisted teleop

Parameters:
    ~/costmap_clear_frequency (double): Frequency in Hz for costmap clearing. Default: 2.0

:author: Addison Sears-Collins
:date: December 5, 2024
"""
import threading
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator
from rcl_interfaces.msg import ParameterDescriptor
from nav2_msgs.srv import ClearEntireCostmap


class AssistedTeleopNode(Node):
    """
    A ROS 2 node for managing Assisted Teleop functionality.
    """

    def __init__(self):
        """Initialize the AssistedTeleopNode."""
        super().__init__('assisted_teleop_node')

        # Declare and get parameters
        self.declare_parameter(
            'costmap_clear_frequency',
            2.0,
            ParameterDescriptor(description='Frequency in Hz for costmap clearing')
        )
        param_value = self.get_parameter('costmap_clear_frequency').value
        clear_frequency: float = float(param_value) if param_value is not None else 2.0

        # Initialize the BasicNavigator for interfacing with Nav2
        self.navigator = BasicNavigator('assisted_teleop_navigator')
        self.navigator.lifecycleShutdown = lambda: None  # ✅ FIX 1: blokuje kill Nav2
        self._clear_local = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )


        # Create subscribers for velocity commands and cancellation requests
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.cmd_vel_callback, 10)
        self.cancel_sub = self.create_subscription(
            Bool, '/cancel_assisted_teleop', self.cancel_callback, 10)

        # Initialize state variables
        self.assisted_teleop_active = False
        self.cancellation_requested = False

        # Create a timer for periodic costmap clearing with configurable frequency
        period = 1.0 / clear_frequency
        self.clear_costmaps_timer = self.create_timer(period, self.clear_costmaps_callback)

        self.get_logger().info(
            f'Assisted Teleop Node initialized with costmap clearing frequency: '
            f'{clear_frequency} Hz'
        )

        # ✅ FIX 2: waitUntilNav2Active() jest blokujące — odpal w osobnym wątku
        threading.Thread(
            target=self.navigator.waitUntilNav2Active,
            daemon=True
        ).start()

    def cmd_vel_callback(self, twist_msg: Twist) -> None:
        """Process incoming velocity commands and activate assisted teleop if needed."""
        if (abs(twist_msg.linear.x) > 0.0 or
            abs(twist_msg.linear.y) > 0.0 or
                abs(twist_msg.angular.z) > 0.0):
            self.cancellation_requested = False

        if not self.assisted_teleop_active and not self.cancellation_requested:
            if (abs(twist_msg.linear.x) > 0.0 or
                abs(twist_msg.linear.y) > 0.0 or
                    abs(twist_msg.angular.z) > 0.0):
                self.start_assisted_teleop()

    def start_assisted_teleop(self) -> None:
        """Start the Assisted Teleop action with indefinite duration."""
        self.assisted_teleop_active = True
        self.cancellation_requested = False
        # ✅ FIX 3: assistedTeleop() jest blokujące — musi działać w osobnym wątku
        threading.Thread(
            target=self.navigator.assistedTeleop,
            kwargs={'time_allowance': 0},
            daemon=True
        ).start()
        self.get_logger().info('AssistedTeleop activated with indefinite duration')

    def cancel_callback(self, msg: Bool) -> None:
        """Handle cancellation requests for assisted teleop."""
        if msg.data and self.assisted_teleop_active and not self.cancellation_requested:
            self.cancel_assisted_teleop()

    def cancel_assisted_teleop(self) -> None:
        """Cancel the currently running Assisted Teleop action."""
        if self.assisted_teleop_active:
            self.navigator.cancelTask()
            self.assisted_teleop_active = False
            self.cancellation_requested = True
            self.get_logger().info('AssistedTeleop cancelled')

    #def clear_costmaps_callback(self) -> None:
    #    """Periodically clear all costmaps to remove temporary obstacles."""
    #    if not self.assisted_teleop_active:
    #        return
    #    self.navigator.clearAllCostmaps()
    #    self.get_logger().debug('Costmaps cleared')
    def clear_costmaps_callback(self) -> None:
        if not self.assisted_teleop_active:
            return
        # call_async — nie spinuje BasicNavigatora, zero konfliktu
        if self._clear_local.service_is_ready():
            self._clear_local.call_async(ClearEntireCostmap.Request())
        if self._clear_global.service_is_ready():
            self._clear_global.call_async(ClearEntireCostmap.Request())
        self.get_logger().debug('Costmaps cleared')


def main():
    """Initialize and run the AssistedTeleopNode."""
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
            # ✅ FIX 4: NIE wywołuj lifecycleShutdown() — Nav2 zarządza sam
        rclpy.shutdown()


if __name__ == '__main__':
    main()
