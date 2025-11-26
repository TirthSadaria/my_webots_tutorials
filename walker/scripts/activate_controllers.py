#!/usr/bin/env python3
"""
Script to manually activate controllers after they're loaded.
This is a workaround for the controller activation timeout issue.
"""

import rclpy
from controller_manager_msgs.srv import SwitchController
import time
import sys

def activate_controllers():
    rclpy.init()
    node = rclpy.create_node('activate_controllers')
    
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')
    
    # Wait for service
    if not client.wait_for_service(timeout_sec=30.0):
        node.get_logger().error('Service /controller_manager/switch_controller not available')
        return False
    
    # Wait a bit more for controllers to be fully loaded
    time.sleep(2.0)
    
    # Activate joint_state_broadcaster
    request = SwitchController.Request()
    request.activate_controllers = ['joint_state_broadcaster']
    request.strictness = SwitchController.Request.BEST_EFFORT
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    if future.result() is not None:
        if future.result().ok:
            node.get_logger().info('Successfully activated joint_state_broadcaster')
        else:
            node.get_logger().warn('Failed to activate joint_state_broadcaster')
    else:
        node.get_logger().warn('No response from switch_controller service')
    
    time.sleep(1.0)
    
    # Activate diffdrive_controller
    request = SwitchController.Request()
    request.activate_controllers = ['diffdrive_controller']
    request.strictness = SwitchController.Request.BEST_EFFORT
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    if future.result() is not None:
        if future.result().ok:
            node.get_logger().info('Successfully activated diffdrive_controller')
            return True
        else:
            node.get_logger().warn('Failed to activate diffdrive_controller')
    else:
        node.get_logger().warn('No response from switch_controller service')
    
    return False

if __name__ == '__main__':
    success = activate_controllers()
    sys.exit(0 if success else 1)

