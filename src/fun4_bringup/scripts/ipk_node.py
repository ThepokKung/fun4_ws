#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from fun4_interfaces.srv import Qtarget, Wantink, Datapoint

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class IPKNode(Node):
    def __init__(self):
        super().__init__('ipk_node')

        """VALUE"""
        self.target_q = [0.0, 0.0, 0.0, 0.0]

        """SERVICE"""
        self.create_service(Datapoint, '/target_manual', self.Target_callback)

        """CLIENT"""
        self.send_target_client = self.create_client(Qtarget, '/target_q')

        self.ink_result = MutuallyExclusiveCallbackGroup()
        self.Want_ink_client = self.create_client(Wantink, '/want_ink', callback_group=self.ink_result)

        while not self.Want_ink_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /want_ink service...')

    def Target_callback(self, request, response):
        if request is not None:
            cal_target = Wantink.Request()
            cal_target.target.x = request.target.x
            cal_target.target.y = request.target.y
            cal_target.target.z = request.target.z

            # Asynchronous call to Wantink service
            future = self.Want_ink_client.call_async(cal_target)
            
            # Wait for the result and call the next function when done
            future.add_done_callback(self.Wanink_response_callback)

            response.success = True
            response.message = 'Waiting for Wantink to complete...'
        return response

    def Wanink_response_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f"Received solution: {result.solution}")
                
                # Update target_q with the result from Wantink service
                self.target_q[0] = result.solution[0]
                self.target_q[1] = result.solution[1]
                self.target_q[2] = result.solution[2]

                # Call SendqTartget_func to send target_q after Wantink is done
                self.SendqTartget_func(self.target_q[0], self.target_q[1], self.target_q[2])
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {e}")

    def SendqTartget_func(self, q1, q2, q3):
        msg = Qtarget.Request()
        msg.q1 = float(q1)
        msg.q2 = float(q2)
        msg.q3 = float(q3)
        
        # Use the correct client for the Qtarget service
        future = self.send_target_client.call_async(msg)
        future.add_done_callback(self.SendqTartget_response_callback)

    def SendqTartget_response_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f"Successfully sent Qtarget: {response}")
        except Exception as e:
            self.get_logger().error(f"Failed to send Qtarget with error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IPKNode()

    # Use MultiThreadedExecutor for concurrency
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
