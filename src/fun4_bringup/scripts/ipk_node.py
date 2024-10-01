#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from fun4_interfaces.srv import Qtarget, Wantink, Datapoint,Setstate,Checkstate

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class IPKNode(Node):
    def __init__(self):
        super().__init__('ipk_node')

        """VALUE"""
        self.target_q = [0.0, 0.0, 0.0, 0.0]
        self.my_state = False

        """SERVICE"""
        self.create_service(Datapoint, '/target_manual', self.Target_callback)
        self.create_service(Setstate,'/setipk_mode',self.Setstate_callback)

        """CLIENT"""
        self.ink_result_cb = MutuallyExclusiveCallbackGroup()
        self.call_state_cb = MutuallyExclusiveCallbackGroup()
        self.call_targetq_cb = MutuallyExclusiveCallbackGroup()
        self.send_target_client = self.create_client(Qtarget, '/target_q',callback_group=self.call_targetq_cb)
        self.Want_ink_client = self.create_client(Wantink, '/want_ink', callback_group=self.ink_result_cb)
        self.check_controller_client = self.create_client(Checkstate,'/checkcontroller_state',callback_group=self.call_state_cb)

    # def Target_callback(self, request, response):
    #     if self.my_state == True:
    #         cal_target = Wantink.Request()
    #         cal_target.target.x = request.target.x
    #         cal_target.target.y = request.target.y
    #         cal_target.target.z = request.target.z
    #         self.get_logger().info(f'IPK GO TARGET : {cal_target.target}')

    #         future_want_ink = self.Want_ink_client.call_async(cal_target)
    #         future_want_ink.add_done_callback(lambda future: self.wantink_response(future, response, request))
    #     else:
    #         response.success = False
    #         response.message = 'Not correct mode'
    #         return response

    # def wantink_response(self, future,response,request):
    #     try:
    #         result = future.result()

    #         msg_robot_check = Checkstate.Request()
    #         msg_robot_check.checkstate = True
    #         future_check_robot = self.check_controller_client.call_async(msg_robot_check)
    #         future_check_robot.add_done_callback(lambda future: self.handle_check_robot_response(future, result, response))

    #     except Exception as e:
    #         self.get_logger().error(f"Service call to /want_ink failed: {e}")
    #         response.success = False
    #         response.message = 'Failed to calculate IK'
    #         self.get_logger().error("Failed to calculate IK")
    #         return responsehttps://www.facebook.com/

    # def robotcheck_response(self,future,result,response):
    #     try:
    #         check_robot = future.result()

    #         if result.success and check_robot.success:
    #             target_q = Qtarget.Request()
    #             target_q.q1 = result.solution[0]
    #             target_q.q2 = result.solution[1]
    #             target_q.q3 = result.solution[2]

    #             # Send the target to the robot asynchronously
    #             self.send_target_client.call_async(target_q)

    #             response.success = True
    #             response.message = 'Good, we will go'
    #         else:
    #             response.success = False
    #             response.message = 'No, we cannot go or robot is busy'
    #     except Exception as e:
    #         self.get_logger().error(f"Service call to /checkcontroller_state failed: {e}")
    #         response.success = False
    #         response.message = 'Failed to check robot state'

    def Target_callback(self, request, response):
        if self.my_state == True:
            cal_target = Wantink.Request()
            cal_target.target.x = request.target.x
            cal_target.target.y = request.target.y
            cal_target.target.z = request.target.z
            self.get_logger().info(f'IPK GO TARGET : {cal_target.target}')

            result = self.Want_ink_client.call(cal_target)
            msg_robot_check = Checkstate.Request()
            msg_robot_check.checkstate = True
            check_robot = self.check_controller_client.call(msg_robot_check)

            self.get_logger().info(f'response : {result.solution}')
            self.get_logger().info(f'Check rebot response : {check_robot}')

            if result.success == True and check_robot.success == True:
                target_q = Qtarget.Request()
                target_q.q1 = result.solution[0]
                target_q.q2 = result.solution[1]
                target_q.q3 = result.solution[2]

                self.send_target_client.call_async(target_q)

                response.success = True
                response.message = 'Good we will go'
            else:
                response.success = False
                response.message = 'No we cant go or this robot working'
            return response
        else:
            response.success = False
            response.message = 'Not cerrent this mode'
            return response
        
    def Setstate_callback(self,request, response):
        if request.setstate == True:
            self.my_state = True
            response.success = True
            response.message = "IPK MODE IS ON"
        elif request.setstate == False:
            self.my_state = False
            response.success = True
            response.message = "IPK MODE IS OFF"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = IPKNode()
    # rclpy.spin(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
