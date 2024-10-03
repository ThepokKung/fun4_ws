#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from fun4_interfaces.srv import Modeselect,Checkstate,Datapoint,Wantink,Qtarget

"""
Mode state
0 is not ready
1 is Inverse Pose Kinematics Mode (IPK)
2 is Tele-operation Mode (Teleop)
3 is Autonomous Mode (Auto)
"""
"""
Controll State
1 is ready
2 is working
"""

class RobotscheduleNode(Node):
    def __init__(self):
        super().__init__('robotschedule_node')

        """VALUE"""
        self.mode_sate = 0
        self.controll_state = 1

        """SERVICE"""
        self.create_service(Modeselect,'/mode',self.Modeselect_callback)
        self.create_service(Datapoint,'/target_manual',self.Targetmanual_callback) #Data Point

        """CLIENT"""
        self.check_state_cb = MutuallyExclusiveCallbackGroup()
        self.check_confroller_client = self.create_client(Checkstate,'/checkcontroller_state',callback_group=self.check_state_cb)
        self.want_ink_cb = MutuallyExclusiveCallbackGroup()
        self.want_ink_clinet = self.create_client(Wantink,'/want_ink',callback_group=self.want_ink_cb)
        self.sendq_target_cb = MutuallyExclusiveCallbackGroup()
        self.sendq_target_clinet = self.create_client(Qtarget,'/target_q',callback_group=self.sendq_target_cb)

        """Start node text"""
        self.get_logger().info(f'Starting : /{self.get_name()}') 

    """Mode Select"""
    def Modeselect_callback(self, request, response):
        self.Check_controller_func()
        self.get_logger().info(f'Controller state is : {self.controll_state}') #DEBUG
        if self.controll_state != 2:
            if request.modeselect == 1:
                self.mode_sate = 1
                response.success = True
                response.message = 'You select Mode "1" -> Inverse Pose Kinematics Mode (IPK)'
                self.get_logger().info('You select Mode "1" -> Inverse Pose Kinematics Mode (IPK)')
            elif request.modeselect == 2:
                self.mode_sate = 2
                response.success = True
                response.message = 'You select Mode "2" -> Tele-operation Mode (Teleop)'
                self.get_logger().info('You select Mode "2" -> Tele-operation Mode (Teleop)')
            elif request.modeselect == 3:
                self.mode_sate = 3
                response.success = True
                response.message = 'You select Mode "3" -> Autonomous Mode (Auto)'
                self.get_logger().info('You select Mode "3" -> Autonomous Mode (Auto)')
            else:
                response.success = False
                response.message = f'You select Mode "{request.modeselect}" -> no mode'
                self.get_logger().info(f'You select Mode "{request.modeselect}" -> no mode' )
        else:
            response.success = False
            response.message = 'Controller is working now'
            self.get_logger().info('Controller is working now')
        return response

    """Start Check Controller func callback"""
    # def Check_controller_func(self):
    #     msg = Checkstate.Request()
    #     msg.checkstate = True
    #     future = self.check_confroller_client.call_async(msg)
    #     future.add_done_callback(self.Check_controller_response)
        
    # def Check_controller_response(self,future):
    #     try:
    #         response = future.result()
    #         if response.success == True:
    #             self.controll_state = 1
    #         else:
    #             self.controll_state = 2
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed with error: {e}")

    def Check_controller_func(self):
        msg = Checkstate.Request()
        msg.checkstate = True
        result = self.check_confroller_client.call(msg)
        if result.success:
            self.controll_state = 1 
        else:
            self.controll_state = 2
    
    def Check_controller_func(self):
        msg = Checkstate.Request()
        msg.checkstate = True
        futere_check_controller = self.check_confroller_client.call_async(msg)
        rclpy.spin_until_future_complete(self, futere_check_controller) # Wait service

        if futere_check_controller.done():
            controller_state = futere_check_controller.result()
            if controller_state.success == 1:
                self.controll_state = 1 
            else:
                self.controll_state = 2
    """END Check Controller func callback"""

    """START Targetmanual func callback"""
    def Targetmanual_callback(self, request, response):
        cal_target = Wantink.Request()
        cal_target.target.x = request.target.x
        cal_target.target.y = request.target.y
        cal_target.target.z = request.target.z
        self.get_logger().info(f'IPK GO TARGET : {cal_target.target}')

        result = self.want_ink_clinet.call(cal_target)
        msg_robot_check = Checkstate.Request()
        msg_robot_check.checkstate = True
        check_robot = self.check_confroller_client.call(msg_robot_check)

        self.get_logger().info(f'response : {result.solution}')
        self.get_logger().info(f'Check rebot response : {check_robot}')

        if result.success == True and check_robot.success == True:
            target_q = Qtarget.Request()
            target_q.q1 = result.solution[0]
            target_q.q2 = result.solution[1]
            target_q.q3 = result.solution[2]
            
            self.sendq_target_clinet.call_async(target_q)

            response.success = True
            response.message = 'Good we will go'
        else:
            response.success = False
            response.message = 'No we cant go or this robot working'    
        return response

    # def Targetmanual_callback(self,request,response):
    #     # self.Check_controller_func()
    #     msg = Checkstate.Request()
    #     msg.checkstate = True
    #     futere_check_controller = self.check_confroller_client.call_async(msg)
        
    #     rclpy.spin_until_future_complete(self, futere_check_controller) # Wait service

    #     if futere_check_controller.done():
    #         controller_state = futere_check_controller.result()
    #         if controller_state.success == 1:
    #             self.controll_state = 1 
    #         else:
    #             self.controll_state = 2

    #     self.get_logger().info(f'controller is : {self.controll_state}') #DEBUG

    #     if self.controll_state != 2:
    #         if self.mode_sate == 1:
                
    #             cal_target = Wantink.Request()
    #             cal_target.target.x = request.target.x
    #             cal_target.target.y = request.target.y
    #             cal_target.target.z = request.target.z
    #             self.get_logger().info(f'IPK GO TARGET : {cal_target.target}') #DEBUG

    #             future_ink_result = self.want_ink_clinet.call_async(cal_target)

    #             rclpy.spin_until_future_complete(self, future_ink_result) # Wait service

    #             if future_ink_result.done():
    #                 ink_result = future_ink_result.result()
    #                 self.get_logger().info(f'ink_result is : {ink_result.success}') #DEBUG
    #                 if ink_result.success == True:
    #                     target_q = Qtarget.Request()
    #                     target_q.q1 = ink_result.solution[0]
    #                     target_q.q2 = ink_result.solution[1]
    #                     target_q.q3 = ink_result.solution[2]

    #                     self.sendq_target_clinet.call_async(target_q)

    #                     response.success = True
    #                     response.message = 'Yes wait for it'
    #                 else:
    #                     response.success = False
    #                     response.message = "We can't go"
    #             # else:
    #             #     response.success = False
    #             #     response.message = "We can't go"
    #         else:
    #             response.success = False
    #             response.message = 'Not cerrent in IPK Mode'
    #     else:
    #         response.success = False
    #         response.message = 'Controller is working'
    #     self.get_logger().info(response.message)
    #     return response

    # def Targetmanual_callback(self, request, response):
    #     # Check controller state asynchronously
    #     self.Check_controller_func()
    #     self.get_logger().info(f'controller is : {self.controll_state}')

    #     if self.controll_state != 2:
    #         if self.mode_sate == 1:
    #             # Prepare the target for the service call
    #             cal_target = Wantink.Request()
    #             cal_target.target.x = request.target.x
    #             cal_target.target.y = request.target.y
    #             cal_target.target.z = request.target.z
    #             self.get_logger().info(f'IPK GO TARGET: {cal_target.target}')  # DEBUG

    #             # Call the /want_ink service asynchronously
    #             future_ink_result = self.want_ink_clinet.call_async(cal_target)
    #             future_ink_result.add_done_callback(lambda future: self.handle_ink_result(future, response))
    #         else:
    #             response.success = False
    #             response.message = 'Not in IPK Mode'
    #             return response
    #     else:
    #         response.success = False
    #         response.message = 'Controller is working'
    #         return response

    # def handle_ink_result(self, future, response):
    #     try:
    #         ink_result = future.result()

    #         self.get_logger().info(f'ink_result is: {ink_result}')  # DEBUG

    #         if ink_result.success:
    #             # Prepare and send the target asynchronously
    #             target_q = Qtarget.Request()
    #             target_q.q1 = ink_result.solution[0]
    #             target_q.q2 = ink_result.solution[1]
    #             target_q.q3 = ink_result.solution[2]

    #             self.sendq_target_clinet.call_async(target_q)

    #             # Set the response to success
    #             response.success = True
    #             response.message = 'Yes, wait for it'
    #         else:
    #             response.success = False
    #             response.message = 'Failed to calculate IK'
    #     except Exception as e:
    #         self.get_logger().error(f'Service call to /want_ink failed: {e}')
    #         response.success = False
    #         response.message = f'Failed to calculate IK: {e}'

    #     return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotscheduleNode()
    # rclpy.spin(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
