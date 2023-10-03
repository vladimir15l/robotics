import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tutorial_action.action import MessageTurtleCommands
import time

class CommandActionClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'execute_turtle_commands')

    def send_goal(self):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = 'forward'
        goal_msg.s = 4
        goal_msg.angle = 0
        self._action_client.wait_for_server()        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        goal_msg.command = 'turn_right'
        goal_msg.s = 0
        goal_msg.angle = 90
        self._action_client.wait_for_server()        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        goal_msg.command = 'forward'
        goal_msg.s = 2
        goal_msg.angle = 0
        self._action_client.wait_for_server()        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        goal_msg.command = 'turn_right'
        goal_msg.s = 0
        goal_msg.angle = 90
        self._action_client.wait_for_server()        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        goal_msg.command = 'forward'
        goal_msg.s = 8
        goal_msg.angle = 0
        self._action_client.wait_for_server()        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_result_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')


    def goal_response_result_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.odom))


def main(args=None):
    rclpy.init(args=args)

    action_client = CommandActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

