import math
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from tutorial_action.action import MessageTurtleCommands


class CommandActionServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self.twist = Twist()
        self.flag = 0
        self.after_pose = Pose()
        self.before_pose = Pose()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'execute_turtle_commands',
            self.execute_callback)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        self.subscription # prevent unused variable warn


    def callback(self, msg):
        self.after_pose = msg
        if self.flag == 1:
            self.before_pose = msg
            self.flag = 0
            self.get_logger().info('send flag {0}'.format(self.flag))

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.flag = 1
        if goal_handle.request.command == 'forward':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = float(goal_handle.request.angle*3.14159/180)
        elif goal_handle.request.command == 'turn_left':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = float(goal_handle.request.angle*3.14159/180)
        elif goal_handle.request.command == 'turn_right':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = -float(goal_handle.request.angle*3.14159/180)
        self.publisher_.publish(self.twist)
        self.get_logger().info('Publishing: "%s"' % self.twist)
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        while self.flag == 1 and (self.after_pose.linear_velocity==0 or self.after_pose.angular_velocity==0):
            pass
        self.get_logger().info('flag: {0}'.format(self.flag))
        while self.after_pose.linear_velocity != 0 or self.after_pose.angular_velocity != 0:
            feedback_msg.odom = int(math.sqrt((self.after_pose.x - self.before_pose.x)**2+(self.after_pose.y - self.before_pose.y)**2))
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.odom))
            goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        return result


def main(args=None):
    rclpy.init(args=args)
    action_turtle_server = CommandActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_turtle_server)
    executor.spin()


if __name__ == '__main__':
    main()
