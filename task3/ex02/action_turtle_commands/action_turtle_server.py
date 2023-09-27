import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from tutorial_action.action import MessageTurtleCommands


class CommandActionServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self.flag = 0
        self.twist = Twist()
        self.before_pose = Pose()
        self.after_pose = Pose()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'execute_turtle_commands',
            self.execute_callback)
        self.subscription # prevent unused variable warn


    def callback(self, msg):
        if self.flag == 1:
            self.before_pose = msg
            self.flag = 0
        self.after_pose = msg
        #self.get_logger().info('I heard: "%s"' % msg)
          

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        if goal_handle.request.command == 'forward':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = float(goal_handle.request.angle*3.14/180)
        elif goal_handle.request.command == 'turn_left':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = float(goal_handle.request.angle*3.14/180)
        elif goal_handle.request.command == 'turn_right':
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = -float(goal_handle.request.angle*3.14/180)
        self.flag = 1
        self.publisher_.publish(self.twist)
        self.get_logger().info('Publishing: "%s"' % self.twist)
        
        feedback_msg = MessageTurtleCommands.Feedback()

        feedback_msg.odom = 1
        self.get_logger().info('Feedback: {0}'.format(feedback_msg.odom))
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)
        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        return result


def main(args=None):
    rclpy.init(args=args)

    action_turtle_server = CommandActionServer()

    rclpy.spin(action_turtle_server)


if __name__ == '__main__':
    main()
