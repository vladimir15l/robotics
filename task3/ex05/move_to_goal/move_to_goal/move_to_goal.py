import sys
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class PublisherSubscriber(Node):

    def __init__(self):
        super().__init__('move_to_goal')
        self.turn = 0
        self.move = 0
        self.pose = Pose()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        self.msg = Twist()
        self.subscription # prevent unused variable warn
        self.timer = self.create_timer(1.2, self.timer_callback)
        
    def callback(self, msg):
        self.pose = msg

    def timer_callback(self):

        vel_msg = Twist()
        goal_pose = Pose()

        goal_pose.x = float(sys.argv[1])
        goal_pose.y = float(sys.argv[2])
 
        d = math.sqrt((goal_pose.x - self.pose.x)**2 + (goal_pose.y - self.pose.y)**2)
        angle = math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta
        
        if not self.turn:
             self.turn = 1
             vel_msg.angular.z = angle  
        elif not self.move:
             self.move = 1
             vel_msg.linear.x = d 
        else:
             self.get_logger().info("Goal Reached!! ")
             quit()
 
        # Publishing our vel_msg
        self.publisher_.publish(vel_msg)




def main():
    rclpy.init()

    move_to_goal = PublisherSubscriber()
    rclpy.spin(move_to_goal)

    move_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()