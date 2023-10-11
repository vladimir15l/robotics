from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
from math import pow, sqrt, atan2, pi 
import rclpy
import sys

class TurtleBot(Node):

     def __init__(self):
         super().__init__('move_to_goal')
         self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
         self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
         self.distance_tolerance = 0.01
         self.vel_msg = Twist()
         self.pose = Pose()
         self.goal_pose = Pose()

        # Get the input from the user.
         self.goal_pose.x = float(sys.argv[1])
         self.goal_pose.y = float(sys.argv[2])
         self.goal_pose.theta = float(sys.argv[3]) * pi / 180
         self.timer = self.create_timer(0.01, self.move2goal)


     def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

     def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

     def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

     def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

     def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

     def move2goal(self):
        """Moves the turtle to the goal."""
        if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:

            # Linear velocity in the x-axis.
            self.vel_msg.linear.x = self.linear_vel(self.goal_pose)
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            self.vel_msg.angular.x = 0.0
            self.vel_msg.angular.y = 0.0
            self.vel_msg.angular.z = self.angular_vel(self.goal_pose)

            # Publishing our vel_msg
            self.publisher.publish(self.vel_msg)
        else:
            # Stopping our robot after the movement is over.
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = self.goal_pose.theta
            self.publisher.publish(self.vel_msg)
            quit()


def main(args=None):
    rclpy.init(args=args)
    x = TurtleBot()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
