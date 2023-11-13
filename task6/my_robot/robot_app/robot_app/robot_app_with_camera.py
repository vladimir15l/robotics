# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		self.subscription = self.create_subscription(Image, '/depth/image', self.callback, 10)
		self.previousData = []
		self.previousVals = []
		self.isInited = False
		self.subscription # prevent unused variable warn

	def meanpreviousVals(self):
		mean = 0
		for i in self.previousVals:
			mean += i
		return mean / len(self.previousVals)
		   
	def callback(self, msg):
		dsensorImage = msg
		velMsg = Twist()
		velMsg.linear.x = 0.5
		self.isInited = True
		if (len(self.previousData) < 3):
			self.previousData.append(dsensorImage.data)
			return
		self.previousData.pop(0)
		self.previousData.append(dsensorImage.data)
		if(dsensorImage.width):
			targetPixelMeanVal = 0
			for im_id in range(len(self.previousData)):
				targetPixelMeanVal += self.previousData[im_id][int(dsensorImage.width*dsensorImage.height/2+dsensorImage.width/2)]
			targetPixelMeanVal /= len(self.previousData)
			self.get_logger().info("%d" % targetPixelMeanVal)
			self.previousVals.append(targetPixelMeanVal)
			if (len(self.previousVals) > 3):
				self.previousVals.pop(0)
			self.get_logger().info('I heard: "%u"' % self.meanpreviousVals())
			if(self.meanpreviousVals() != 0):
				velMsg.linear.x = 0.0
				self.isInited = True
		
		if(self.isInited):
			self.publisher_.publish(velMsg)

def main(args=None):
    rclpy.init(args=args)
    robot_app = PublisherSubscriber()
    rclpy.spin(robot_app)
	
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()