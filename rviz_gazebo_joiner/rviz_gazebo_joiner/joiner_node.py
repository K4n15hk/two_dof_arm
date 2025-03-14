#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JoinerNode(Node):
	msg=JointState()
	def __init__(self):
		super().__init__('JoinerNode')
		self.publisher=self.create_publisher(JointState,'/joint_states',10)
		timer_period=1
#		self.timer=self.create_timer(timer_period,self.publish_joint_states)
		self.subscription=self.create_subscription(JointState,'/world/empty/model/urdf_model/joint_state',self.listener_callback,10)
		
		
	def listener_callback(self,data):
		global msg
		self.msg.header.stamp = self.get_clock().now().to_msg()
		self.msg.name=data.name
		self.msg.position=data.position
		self.publish_joint_states()
		
		
	def publish_joint_states(self):
		global msg
		self.publisher.publish(self.msg)
		
		
		
		

def main(args=None):
    rclpy.init(args=args)
    node = JoinerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
