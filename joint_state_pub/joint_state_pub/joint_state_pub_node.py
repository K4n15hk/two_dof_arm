#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisherNode(Node):
    l1=3.75
    l2=4.0
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2']  # Names of your robot's joints
        x1,y1=input("Enter pos(x y): ").split(" ")
        x=float(x1)
        y=float(y1)
        theta1,theta2=self.inverse_kinematics(x,y,self.l1,self.l2)
        if(theta1 == 69 and theta2==69):
            print("Cannot reach!!")
        else:
            joint_state.position = [theta1,theta2]  # Set desired joint positions here
        
        self.publisher.publish(joint_state)
   
    def inverse_kinematics(self,x,y,l1,l2):
        theta1=0.0
        theta2=0.0
        if((pow(x,2)+pow(y,2))> (pow(l1,2)+pow(l2,2))):
            return 69,69
            
        else:
            theta2=math.acos(((pow(x,2)+pow(y,2))-(pow(l1,2)+pow(l2,2)))/(2 * l1 * l2))
            if(x == 0.0):
                theta1=1.570795- math.acos(((pow(x,2)+pow(y,2))+(pow(l1,2)-pow(l2,2)))/(2 * l1 * pow(pow(x,2)+pow(y,2),0.5)))
            else:
                theta1=math.atan(y/x)- math.acos(((pow(x,2)+pow(y,2))+(pow(l1,2)-pow(l2,2)))/(2 * l1 * pow(pow(x,2)+pow(y,2),0.5)))
            
            if(x<0):
                return 3.14159+theta1,theta2
            return theta1,theta2
   

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

