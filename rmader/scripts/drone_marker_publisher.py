#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2023, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import roslib
import rospy
import math
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
from numpy import linalg as LA
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply, random_quaternion
from visualization_msgs.msg import Marker
import tf

class DroneMarkerPublisher:

    def __init__(self):

        print("Starting drone_marker_publisher node")

        self.state = State()
        self.world = PoseStamped()
        self.world.pose.position.x = rospy.get_param('~x', 0.0)
        self.world.pose.position.y = rospy.get_param('~y', 0.0)
        self.world.pose.position.z = rospy.get_param('~z', 0.0)
        yaw = rospy.get_param('~yaw', 0.0); pitch=0.0; roll=0.0
        quat = quaternion_from_euler(yaw, pitch, roll, 'rzyx')
        self.world.pose.orientation.x = quat[0]
        self.world.pose.orientation.y = quat[1]
        self.world.pose.orientation.z = quat[2]
        self.world.pose.orientation.w = quat[3]

        name = rospy.get_namespace()
        self.name = name[1:-1]

        self.pubMarkerDrone = rospy.Publisher('drone_marker', Marker, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_drone_marker)
        rospy.sleep(1.0)

    def goalCB(self, data):

        thrust=np.array([data.a.x, data.a.y, data.a.z + 9.81]) 
        thrust_normalized=thrust/np.linalg.norm(thrust)
        a=thrust_normalized[0]
        b=thrust_normalized[1]
        c=thrust_normalized[2]
        qabc = random_quaternion()
        tmp=(1/math.sqrt(2*(1+c)))
        qabc[3] = tmp*(1+c) #w
        qabc[0] = tmp*(-b)  #x
        qabc[1] = tmp*(a)   #y
        qabc[2] = 0         #z
        qpsi = random_quaternion()
        qpsi[3] = math.cos(data.psi/2.0)  #w
        qpsi[0] = 0                       #x 
        qpsi[1] = 0                       #y
        qpsi[2] = math.sin(data.psi/2.0)  #z

        w_q_b=quaternion_multiply(qabc,qpsi)
        self.state.header.frame_id="world"
        self.state.pos=data.p
        self.state.vel=data.v
        self.state.quat.w=w_q_b[3]  #w
        self.state.quat.x=w_q_b[0]  #x 
        self.state.quat.y=w_q_b[1]  #y
        self.state.quat.z=w_q_b[2]  #z

    def publish_drone_marker(self, timer):
        marker=Marker()
        marker.id=1
        marker.ns="mesh_"+self.name
        marker.header.frame_id="world"
        marker.type=marker.MESH_RESOURCE
        marker.action=marker.ADD
        marker.pose.position.x=self.state.pos.x
        marker.pose.position.y=self.state.pos.y
        marker.pose.position.z=self.state.pos.z
        marker.pose.orientation.x=self.state.quat.x
        marker.pose.orientation.y=self.state.quat.y
        marker.pose.orientation.z=self.state.quat.z
        marker.pose.orientation.w=self.state.quat.w
        marker.lifetime = rospy.Duration.from_sec(0.0)
        marker.mesh_use_embedded_materials=True
        marker.mesh_resource="package://panther_gazebo/meshes/quadrotor/quadrotor.dae"
        marker.scale.x=0.75
        marker.scale.y=0.75
        marker.scale.z=0.75
        self.pubMarkerDrone.publish(marker)  

def startNode():
    c = DroneMarkerPublisher()
    rospy.Subscriber("goal", PoseStamped, c.goalCB)
    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    print ("Starting drone_marker_publisher node for: " + ns)
    rospy.init_node('drone_marker')
    startNode()