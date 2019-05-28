#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

import numpy as np

import math
from math import radians, degrees
from hsrb_interface import Robot, ItemTypes
from hsrb_interface import geometry

from hsrb_interface import robot as _robot

_robot.enable_interactive()



def callback(data):
    
    whole_body.move_to_neutral()
    omni_base.go_abs(0,0,0)
    
    omni_base.go_rel(x=-0.1, y=-0.08)
    
    data = data.data.reshape(-1, 2)
    dis = [(x-0)**2+(y-240)**2 for (x,y) in data]
    ob_to = data[np.argmax(dis)]
    ob_from = data[np.argmin(dis)]
    print "from:", ob_from, ", to", ob_to
    
    omni_base.go_rel(y=-(ob_from[0]-320.)/306.)
    gripper.set_distance(1.3)
    
    omni_base.go_rel(x=0.1)
    whole_body.move_end_effector_pose(
        geometry.pose(x=-0.18, z=0.16, ej=math.radians(-90.0)), ref_frame_id='hand_palm_link')
    gripper.apply_force(1.0)
    
    whole_body.move_to_neutral()
    omni_base.go_abs(0,0,0)
    
    omni_base.go_rel(x=-0.1, y=-0.08)
    omni_base.go_rel(y=-(ob_to[0]-320.)/306.)
    
    omni_base.go_rel(x=0.1)
    whole_body.move_end_effector_pose(
        geometry.pose(x=-0.10, z=0.16, ej=math.radians(-90.0)), ref_frame_id='hand_palm_link')
    gripper.set_distance(1.3)

    whole_body.move_to_neutral()
    omni_base.go_abs(0,0,0)
    
with Robot() as robot:
    whole_body = robot.try_get('whole_body')
    omni_base = robot.try_get('omni_base')
#     collision_world = robot.try_get('global_collision_world')
#     suction = robot.try_get('suction')
    gripper = robot.try_get('gripper')
#     wrist_wrench = robot.try_get('wrist_wrench')
#     marker = robot.try_get('marker')
#     battery = robot.try_get('battery')
#     tts = robot.try_get('default_tts')
#     shell(LOGO)

#     rospy.init_node('listener')
    
    omni_base.go_abs(0,0,0)
    whole_body.move_to_neutral()
    
    rospy.Subscriber("contours", numpy_msg(Floats), callback, queue_size=1)
    rospy.spin()