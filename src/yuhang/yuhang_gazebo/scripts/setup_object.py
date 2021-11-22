#!/usr/bin/env python


import sys
import copy
import rospy
import geometry_msgs.msg
import math
import tf

from geometry_msgs.msg import Pose


from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState




class SetupObjects(object):


  def __init__(self):

    rospy.sleep(1)
    model_names = ["pawnB1-201", "pawnB1-201_0", "pawnB1-201_1", "pawnB1-202", "pawnB1-202_0", "pawnB1-202_1", "pawnB1-203", "pawnB1-203_0", "pawnB1-203_1"]
    model_poses = [
        [-0.05, 0.45], [-0.05, 0.39], [-0.05, 0.33],
        [-0.15, 0.45], [-0.15, 0.39], [-0.15, 0.33],
        [-0.25, 0.45],[-0.25, 0.39], [-0.25, 0.33]
        ]
    model_poses = [
        [-0.09, 0.48], [-0.17, 0.48], [-0.25, 0.48],
        [-0.09, 0.42], [-0.17, 0.42], [-0.25, 0.42],
        [-0.09, 0.37],[-0.17, 0.37], [-0.25, 0.37]
        ]
    for i in range(len(model_names)-1,-1,-1):
      pose = [-0.25, 0.5, 2.1, 0, 0, 0]
      self.object_pose = self.get_pose(pose)

      self.state_msg = ModelState()
      self.state_msg.model_name = model_names[i]
      self.state_msg.pose = self.object_pose
      self.state_msg.pose.position.x = model_poses[i][0]
      self.state_msg.pose.position.y = model_poses[i][1]
      rospy.wait_for_service('/gazebo/set_model_state')
      try:
          set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
          resp = set_state( self.state_msg )
      except rospy.ServiceException, e:
          print "Service call failed: %s" % e
      rospy.sleep(0.15)

    sub_areas = [
        "aruco_visual_marker_0", "aruco_visual_marker_0_1", "aruco_visual_marker_0_2",
        "aruco_visual_marker_1", "aruco_visual_marker_1_1", "aruco_visual_marker_1_2",
        "aruco_visual_marker_2", "aruco_visual_marker_2_1", "aruco_visual_marker_2_2"]
    sub_areas_poses = [
        [0.25, 0.32],[0.17, 0.32], [0.09, 0.32],
        [0.25, 0.39],[0.17, 0.39], [0.09, 0.39],
        [0.25, 0.46],[0.17, 0.46], [0.09, 0.46]]
    for i in range(len(sub_areas)):
      pose = [0.25, 0.25, 0.9952, 0, 0, 0]
      self.object_pose = self.get_pose(pose)

      self.state_msg = ModelState()
      self.state_msg.model_name = sub_areas[i]
      self.state_msg.pose = self.object_pose
      self.state_msg.pose.position.x = sub_areas_poses[i][0]
      self.state_msg.pose.position.y = sub_areas_poses[i][1]
      rospy.wait_for_service('/gazebo/set_model_state')
      try:
          set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
          resp = set_state( self.state_msg )
      except rospy.ServiceException, e:
          print "Service call failed: %s" % e


    rospy.sleep(1)

  def get_pose(self, pose):

    object_pose = Pose()
    object_pose.position.x = pose[0]
    object_pose.position.y = pose[1]
    object_pose.position.z = pose[2]

    quaternion = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]

    return object_pose

def main():
  try:
    setup_obj = SetupObjects()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
