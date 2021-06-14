#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander     
import moveit_msgs.msg
import geometry_msgs.msg
import math as m
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, quaternion_from_matrix,rotation_matrix


def all_close(goal, actual, tolerance):

  """

  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list

  @param: goal       A list of floats, a Pose or a PoseStamped

  @param: actual     A list of floats, a Pose or a PoseStamped

  @param: tolerance  A float

  @returns: bool

  """

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
      return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
      return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pp',anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name_1 = "manipulator"
        group_1 = moveit_commander.MoveGroupCommander(group_name_1)
        
        #group_name_2 = "hand"
        #group_2 = moveit_commander.MoveGroupCommander(group_name_2)
 

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        planning_frame = group_1.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)

        eef_link = group_1.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        group_names = robot.get_group_names()
        print ("============ Robot Groups:", robot.get_group_names())

        print ("============ Printing robot state")
        print (robot.get_current_state())
       

        print ("=========== Printing named pose")
        print (group_1.get_named_targets())

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group_1 = group_1
        #self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self,joint_goal):
        group_1 = self.group_1
        
        group_1.go(joint_goal, wait=True)
        group_1.stop()

    def go_to_pose_goal(self,x,y,z):
        group_1 = self.group_1
        pose_goal = group_1.get_current_pose()
            
        pose_goal = geometry_msgs.msg.Pose()
        #quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
     
        R = rotation_matrix(m.pi,(1,0,0))
        quaternion = quaternion_from_matrix(R)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

     
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        group_1.set_pose_target(pose_goal)

        plan = group_1.go(wait=True)
        group_1.stop()
        group_1.clear_pose_targets()
        current_pose = self.group_1.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def set_named_target(self):
        group_1 = self.group_1
        group_1.set_named_target("up")
        plan1 = group_1.plan()
        group_1.execute(plan1,wait=False)

    def elip_parametrized_path(self,wpose):
        waypoints = []
        for i in np.arange(0, 2*m.pi, 0.01):
            
            Yc = 0
            Zc = 0.2
            a = 0.1
            b = 0.3
            c = 0.7
            d = 0
            e = 700
            phi = 30
            
            x = c
            y = Yc + a*m.cos(i)*m.cos(m.radians(phi)) - b*m.sin(i)*m.sin(m.radians(phi))
            z = Zc + a*m.cos(i)*m.sin(m.radians(phi)) + b*m.sin(i)*m.sin(m.radians(phi))
            
            wpose.position.x = x
            wpose.position.y = y            
            wpose.position.z = z
            waypoints.append(copy.deepcopy(wpose))
        return waypoints

    def cir_parametrized_path(self,wpose):

        R = rotation_matrix(m.pi,(1,0,0))
        quaternion = quaternion_from_matrix(R)
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]


        waypoints = []
        a = 0.5
        r = 0.2
        h = 0.1
        k = 0.6
        for i in np.arange(0, 2*m.pi, 0.009):
            z = a
            y = r*m.cos(i) + h
            x = r*m.sin(i) + k

            wpose.position.x = x
            wpose.position.y = y            
            wpose.position.z = z
      
            
            waypoints.append(copy.deepcopy(wpose))
        return waypoints

    def plan_cartesian_line(self, scale=1):
        group_1 = self.group_1

        waypoints = []

        wpose = group_1.get_current_pose().pose
        R = rotation_matrix(m.pi,(1,0,0))
        quaternion = quaternion_from_matrix(R)
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]

        wpose.position.x -= scale * 0.7  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group_1.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        return plan, fraction



    def plan_cartesian_path(self):
        group_1 = self.group_1
        wpose = group_1.get_current_pose().pose
        waypoints = self.cir_parametrized_path(wpose)
        print(waypoints)
        (plan,fraction) = group_1.compute_cartesian_path(waypoints,0.1,0.0)
        return plan,fraction

    def execute_path(self,plan):
        group = self.group_1
        group.execute(plan, wait=True)


def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()
            
        #jg = [0, 1.57, 0, 0, 0, 0]
        #tutorial.go_to_joint_state(jg)
        
        #cartesian_plan,fraction = tutorial.plan_cartesian_path()
        #tutorial.execute_path(cartesian_plan)
    
        raw_input()
        tutorial.go_to_pose_goal(0.3,0.7,0.0)
        raw_input()
        tutorial.go_to_pose_goal(0.3,-0.7,0.0)
        raw_input()
        #cartesian_plan,fraction = tutorial.plan_cartesian_line()
        #tutorial.execute_path(cartesian_plan)
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()