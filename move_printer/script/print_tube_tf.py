#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from pylab import frange
from math import pi, cos, sin
from std_msgs.msg import String, Bool
from move_printer.srv import TubeDim
import geometry_msgs.msg



class Printer:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('print_tube_moveit',anonymous=True)
        self.group = moveit_commander.MoveGroupCommander("printer_group")
        rospy.Service('print_tube', TubeDim, self.print_tube)
        self.plate_z=0.035
        self.pub_extruder = rospy.Publisher('/extrude', Bool, queue_size=10)
        rospy.spin()

    def deg_to_rad(self,deg):
        return deg*pi/180


    def plan_tube(self, diameter, height):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        radius = diameter/2
        heights = list(frange(self.plate_z,height,0.01))
        for h in heights:
            for angle in range(0,360):
                wpose.position.z = h
                wpose.position.x = radius * cos(self.deg_to_rad(angle))
                wpose.position.y = radius * sin(self.deg_to_rad(angle))
                waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        self.group.plan()
        rospy.sleep(3)
        self.group.execute(plan,wait=True)
        return plan, fraction

    def plan_cercle(self, radius): #deprecated
        waypoints = []
        wpose = self.group.get_current_pose().pose
        for angle in range(0,360):
            wpose.position.z = 0.2
            wpose.position.x = radius * cos(self.deg_to_rad(angle))
            wpose.position.y = radius * sin(self.deg_to_rad(angle))
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        self.group.plan()
        rospy.sleep(1)
        self.group.execute(plan,wait=True)
        self.group.set_named_target('init_pose')
        self.group.plan()
        return plan, fraction

    def print_tube(self,req):
        extrusion_enabled= Bool()
        pose_target = geometry_msgs.msg.Pose()

        #Go to first point of extrusion
        pose_target.orientation.w = 1.0
        pose_target.position.x = req.diameter/2
        pose_target.position.y = 0
        pose_target.position.z = self.plate_z
        print pose_target
        self.group.set_joint_value_target(pose_target)
        self.group.execute(self.group.plan())

        #Enable extrusion
        extrusion_enabled.data=True
        self.pub_extruder.publish(extrusion_enabled)

        #Print the 3D piece
        self.plan_tube(req.diameter,req.height)

        #Disable extrusion
        extrusion_enabled.data=False
        self.pub_extruder.publish(extrusion_enabled)

        #Go back to init Pose
        self.group.set_named_target('init_pose')
        self.group.execute(self.group.plan())
        return True




def main():
    p = Printer()

if __name__ == "__main__":
    main()