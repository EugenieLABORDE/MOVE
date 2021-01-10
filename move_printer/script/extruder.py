#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import Bool, ColorRGBA

from visualization_msgs.msg import Marker


class Extruder():

  def __init__(self):
    self.pub_marker = rospy.Publisher('printing_marker', Marker, queue_size=10)
    self.counter = 0
    self.extrusion_enabled = False
    self.c = ColorRGBA()
    rospy.Subscriber('/extrude', Bool, self.extrusion_activation)

    self.plasticParam = rospy.get_param("/extruder/plastic")
    print self.plasticParam['color']['r']



  def extrusion_activation(self, msg):
    self.extrusion_enabled = msg.data


  def print_marker(self):
    marker = Marker()
    marker.header.frame_id = "extruder_link";
    marker.header.stamp = rospy.Time.now();
    marker.ns ="PLA_pixel";
    marker.id = self.counter
    self.counter += 1

    marker.color.a = self.plasticParam['color']['a']
    marker.color.r = self.plasticParam['color']['r']
    marker.color.g = self.plasticParam['color']['g']
    marker.color.b = self.plasticParam['color']['b']
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.type = marker.CUBE
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    self.pub_marker.publish(marker)


if __name__=='__main__':
  try:

    rospy.init_node('Extruder')
    ex = Extruder()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        if ex.extrusion_enabled :
            ex.print_marker()
        rate.sleep()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
