#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('unibas_viewer')
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class unibas_viewer_depth:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw",Image)
    self.depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw",Image)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.callback)

  def callback(self, rgb_data, depth_data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
      depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
      depth_array = np.array(depth_image, dtype=np.float32)
      cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
    except CvBridgeError as e:
      print(e)

    cv2.imshow("image_raw", cv_image)
    cv2.imshow("depth", depth_array)
    cv2.waitKey(500)


def main(args):
  uvd = unibas_viewer_depth()
  rospy.init_node('unibas_viewer_depth', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

