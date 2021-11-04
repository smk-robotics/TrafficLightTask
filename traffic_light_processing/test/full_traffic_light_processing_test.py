#! /usr/bin/env python
import cv2
import rostest
import rospkg
import rospy
import os.path as osp
import unittest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class TrafficLightProcessingTest(unittest.TestCase):
    @classmethod
    def __zone_height_callback(cls, zone_height_msg):
        cls.zone_height = zone_height_msg
            
    @classmethod
    def setUpClass(cls):
        rospy.init_node("traffic_light_processing_test", anonymous="True")
        cls.zone_height = None
        cls.zone_height_sub = rospy.Subscriber(
            "~zone_height", Float32, cls.__zone_height_callback)
        cls.image_pub = rospy.Publisher("~image_topic", Image, queue_size=1)
        rospy.sleep(0.3)
    
    def test_first(self):
        path_to_pkg = rospkg.RosPack().get_path("traffic_light_fetcher")
        path_to_img = osp.join(path_to_pkg, "testdata", "1.jpg")
        cv_img = cv2.imread(path_to_img)
        msg_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.image_pub.publish(msg_img)
        rospy.sleep(0.2)
        self.assertEqual(self.zone_height.data, 11.0)

    def test_second(self):
        path_to_pkg = rospkg.RosPack().get_path("traffic_light_fetcher")
        path_to_img = osp.join(path_to_pkg, "testdata", "2.jpg")
        cv_img = cv2.imread(path_to_img)
        msg_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.image_pub.publish(msg_img)
        rospy.sleep(0.2)
        self.assertEqual(self.zone_height.data, 10.0)


    def test_third(self):
        path_to_pkg = rospkg.RosPack().get_path("traffic_light_fetcher")
        path_to_img = osp.join(path_to_pkg, "testdata", "3.jpg")
        cv_img = cv2.imread(path_to_img)
        msg_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.image_pub.publish(msg_img)
        rospy.sleep(0.2)
        self.assertEqual(self.zone_height.data, 9.0)


if __name__ == "__main__":
    rostest.rosrun(
        "traffic_light_processing",
        "traffic_light_processing_test",
        TrafficLightProcessingTest
    )