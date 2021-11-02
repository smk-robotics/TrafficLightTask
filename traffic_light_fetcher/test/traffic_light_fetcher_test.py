#! /usr/bin/env python 
import cv2
import rospy
import rospkg
import rostest
import os.path as osp
import unittest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

class TrafficLightFetcherTest(unittest.TestCase):
    @classmethod
    def _traffic_light_detected_callback(cls, traffic_light_detected_msg):
        cls.traffic_light_detected = traffic_light_detected_msg

    @classmethod
    def _traffic_light_size_callback(cls, traffic_light_size_msg):
        cls.last_detected_traffic_light = traffic_light_size_msg;

    @classmethod
    def setUpClass(cls):
        rospy.init_node("traffic_light_fetcher_test", anonymous=True)
        cls.traffic_light_detected = False
        cls.last_detected_traffic_light = None
        cls.image_pub = rospy.Publisher("~image_topic", Image, queue_size=1)
        cls.traffic_light_detected_sub = rospy.Subscriber(
            "~traffic_light_detected", Bool, cls._traffic_light_detected_callback)
        cls.traffic_light_size = rospy.Subscriber(
            "~traffic_light_size", Vector3, cls._traffic_light_size_callback)
        rospy.sleep(0.5)

    def test_first_(self):
        path_to_pkg = rospkg.RosPack().get_path("traffic_light_fetcher")
        path_to_img = osp.join(path_to_pkg, "testdata", "1.jpg")
        cv_img = cv2.imread(path_to_img)
        msg_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.image_pub.publish(msg_img)
        rospy.sleep(0.2)
        self.assertEqual(self.traffic_light_detected.data, True)
        self.assertEqual(self.last_detected_traffic_light.x, 11.0)
        self.assertEqual(self.last_detected_traffic_light.y, 33.0)
        self.assertEqual(self.last_detected_traffic_light.z, 0.0)

    def test_second_(self):
        path_to_pkg = rospkg.RosPack().get_path("traffic_light_fetcher")
        path_to_img = osp.join(path_to_pkg, "testdata", "2.jpg")
        cv_img = cv2.imread(path_to_img)
        msg_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.image_pub.publish(msg_img)
        rospy.sleep(0.2)
        self.assertEqual(self.traffic_light_detected.data, True)
        self.assertEqual(self.last_detected_traffic_light.x, 10.0)
        self.assertEqual(self.last_detected_traffic_light.y, 30.0)
        self.assertEqual(self.last_detected_traffic_light.z, 0.0)

    def test_third_(self):
        path_to_pkg = rospkg.RosPack().get_path("traffic_light_fetcher")
        path_to_img = osp.join(path_to_pkg, "testdata", "3.jpg")
        cv_img = cv2.imread(path_to_img)
        msg_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.image_pub.publish(msg_img)
        rospy.sleep(0.2)
        self.assertEqual(self.traffic_light_detected.data, True)
        self.assertEqual(self.last_detected_traffic_light.x, 9.0)
        self.assertEqual(self.last_detected_traffic_light.y, 27.0)
        self.assertEqual(self.last_detected_traffic_light.z, 0.0)


if __name__ == "__main__":
    rostest.rosrun(
        "traffic_light_fetcher",
        "traffic_light_fetcher_test",
        TrafficLightFetcherTest,
    )
