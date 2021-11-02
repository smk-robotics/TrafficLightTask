#! /usr/bin/env python 
import rospy
import rostest
import unittest
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

class TrafficLightAnalysisTest(unittest.TestCase):
    @classmethod
    def _zone_height_callback(cls, zone_height_msg):
        cls.zone_height = zone_height_msg

    @classmethod
    def setUpClass(cls):
        rospy.init_node("traffic_light_analysis_test", anonymous=True)
        cls.zone_height = None
        cls.traffic_light_detected_signal_pub = rospy.Publisher(
            "~traffic_light_detected", Bool, queue_size=1)
        cls.traffic_light_size_pub = rospy.Publisher(
            "~traffic_light_size", Vector3, queue_size=1)
        cls.zone_height_sub = rospy.Subscriber(
            "~zone_height", Float32, cls._zone_height_callback)
        rospy.sleep(0.3)

    def test_first_(self):
        traffic_light_size = Vector3()
        traffic_light_size.x = 10.0
        traffic_light_size.y = 30.0
        traffic_light_size.z = 0.0
        traffic_light_detected = Bool()
        traffic_light_detected.data = True
        self.traffic_light_size_pub.publish(traffic_light_size)
        self.traffic_light_detected_signal_pub.publish(traffic_light_detected)
        rospy.sleep(0.1)
        self.assertEqual(self.zone_height.data, 10.0)

    def test_second_(self):
        traffic_light_size = Vector3()
        traffic_light_size.x = 11.0
        traffic_light_size.y = 33.0
        traffic_light_size.z = 0.0
        traffic_light_detected = Bool()
        traffic_light_detected.data = True
        self.traffic_light_size_pub.publish(traffic_light_size)
        self.traffic_light_detected_signal_pub.publish(traffic_light_detected)
        rospy.sleep(0.1)
        self.assertEqual(self.zone_height.data, 11.0)

    def test_third_(self):
        traffic_light_size = Vector3()
        traffic_light_size.x = 9.0
        traffic_light_size.y = 27.0
        traffic_light_size.z = 0.0
        traffic_light_detected = Bool()
        traffic_light_detected.data = True
        self.traffic_light_size_pub.publish(traffic_light_size)
        self.traffic_light_detected_signal_pub.publish(traffic_light_detected)
        rospy.sleep(0.1)
        self.assertEqual(self.zone_height.data, 9.0)

if __name__ == "__main__":
    rostest.rosrun(
        "traffic_light_analysis",
        "traffic_light_analysis_test",
        TrafficLightAnalysisTest,
    )
