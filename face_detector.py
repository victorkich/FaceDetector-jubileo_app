#!/usr/bin/env python3

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import face_detection
import rospy
import cv2


class FaceEnable:
    def __init__(self):
        self.bridge = CvBridge()
        self.detected = []
        locations = Int16MultiArray()
        self.detector = face_detection.build_detector("DSFDDetector", confidence_threshold=.5, nms_iou_threshold=.3)
        sub_img = rospy.Subscriber('raw_image', Image, self.update_img)
        pub_eye = rospy.Publisher('Eyes', Int16MultiArray, queue_size=1)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.detections = None
            rate.sleep()
            locations.data = self.get_locations()
            print(locations.data)
            pub_eye.publish(locations)

    def update_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.UMat(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        rgb_image = cv2.UMat.get(img)
        self.detected = self.detector.detect(rgb_image)

    def get_locations(self):
        points = []
        for face in self.detected:
            x = (face[2] + face[3])/2
            y = (face[0] + face[1])/2
            points.append([x, y])
        return points


if __name__ == '__main__':
    rospy.init_node('faceEnable', anonymous=False, disable_signals=False)
    try:
        FaceEnable()
    except rospy.ROSInterruptException as e:
        print('Error on faceEnable...')
        print(e)
