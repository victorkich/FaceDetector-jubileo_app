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
        self.detector = face_detection.build_detector("DSFDDetector", confidence_threshold=.5, nms_iou_threshold=.3)
        self.sub_img = rospy.Subscriber('raw_image', Image, self.update_img)
        pub_eye = rospy.Publisher('Eyes', Int16MultiArray, queue_size=1)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.detections = None
            rate.sleep()
            print(self.detections)
            pub_eye.publish()

    def update_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.UMat(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        rgb_image = cv2.UMat.get(img)
        self.detections = self.detector.detect(rgb_image)


if __name__ == '__main__':
    rospy.init_node('faceEnable', anonymous=False, disable_signals=False)
    try:
        FaceEnable()
    except rospy.ROSInterruptException as e:
        print('Error on faceEnable...')
        print(e)
