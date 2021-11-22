#! /usr/bin/env python
import rospy
import cv2
import numpy as np
import cv2.aruco as aruco
import random
import tf
import sys

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

LAPLACIAN_ENHANCEMENT = np.array([-1,-1,-1,-1,9,-1,-1,-1,-1]).reshape(3,3)

calib_path = '~/submit_code/qmul/FYP/'

aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

min_x = -0.25
max_x = 0.25
min_y = 0.175
min_y = 0.3
max_y = 0.5

range_x = (min_x*1000.0, max_x*1000.0)
range_y = (min_y*1000.0, max_y*1000.0)

pose = Pose()
pose.position.x = -0.25
pose.position.y = 0.5
pose.position.z = 1.07
quaternion = tf.transformations.quaternion_from_euler(0,0,0)
pose.orientation.x = quaternion[0]
pose.orientation.y = quaternion[1]
pose.orientation.z = quaternion[2]
pose.orientation.w = quaternion[3]
class TestAccuracy:
    def __init__(self):
        rospy.init_node('test_accuracy', anonymous=True)

        self.bridge = CvBridge()

        # subscribers
        rospy.loginfo("waiting for image_raw...")
        rospy.wait_for_message('/camera/color/image_raw', Image)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        self.fetch = False
        self.n_marker = -1
        self.cube_names = [
            "pawnB1-201", "pawnB1-201_0", "pawnB1-201_1",
            "pawnB1-202", "pawnB1-202_0", "pawnB1-202_1",
            "pawnB1-203", "pawnB1-203_0", "pawnB1-203_1"
        ]



        self.state_msg = ModelState()
        self.state_msg.pose = pose
        rospy.loginfo("waiting for set_model_state...")
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.loginfo("reset the cube position...")
        for i in range(len(self.cube_names)):

            try:
                self.state_msg.model_name = self.cube_names[i]
                self.state_msg.pose.position.x = -1
                self.state_msg.pose.position.y = -1
                resp = self.set_state( self.state_msg )
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        rospy.sleep(1)



    def if_collision(self, arr, x, y):
        if arr == []:
            return False
        for t in arr:
            if abs(t[0]-x) < 0.05 and abs(t[1]-y) < 0.05:
                return True
        return False

    def random_positions(self, n):
        arr = []
        changes = [0.005, 0.01]
        for i in range(n):
            ran_x = random.randint(range_x[0], range_x[1])/1000.0
            ran_y = random.randint(range_y[0], range_y[1])/1000.0
            result = self.if_collision(arr, ran_x, ran_y)

            while result:
                if random.randint(0,1) == 0:
                    ran_x = random.randint(range_x[0], range_x[1])/1000.0
                else:
                    ran_y = random.randint(range_y[0], range_y[1])/1000.0
                result = self.if_collision(arr, ran_x, ran_y)
            arr.append((ran_x, ran_y))
        return arr
    def image_callback(self, msg):
        self.n_marker = -1
        if self.fetch:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            cv_image = cv2.GaussianBlur(cv_image, (5,5), 0)
            cv_image = cv2.filter2D(cv_image, -1, LAPLACIAN_ENHANCEMENT)
            cv_image = cv2.filter2D(cv_image, -1, LAPLACIAN_ENHANCEMENT)
            cv_image = cv2.adaptiveThreshold(
                cv_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,17,2)
            # _,cv_image = cv2.threshold(cv_image,150,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

            # cv_image = cv2.adaptiveThreshold(cv_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
            corners, ids, rejected = aruco.detectMarkers(
                                                image=cv_image,
                                                dictionary=aruco_dict,
                                                parameters=parameters,
                                                # cameraMatrix = camera_matrix,
                                                # distCoeff=camera_distortion
                                            )
            if ids is not None:
                if self.n_marker < len(ids):
                    self.n_marker = len(ids)
                aruco.drawDetectedMarkers(cv_image, corners)

            # else:
                # rospy.loginfo("No aruco marker found")

            cv2.imshow("image", cv_image)
            cv2.waitKey(3)

        return
    def test(self):
        file1 = open('/home/yuhang/qmul/jupyter_fyp/thre17_restrict_new.txt', 'w')
        L = []
        total =100.0
        for i in range(1,10):
            rospy.loginfo("[{}] cubes".format(i))
            correct = 0
            for each in range(int(total)):
                pos = self.random_positions(i)
                for j in range(len(pos)):
                    try:
                        self.state_msg.model_name = self.cube_names[j]
                        self.state_msg.pose.position.x = pos[j][0]
                        self.state_msg.pose.position.y = pos[j][1]
                        resp = self.set_state( self.state_msg )
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                self.fetch = True
                rospy.sleep(1)
                self.fetch = False
                if self.n_marker == i:
                    correct += 1
                sys.stdout.write('\r{}/100% ({} detected!)'.format(each+1, correct))
                sys.stdout.flush()
                self.n_marker = -1
            acc = str(correct)
            rospy.loginfo(acc + "/"+str(total))
            L.append(acc+"\n")
        file1.writelines(L)
        file1.close()
if __name__ == '__main__':
    try:
        test = TestAccuracy()
        test.test()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('-> Tracking node terminated.')
    cv2.destroyAllWindows()
