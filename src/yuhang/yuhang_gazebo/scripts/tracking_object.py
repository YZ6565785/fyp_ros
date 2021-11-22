#! /usr/bin/env python
import roslib
roslib.load_manifest('yuhang_gazebo')

import sys
import rospy
import cv2
import tf
import math
import struct
import cv2.aruco as aruco
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Point

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from std_msgs.msg import Int32MultiArray

LAPLACIAN_ENHANCEMENT = np.array([-1,-1,-1,-1,9,-1,-1,-1,-1]).reshape(3,3)

aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()
id_to_find  = 0
marker_size  = 5
calib_path = '~/submit_code/qmul/FYP/'
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')
font = cv2.FONT_HERSHEY_PLAIN

CUBE_SIZE = 0.047
platform_height = 0.9947739999999999

camera_x = 0
camera_y = 0.161000
camera_z = 1.656500

camera_angle_r = 0
camera_angle_p = 1.186820
camera_angle_y = 0



#==============================
###### helper functions
#==============================
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def get_pixel(point1, point2):
    x = point1[0] + (point2[0] - point1[0])/2.0
    y = point1[1] + (point2[1] - point1[1])/2.0
    return (int(x),int(y))





class Tracking:
    def __init__(self):
        self.cube_radius =0.03 / 2.0
        rospy.init_node('tracking_target', anonymous=True)

        self.pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.pub_pose = PoseStamped()
        self.pub_pose.header.seq = 0

        self.rate = rospy.Rate(1)



        self.target_found = False
        self.last_d = 0

        self.r = 0
        self.p = 0
        self.y = 0

        self.bridge = CvBridge()


        self.tracking_id = None
        self.queue = []
        self.subareas_status = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.sub = [-1,-1,-1]
        self.detection = []


        # subscribers
        rospy.wait_for_message('/camera/color/image_raw', Image)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback_aruco, queue_size=1)

        rospy.wait_for_message('/camera/depth/points', PointCloud2)
        self.image_points = rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback_pointcloud)

        # pulishers:
        self.position_pub = rospy.Publisher('/tracking_object/target_pose', Point, queue_size = 1)
        self.sub_target_id_pub = rospy.Publisher('/tracking_object/sub_target_id', Int32MultiArray, queue_size = 1)
        self.sub_status_pub = rospy.Publisher('/tracking_object/sub_status', Int32MultiArray, queue_size = 1)
        self.cube_201_poses_pub = rospy.Publisher('/tracking_object/cube_201_poses', PoseArray, queue_size = 1)
        self.cube_202_poses_pub = rospy.Publisher('/tracking_object/cube_202_poses', PoseArray, queue_size = 1)
        self.cube_203_poses_pub = rospy.Publisher('/tracking_object/cube_203_poses', PoseArray, queue_size = 1)
        self.image_pub = rospy.Publisher("/tracking_object/image_topic_output",Image,queue_size=1)
        self.available_sub_pub = rospy.Publisher("/tracking_object/available_sub",PoseArray,queue_size=1)




        self.available_sub = [None, None, None] # available subareas
        self.cube_201_poses = [None]*3
        self.cube_202_poses = [None]*3
        self.cube_203_poses = [None]*3
        self.cubes_poses = [[0,0,0] for i in range(9)]
        self.cube1 = [0,0]
        self.cube2 = [0,0]
        self.cube3 = [0,0]

        self.target_id = None
        self.target_pose = None
        self.target_u = None
        self.target_v = None
        self.target_d = 0


        self.cloud_data = PointCloud2()
        self.rate.sleep()


    def checkChange(self,coor, id):
        pass
    def is_cube(self, marker_id):
        if marker_id == 201 or marker_id == 202 or marker_id == 203:
            return True
        return False
    def inReadyArea(self, x, y):
        if x < 0 and y > 0.3:
            return True
    def update_subareas(self, id, coor):
        for sub in self.subareas_status:
            b = [0.25, 0.17, 0.09]
            for j in range(len(b)):
                if abs(coor[0] - b[j]) < 0.02:
                    self.subareas_status[id][j] = 0

        return

    # take a coordinate (x,y,z) input, and return 1, 2, or 3 (each represents a subarea)
    def check_subareas(self,coor):
        a = [0.32, 0.39, 0.46] # y position
        for i in range(len(a)):
            if abs(coor[1] - a[i]) < 0.02:
                b = [0.25, 0.17, 0.09] # x position
                for j in range(len(b)):
                    if abs(coor[0] - b[j]) < 0.02:
                        self.subareas_status[i][j] = 1
                        return i
        return -1


    def image_callback_aruco(self, msg):
        R_flip  = np.zeros((3,3), dtype=np.float32)
        R_flip[0,0] =-1.0
        R_flip[1,1] =-1.0
        R_flip[2,2] =-1.0
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.GaussianBlur(cv_image, (5,5), 0)
        cv_image = cv2.filter2D(cv_image, -1, LAPLACIAN_ENHANCEMENT)
        cv_image = cv2.filter2D(cv_image, -1, LAPLACIAN_ENHANCEMENT)
        corners, ids, rejected = aruco.detectMarkers(image=cv_image, dictionary=aruco_dict, parameters=parameters, cameraMatrix = camera_matrix, distCoeff=camera_distortion)

        self.detection = [corners, ids]

        haveTarget = False

        u,v = None, None
        self.sub = [-1,-1,-1]
        self.available_sub = PoseArray()

        self.cube_201_poses = PoseArray()
        self.cube_202_poses = PoseArray()
        self.cube_203_poses = PoseArray()
        if ids is not None:

            for i in range(len(ids)):
                marker_id = ids[i][0]

                # collect the subarea positions
                item = Pose()
                p = Point()
                item.position = p

                # calculate the pixel point of each marker detected
                # top left point
                point1 = ( int(corners[i][0][0][0]), int(corners[i][0][0][1]) )
                # bottom right point
                point2 = ( int(corners[i][0][2][0]), int(corners[i][0][2][1]) )
                point = get_pixel(point1,point2)

                if(self.is_cube(marker_id)):
                    x, y, z = self.calculate_coor(
                        self.pixelTo3DPoint(self.cloud_data, point[0], point[1]),
                        CUBE_SIZE)

                    item.position.x, item.position.y, item.position.z = x,y,z

                    #point = get_pixel(corners[0][0])
                    posit = self.check_subareas((x,y,z)) # return 1, 2, or 3, which represents the id of one subarea

                    if posit >=0:
                        self.sub[posit] = marker_id

                    if self.inReadyArea(x, y):
                        if marker_id == 201:
                            self.cube_201_poses.poses.append(item)
                        elif marker_id == 202:
                            self.cube_202_poses.poses.append(item)
                        else:
                            self.cube_203_poses.poses.append(item)

                else: # looping the subareas

                    x, y, z = self.calculate_coor(
                        self.pixelTo3DPoint(self.cloud_data, point[0], point[1]),
                        0)
                    print("=========>[{}]: {},{},{}".format(marker_id,x,y,z))

                    item.position.x, item.position.y, item.position.z = x,y,z

                    self.available_sub.poses.append(item)

                    self.update_subareas(marker_id,(x,y,z))


            self.available_sub_pub.publish(self.available_sub)

            self.cube_201_poses_pub.publish(self.cube_201_poses)
            self.cube_202_poses_pub.publish(self.cube_202_poses)
            self.cube_203_poses_pub.publish(self.cube_203_poses)

            print(self.sub)
            for i in range(len(ids)):
                marker_id = ids[i][0]

                # calculate the pixel point of each marker detected
                # top left point
                point1 = ( int(corners[i][0][0][0]), int(corners[i][0][0][1]) )
                # bottom right point
                point2 = ( int(corners[i][0][2][0]), int(corners[i][0][2][1]) )

                point = get_pixel(point1,point2)
                # print("#[{}]top left and bottom right pixels: {}".format(marker_id, (point1, point2), ))
                if self.is_cube(marker_id) :
                    x, y, z = self.calculate_coor(
                        self.pixelTo3DPoint(self.cloud_data, point[0], point[1]),
                        CUBE_SIZE)
                    if self.inReadyArea(x, y):

                        #print("GREAT! cube {} is inside of area".format(marker_id))
                        for i in range(len(self.sub)):
                            if marker_id == self.sub[i]:
                                haveTarget = True
                                self.target_id = marker_id
                                self.target_pose = (x,y,z)
                                print('###################')
                                print("###################>>>target id: {} is at subarea[{}].".format(marker_id,i))
                                print('###################')
                            #else:
                                #print("cube {} is not inside of subarea[{}], subarea has id: {}".format(marker_id, i, self.sub[i]))
                        #else:
                        #print("cube {} is not inside of area, {},{},{}".format(marker_id,x,y,z))
                        # print("#[{}]center pixel: {}".format(marker_id, (point[0], point[1]), ))
                        # print("#########cube {}: x:{}, y:{}".format(marker_id, point[0], point[1]))
                        print("cube markers [{}] found!, (x:{},\ty:{},\tz:{})".format(marker_id,
                                                                     x,y,z))
            msg = Point()
            print(haveTarget)
            if haveTarget:
                msg.x, msg.y, msg.z = self.target_pose[0], self.target_pose[1], self.target_pose[2]
                if msg.x < 0:
                    self.position_pub.publish(msg)
                    print("############################################################################################################")
                    print("# target coordinates to the world: x: {}, y:{}, z:{}".format(msg.x, msg.y, msg.z))
                    print("############################################################################################################")



            aruco.drawDetectedMarkers(cv_image, corners)
        else:
            print("Not found.")

        print(self.subareas_status)
        cv2.imshow("Kinect Camera", cv_image)
        cv2.waitKey(3)

        sub_status_msg = Int32MultiArray()
        sub_status_msg.data = self.subareas_status.flatten()
        self.sub_status_pub.publish(sub_status_msg)

        sub_target_id_msg = Int32MultiArray()
        sub_target_id_msg.data = self.sub
        self.sub_target_id_pub.publish(sub_target_id_msg)
        print("sub target id: ({})".format(sub_target_id_msg))
        return

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)

        w = image.shape[1]
        h = image.shape[0]
        (rows,cols,channels) = image.shape
        if cols > 60 and rows > 60 :
            color = (0,0,255)
            cv2.circle(image, (w/2,h/2), 10, color, 1)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_color = [86, 72, 11]
        upper_color = [193, 172, 108]
        lower_pink = numpy.array(lower_color[::-1], numpy.uint8)
        upper_pink = numpy.array(upper_color[::-1], numpy.uint8)
        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        # dilate and erode with kernel size 11x11
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, numpy.ones((3,3)))

        # find all of the contours in the mask image
        _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contourLength  = len(contours)

        if self.contourLength < 1:
            print("No target found.")
        else:
            #print("length of contours: ", self.contourLength)
            #print("Target found.")

            area = [0.0]*len(contours)
            for i in range(self.contourLength): ## Loop through all of the contours, and get their areas
                area[i] = cv2.contourArea(contours[i])

            #### Target #### the largest "pink" object
            #print("max of areas: ", max(area))
            target_image = contours[area.index(max(area))]

            # Using moments find the center of the object and draw a red outline around the object
            target_m = cv2.moments(target_image)
            self.target_u = int(target_m['m10']/target_m['m00'])
            self.target_v = int(target_m['m01']/target_m['m00'])
            points = cv2.minAreaRect(target_image)
            box = cv2.boxPoints(points)
            box = numpy.int0(box)
            cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
            #rospy.loginfo("Center of target is x at %d and y at %d", int(self.target_u), int(self.target_v))

            self.target_found = True

        cv2.imshow("Target", image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def callback_pointcloud(self,data):
        self.cloud_data = data
        '''

        for i in range(len(self.detection[0])):
            if self.detection[1][i] == 0:
                point = get_pixel(self.detection[0][0][0])
                x, y, z = self.calculate_coor(
                    self.pixelTo3DPoint(data, point[0], point[1]),
                    CUBE_SIZE)
                self.check_subareas((x,y,z))
        msg = Int32MultiArray()
        msg.data = self.subareas_status[0]
        self.sub1_pub.publish(msg)
        '''
        '''
        if data != None:

            coor = self.pixelTo3DPoint(data, self.target_u, self.target_v)

            #print (" x : %.4f  y: %.4f  z: %.4f" %(coor[0], coor[1], coor[2]))
            print(self.target_u, self.target_v)
            msg = Point()
            msg.x, msg.y, msg.z = self.calculate_coor(
                (coor[0],coor[1],coor[2]),
                CUBE_SIZE)
            if msg.x < 0:
                self.position_pub.publish(msg)
        '''

    def calculate_coor(self, coor, object_size):
        position = [0]*3
        x = coor[0]
        y = coor[1]
        z = coor[2]

        if y > 0: # below the center
            hyp = y / math.sin(camera_angle_p)
            adj_whole = (camera_z - platform_height - object_size) / math.tan(camera_angle_p)

            target_to_camera_world_z = adj_whole - hyp

            position[1] = target_to_camera_world_z + camera_y

            answer = round(target_to_camera_world_z,3)
            #print("distance to world (0,0) is : distance="+str(answer))


        else:
            hyp = -y / math.sin(camera_angle_p)
            adj_whole = (camera_z - platform_height - object_size) / math.tan(camera_angle_p)

            target_to_camera_world_z = adj_whole + hyp

            position[1] = target_to_camera_world_z + camera_y

            answer = round(target_to_camera_world_z,3)
            #print("distance to world (0,0) is : distance="+str(answer))

        position[2] = 1.009

        return coor[0], position[1], position[2]

    def pixelTo3DPoint(self, cloud, u, v):
        assert isinstance(cloud, PointCloud2)
        width = cloud.width
        height = cloud.height
        point_step = cloud.point_step
        row_step = cloud.row_step

        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        X = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        Y = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        Z = struct.unpack('f', byte_format)[0]

        return [X, Y, Z]


if __name__ == '__main__':
    try:
        tracker = Tracking()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('-> Tracking node terminated.')

    cv2.destroyAllWindows()
