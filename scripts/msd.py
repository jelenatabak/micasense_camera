#!/usr/bin/env python3

from numpy.core.numeric import NaN
import rospy

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image as Im
from tf.transformations import quaternion_matrix
import tf2_ros
import ros_numpy

import xml.etree.ElementTree as ET
import struct
import ctypes
import numpy as np
from math import isnan, sqrt, pow
import cv2
from cv_bridge import CvBridge
from micasense.image import Image
import os
import queue


def float_to_rgb(float_rgb):
    """ Converts a packed float RGB format to an RGB list    
        Args:
            float_rgb: RGB value packed as a float
        Returns:
            color (list): 3-element list of integers [0-255,0-255,0-255]
    """
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    color = [r, g, b]

    return color


def rgb_to_float(color):
    """ Converts an RGB list to the packed float format used by PCL

        From the PCL docs:
        "Due to historical reasons (PCL was first developed as a ROS package),
         the RGB information is packed into an integer and casted to a float"

        Args:
            color (list): 3-element list of integers [0-255,0-255,0-255]

        Returns:
            float_rgb: RGB value packed as a float
    """
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb


class CameraModel:
    def __init__(self, path):
        self.path = path
        self.three = ET.parse(self.path)
        self.root = self.three.getroot()
        self.px = float(self.root[0][3][1].text)
        self.py = float(self.root[0][3][2].text)
        self.u0 = float(self.root[0][3][3].text)
        self.v0 = float(self.root[0][3][4].text)
        self.width = 1280
        self.height = 960

    def get_pixel_coordinates(self, point):
        """ Conversion from meters to pixels. 
            Args:
                x,y,z: 3d point in camera coordinate frame
            Returns:
                u,v: pixel coordinates
        """
        x = point[0].item()
        y = point[1].item()
        z = point[2].item()

        u = self.u0 + self.px * x/z
        v = self.v0 + self.py * y/z
        u_round = round(u)
        v_round = round(v)
        if u_round > self.width-1 or u_round < 0:
            u_round = None
        if v_round > self.height-1 or v_round < 0:
            v_round = None
        #dist = sqrt(pow((u-u_round), 2) + pow((v-v_round), 2))
        return u_round, v_round


class MSPointCloud:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_frame = 'panda_camera'
        self.ms_bands = 10
        self.pc_composite_pub = rospy.Publisher(
            "/composite_cloud", PointCloud2, queue_size=10)

        self.spectral_pub = []
        for i in range(10):
            self.spectral_pub.append(rospy.Publisher(
                "/pc_band_" + str(i+1), PointCloud2, queue_size=10))

        self.ms_camera = []
        for i in range(1, 6):
            xml_path = '/home/jelena/catkin_ws_old/src/micasense_camera/intrinsics/ms_red_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))
        for i in range(1, 6):
            xml_path = '/home/jelena/catkin_ws_old/src/micasense_camera/intrinsics/ms_blue_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))

        # self.clouds_path = '/home/jelena/catkin_ws_old/src/micasense_camera/pepper2_44/pointclouds/'
        # self.clouds = sorted(os.listdir(self.clouds_path))
        self.base_path = "/home/jelena/catkin_ws_old/src/micasense_camera/pepper4_44/"
        self.msd_path = self.base_path + "msd/"
        self.images_path = self.base_path + "images/"
        self.plant_path = self.base_path + "plant/"
        self.pepper_path = self.base_path + "pepper/"
        self.prefix = "pepper4_"

        self.pcs = queue.Queue()
        self.pc_num = 0
        rospy.Subscriber("/dataset_collection/pc_save", PointCloud2, self.cbck, queue_size=45)
        
    def cbck(self, msg):
        print("In callback!")
        self.pcs.put(msg)


    def generate_mspc(self):
        # self.pc = rospy.wait_for_message(
        #    '/camera/depth_registered/points', PointCloud2)
        # self.pc = rospy.wait_for_message(
        #     '/dataset_collection/pc_save', PointCloud2)
        while not rospy.is_shutdown():
            if self.pcs.empty():
                rospy.sleep(0.001)
            else:
                msg = self.pcs.get()
                self.pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(
                    msg)  # shape (480,640)
                self.array = np.copy(self.pc_array)

                # load pc 
                # pc_path = self.clouds_path + self.clouds[pc_num]
                # print(pc_path)
                # pcd = o3d.io.read_point_cloud(pc_path)
                # self.array = np.asarray(pcd.points) 

                self.pc_height = self.pc_array.shape[0]
                self.pc_width = self.pc_array.shape[1]
                self.ms_array = np.full(
                    [self.pc_height, self.pc_width, self.ms_bands], NaN)

                # image shape = (960, 1280)
                images = []
                img_num = self.pc_num
                if self.pc_num > 18:
                    img_num += 1

                for i in range(1, 6):
                    path = self.base_path + "SYNC0000SET_red/000/IMG_" + str(img_num).zfill(4) + "_" + \
                        str(i) + '.tif'
                    #images.append(Image(path).undistorted_reflectance())
                    images.append(cv2.imread(path, cv2.IMREAD_GRAYSCALE))
                    # cv2.imshow("test", images[i-1])
                    # cv2.waitKey(0)

                for i in range(6,11):
                    path = self.base_path + "SYNC0000SET_blue/000/IMG_" + str(img_num).zfill(4) + "_" + \
                        str(i) + '.tif'
                    #images.append(Image(path).undistorted_reflectance())
                    images.append(cv2.imread(path, cv2.IMREAD_GRAYSCALE))


                tfs = []
                for i in range(1, 6):
                    ms_frame = 'ms_red_band' + str(i)
                    try:
                        trans = self.tf_buffer.lookup_transform(
                            ms_frame, self.camera_frame, rospy.Time())
                        tfs.append(trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        print("Did not get transform")
                        return

                for i in range(1, 6):
                    ms_frame = 'ms_blue_band' + str(i)
                    try:
                        trans = self.tf_buffer.lookup_transform(
                            ms_frame, self.camera_frame, rospy.Time())
                        tfs.append(trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        print("Did not get transform")
                        return

                T = []
                for i in range(self.ms_bands):
                    trans = tfs[i].transform.translation
                    rot = tfs[i].transform.rotation
                    Tx = np.eye(4, dtype="float32")
                    Tx = np.asmatrix(quaternion_matrix([rot.x, rot.y, rot.z, rot.w]))
                    Tx[0:3, 3] = np.resize([trans.x, trans.y, trans.z], (3, 1))
                    T.append(Tx)

                # iterate pc and extract multispectral information for each point
                for i in range(self.pc_height):
                    for j in range(self.pc_width):
                        pt = self.array[i][j]

                        if not isnan(pt[0]):
                            point = np.matrix([[pt[0]], [pt[1]], [pt[2]], [1]])

                            for n in range(self.ms_bands):
                                point_transformed = np.dot(T[n], point)

                                u, v = self.ms_camera[n].get_pixel_coordinates(
                                    point_transformed)
                                if u is not None and v is not None:
                                    self.ms_array[i, j, n] = images[n][v, u]

                with open(self.msd_path + self.prefix + 'msd_' + str(self.pc_num) + '.npy', 'wb') as f:
                    np.save(f, self.ms_array)

                print("Saved msd number " + str(self.pc_num))
                self.pc_num += 1


    def save_masks(self):
        pc_pub = rospy.Publisher("/test_thresh", PointCloud2, queue_size=10)
        orig_pc_pub = rospy.Publisher("/orig_pc", PointCloud2, queue_size=10)

        # with open("/home/jelena/catkin_ws_old/src/micasense_camera/pepper1/msd.npy", 'rb') as f:
        #     ms_array = np.load(f)
        #print(ms_array.shape)

        # pc = rospy.wait_for_message('/dataset_collection/pc_save', PointCloud2)
        while not rospy.is_shutdown():
            if self.pcs.empty():
                rospy.sleep(0.001)
            else:
                ms_path = self.msd_path + self.prefix + "msd_" + str(self.pc_num) + ".npy"
                print(ms_path)
                with open(ms_path, 'rb') as f:
                    ms_array = np.load(f)
                    self.ms_array = ms_array

                msg = self.pcs.get()
                pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                pc_height = pc_array.shape[0]
                pc_width = pc_array.shape[1]

                self.pc_array = pc_array
                self.pc_height = pc_height
                self.pc_width = pc_width
                array = np.copy(pc_array)

                z_thresh = 1
                ir_thresh = 130
                re1_thresh = 100
                re3_thresh = 100
                
                red_thresh = 70
                green_thresh = 60

                total_thresh = 2
                plant = 5
                pepper = 10

                for i in range(pc_height):
                    for j in range(pc_width):
                        if pc_array[i, j][2] > z_thresh:
                            array[i, j][3] = 0
                        else:
                            array[i, j][3] = 1

                for i in range(pc_height):
                    for j in range(pc_width):
                        if array[i, j][3]:
                            if ms_array[i, j, 3] > ir_thresh:
                                array[i,j][3] += 1
                        
                            if ms_array[i, j, 4] > re1_thresh:
                                array[i,j][3] += 1

                            if ms_array[i, j, 9] > re3_thresh:
                                array[i,j][3] += 1


                            if array[i, j][3] > total_thresh:
                                array[i,j][3] = plant

                                if (ms_array[i,j,2] > red_thresh or ms_array[i,j,7] > red_thresh) and ms_array[i,j,1] < green_thresh:
                                    array[i,j][3] = pepper

                mask_plant = np.zeros((pc_height, pc_width))
                mask_pepper = np.zeros((pc_height, pc_width))

                for i in range(pc_height):
                    for j in range(pc_width):
                        if array[i,j][3] >= pepper:
                            mask_pepper[i,j] = 255
                        elif array[i,j][3] >= plant:
                            mask_plant[i,j] = 255

                kernel = np.ones((4, 4), np.uint8)
                # mask_plant = cv2.erode(mask_plant, kernel)
                mask_pepper = cv2.erode(mask_pepper, kernel)
                kernel = np.ones((2, 2), np.uint8)
                mask_plant = cv2.dilate(mask_plant, kernel)
                # mask_plant = cv2.erode(mask_plant, kernel)


                # save both masks as png
                plant_path = self.plant_path + self.prefix + str(self.pc_num) + ".png"
                pepper_path = self.pepper_path + self.prefix + str(self.pc_num) + ".png"
                #img_path = self.base_path + "img.jpg"

                cv2.imwrite(plant_path, mask_plant)
                cv2.imwrite(pepper_path, mask_pepper)
                # cv2.imshow("test", mask_pepper)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                # self.publish_ms_cloud(4)


                # img_msg = rospy.wait_for_message('/dataset_collection/image_save', Im)
                # bridge = CvBridge()
                # cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
                # cv2.imwrite(img_path, cv_image)

                print("Masks number " + str(self.pc_num) + " saved!")
                self.pc_num += 1


                # pc_transformed = ros_numpy.point_cloud2.array_to_pointcloud2(
                #     array, frame_id=self.camera_frame)
                # pc_transformed.fields[3].name = 'intensity'

                # # while not rospy.is_shutdown():
                # pc_pub.publish(pc_transformed)
                # orig_pc_pub.publish(msg)
                # print("PC published!")
                # rospy.sleep(30)

    def publish_ms_cloud(self, band_index):
        index = band_index - 1
        if self.pc_array is not None:
            array = np.copy(self.pc_array)
            for i in range(self.pc_height):
                for j in range(self.pc_width):
                    if not isnan(self.ms_array[i, j, index]):
                        array[i, j][3] = self.ms_array[i, j, index]
                    else:
                        array[i, j][3] = NaN

            pc_transformed = ros_numpy.point_cloud2.array_to_pointcloud2(
                array, frame_id=self.camera_frame)
            pc_transformed.fields[3].name = 'band' + str(band_index)
            self.spectral_pub[index].publish(pc_transformed)
        else:
            rospy.loginfo("Missing original pointcloud!")

    def publish_false_composite(self, composite):
        if self.pc_array is not None:
            array = np.copy(self.pc_array)
            for i in range(self.pc_height):
                for j in range(self.pc_width):
                    fc = []
                    visible = True
                    for k in composite:
                        ind = k - 1
                        if not isnan(self.ms_array[i, j, ind]):
                            fc.append(self.ms_array[i, j, ind])
                        else:
                            visible = False
                    if visible:
                        array[i, j][3] = rgb_to_float(fc)
                    else:
                        array[i, j][3] = NaN

            pc_transformed = ros_numpy.point_cloud2.array_to_pointcloud2(
                array, frame_id=self.camera_frame)
            self.pc_composite_pub.publish(pc_transformed)
        else:
            rospy.loginfo("Missing original pointcloud!")

    def publish_vegetation_index(self, index):
        if self.pc_array is not None:
            pub = rospy.Publisher(index, PointCloud2, queue_size=10)

            # (nir - red) / (nir + red)
            if index == 'ndvi':
                array = np.copy(self.pc_array)
                for i in range(self.pc_height):
                    for j in range(self.pc_width):
                        nir = self.ms_array[i, j, 3]
                        red = self.ms_array[i, j, 2]

                        if not isnan(nir) and not isnan(red):
                            array[i, j][3] = (nir-red)/(nir+red)
                        else:
                            array[i, j][3] = NaN

                pc_transformed = ros_numpy.point_cloud2.array_to_pointcloud2(
                    array, frame_id=self.camera_frame)
                pc_transformed.fields[3].name = index
                pub.publish(pc_transformed)

            # (nir - rededge) / (nir + rededge)
            elif index == 'ndre':
                array = np.copy(self.pc_array)
                for i in range(self.pc_height):
                    for j in range(self.pc_width):
                        nir = self.ms_array[i, j, 3]
                        rededge = self.ms_array[i, j, 4]

                        if not isnan(nir) and not isnan(rededge):
                            array[i, j][3] = (nir-rededge)/(nir+rededge)
                        else:
                            array[i, j][3] = NaN

                pc_transformed = ros_numpy.point_cloud2.array_to_pointcloud2(
                    array, frame_id=self.camera_frame)
                pc_transformed.fields[3].name = index
                pub.publish(pc_transformed)

            else:
                print("Unknown index: " + index)
        else:
            rospy.loginfo("Missing original pointcloud!")


def main():
    rospy.init_node('msd', anonymous=True)
    camera = MSPointCloud()
    camera.save_masks()
    # for i in range(1):
    #     camera.generate_mspc(i)
    #     print("MS cloud generated!")

    # while not rospy.is_shutdown():
    #     camera.publish_ms_cloud(4)
    #     #camera.publish_false_composite([4, 4, 4])
    #     #camera.publish_vegetation_index('ndvi')
    #     print("Cloud published!")


def test():
    rospy.init_node('msd', anonymous=True)
    camera = MSPointCloud()
    camera.test_thresh()
    
if __name__ == "__main__":
    main()
