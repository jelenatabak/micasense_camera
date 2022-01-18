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
import dynamic_reconfigure.client
import requests


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


class DataCollector:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_frame = 'panda_camera'
        self.ms_bands = 10

        self.ms_camera = []
        for i in range(1, 6):
            xml_path = '/home/jelena/catkin_ws_old/src/micasense_camera/intrinsics/ms_red_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))
        for i in range(1, 6):
            xml_path = '/home/jelena/catkin_ws_old/src/micasense_camera/intrinsics/ms_blue_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))

        self.tfs = []
        for i in range(1, 6):
            ms_frame = 'ms_red_band' + str(i)
            try:
                trans = self.tf_buffer.lookup_transform(
                    ms_frame, self.camera_frame, rospy.Time())
                self.tfs.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Did not get transform")
                return

        for i in range(1, 6):
            ms_frame = 'ms_blue_band' + str(i)
            try:
                trans = self.tf_buffer.lookup_transform(
                    ms_frame, self.camera_frame, rospy.Time())
                self.tfs.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Did not get transform")
                return

        self.T = []
        for i in range(self.ms_bands):
            trans = self.tfs[i].transform.translation
            rot = self.tfs[i].transform.rotation
            Tx = np.eye(4, dtype="float32")
            Tx = np.asmatrix(quaternion_matrix([rot.x, rot.y, rot.z, rot.w]))
            Tx[0:3, 3] = np.resize([trans.x, trans.y, trans.z], (3, 1))
            self.T.append(Tx)

        self.addr = '192.168.10.254'
        self.stereo_client = dynamic_reconfigure.client.Client(
            '/camera/stereo_module')

    def collect(self):
        while not rospy.is_shutdown():
            # sleep n hours
            self.collect_data()
            # turn chamber, turn camera..

    def collect_data(self):
        self.pc_msg = rospy.wait_for_message('/pc_topic', PointCloud2)
        self.rs_img = rospy.wait_for_message('/rgb_topic', Image)
        self.capture()
        # get imgs http
        self.images = []
        self.generate_mspc()
        # divide into cells? 
        self.calculate_ndvi()
        self.store_data()

    def store_data(self):
        pass

    def calculate_ndvi(self):
        array = np.copy(self.pc_array)
        for i in range(self.pc_height):
            for j in range(self.pc_width):
                nir = self.ms_array[i, j, 3]
                red = self.ms_array[i, j, 2]

                if not isnan(nir) and not isnan(red):
                    array[i, j][3] = (nir-red)/(nir+red)
                else:
                    array[i, j][3] = NaN

    def generate_mspc(self):
        self.pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(
            self.msg)  # shape (480,640)
        self.array = np.copy(self.pc_array)
        self.pc_height = self.pc_array.shape[0]
        self.pc_width = self.pc_array.shape[1]
        self.ms_array = np.full(
            [self.pc_height, self.pc_width, self.ms_bands], NaN)

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
                            self.ms_array[i, j, n] = self.images[n][v, u]

    def capture(self):
        self.stereo_client.update_configuration({'emitter_enabled': 0})
        rospy.sleep(1)
        requests.get('http://192.168.10.254/capture', {'block': True, })
        self.stereo_client.update_configuration({'emitter_enabled': 1})
        print("Capture complete")


def main():
    rospy.init_node('data_collector', anonymous=True)
    collector = DataCollector()
    collector.collect()


if __name__ == "__main__":
    main()
