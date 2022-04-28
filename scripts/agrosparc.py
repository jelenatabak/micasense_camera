#!/usr/bin/env python3

from turtle import shape
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

from micasense_camera.ssh_tunnel import *
from micasense_camera.connect import *
from micasense_camera.db_interaction import *

import datetime
import rosbag
import urllib.request

import pysftp
import shutil

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

        self.pc_topic = 'camera/depth_registered/points'
        self.rs_topic = 'camera/color/image_raw'

        self.local_path = '/home/franka/catkin_ws/src/micasense_camera/agrosparc_data'
        self.base_path = '/mnt/data/home/agrosparc/agrosparc_data'
        os.umask(0)

        self.ms_camera = []
        for i in range(1, 6):
            xml_path = '/home/franka/catkin_ws/src/micasense_camera/intrinsics/ms_red_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))
        for i in range(1, 6):
            xml_path = '/home/franka/catkin_ws/src/micasense_camera/intrinsics/ms_blue_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))

        rospy.sleep(5)
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
        # self.stereo_client = dynamic_reconfigure.client.Client(
        #     '/camera/stereo_module')

        self.server = create_SSH_tunnel()
        try:
            self.server.start()
            print('SSH tunnel is open')
        except:
            print("SSH tunnel wasn't openned")

        self.conn = connect()
        print('Initialized!')

        remote_host = '161.53.68.231'
        remote_user = 'agrosparc_user'
        remote_user_pass = 'vege123!'
        self.srv = pysftp.Connection(host=remote_host, username=remote_user, password=remote_user_pass)


    def collect_data(self):
        while not rospy.is_shutdown():
            self.cam = fetch_camera_trigger(self.conn)
            if not self.cam.empty and self.cam['trigger'].iloc[-1]:
                self.plant_id = int(self.cam['plant_id'].iloc[-1])
                self.zone_id = int(self.cam['zone_id'].iloc[-1])
                self.device_id = int(self.cam['device_id'].iloc[-1])

                print('Collecting data')
                self.ts = datetime.datetime.now()
                self.pc_msg = rospy.wait_for_message(self.pc_topic, PointCloud2)
                self.rs_msg = rospy.wait_for_message(self.rs_topic, Im)
                print('Recieved pc and rs messages')
                self.capture()
                self.store_data()

                # self.generate_mspc()
                # self.calculate_indices()
                self.publish_data()
                print('Data written to DB')

                ts = self.cam['timestamp'].iloc[-1]
                update_camera_trigger(False,'laptop',ts,self.conn)
                print('Trigger updated')

            else:
                print("Waiting for the trigger...")
                rospy.sleep(3)

    def publish_data(self):
        batch_ts = self.ts
        ts = self.ts
        plant_id = int(self.cam['plant_id'].iloc[-1])
        zone_id = int(self.cam['zone_id'].iloc[-1])
        device_id = int(self.cam['device_id'].iloc[-1])
        image_path = self.path
        insert_values = (batch_ts, ts, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, self.plant_id, self.zone_id, self.device_id, image_path)
        insert_camera_measurements(insert_values, self.conn)


    def store_data(self):
        os.mkdir(self.local_path)
        self.path = os.path.join(self.base_path, str(self.device_id) + "_" + str(self.zone_id) + "_" + str(self.ts))
        self.srv.makedirs(self.path, mode=777)

        # save bag files
        bag_name = str(self.ts) + '.bag'
        bag_path = os.path.join(self.local_path, bag_name)
        bag = rosbag.Bag(bag_path, 'w')
        bag.write(self.pc_topic, self.pc_msg)
        bag.write(self.rs_topic, self.rs_msg)
        bag.close()
        print('Bags saved: ' + bag_path)

        # save ms_red images
        paths = self.r.json().get('raw_storage_path')
        for i in range(1,6):
            path = paths.get(str(i))
            path_split = path.split('/')
            write_path = os.path.join(self.local_path, path_split[2], path_split[3])
            print(write_path)
            os.umask(000)
            os.makedirs(write_path, mode=0o777, exist_ok=True)
            req = 'http://192.168.10.254' + path
            f = open(os.path.join(write_path, path_split[4]), 'wb')
            f.write(urllib.request.urlopen(req).read())
            f.close()
            print('Image saved: ' + write_path)

        # copy to server and delete local copy
        self.srv.put_r(self.local_path, self.path, preserve_mtime=True)
        shutil.rmtree(self.local_path)

    def calculate_indices(self):
        self.ndvi_array = np.copy(self.pc_array)
        for i in range(self.pc_height):
            for j in range(self.pc_width):
                nir = self.ms_array[i, j, 3]
                red = self.ms_array[i, j, 2]

                if not isnan(nir) and not isnan(red):
                    self.ndvi_array[i, j][3] = (nir-red)/(nir+red)
                else:
                    self.ndvi_array[i, j][3] = NaN
    
    def calculate_indices_test(self):
        pc_pub = rospy.Publisher("/test_pc", PointCloud2, queue_size=10)

        self.images = []
        for i in range(1, 6):
            path = '/home/jelena/bags/agrosparc_test/IMG_0003_' + str(i) + '.tif'
            #self.images.append(cv2.imread(path, cv2.IMREAD_UNCHANGED))
            self.images.append(Image(path).undistorted_reflectance())

        self.ms_bands = 5

        test_bag = rosbag.Bag('/home/jelena/bags/agrosparc_test/test.bag')
        for topic, msg, t in test_bag.read_messages(topics=['camera/depth_registered/points']):
            self.pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

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
                        point_transformed = np.dot(self.T[n], point)

                        u, v = self.ms_camera[n].get_pixel_coordinates(
                            point_transformed)
                        if u is not None and v is not None:
                            self.ms_array[i, j, n] = self.images[n][v, u]

        self.ndvi_array = np.copy(self.pc_array)
        nir_thresh = 0.15
        ind = np.zeros(12)
        counter = 0

        # for i in range(self.pc_height):
        #     for j in range(self.pc_width):

        for i in range(320,480):
            for j in range(300,390):
                blue = self.ms_array[i, j, 0]
                green = self.ms_array[i, j, 1]
                red = self.ms_array[i, j, 2]
                nir = self.ms_array[i, j, 3]
                re = self.ms_array[i, j, 4]

                if not isnan(nir) and not isnan(red) and not isnan(green) and not isnan(re) and not isnan(blue) and nir > nir_thresh:
                    counter += 1
                    ind[0] += (nir-red)/(nir+red)
                    ind[1] += nir/red
                    ind[2] += (nir-green)/(nir+green)
                    ind[3] += 0.16*(nir-red)/(nir+red+0.16)
                    ind[4] += (2*nir + 1 - sqrt(pow((2*nir+1), 2) - 8*(nir-red))) / 2
                    ind[5] += 1.5*(1.2*(nir-green)-2.5*(red-green)) / sqrt(pow((3*nir+1),2)-(6*nir-5*sqrt(red))-0.5) 
                    ind[6] += 1.16*(nir-red)/(nir+red+0.16)
                    ind[7] += nir - re/nir + re
                    ind[8] += 2.5*(nir-red) / (nir+6*red-7.5*blue+1)
                    ind[9] += 0.5*(120*(nir-green)-200*(red-green))
                    ind[10] += (nir-red) / sqrt(nir+red)
                    ind[11] += nir-red

                    self.ndvi_array[i, j][3] = (nir-red)/(nir+red)
                else:
                    self.ndvi_array[i, j][3] = NaN

        avg_indices = ind / counter
        print(avg_indices)

        msg = ros_numpy.point_cloud2.array_to_pointcloud2(self.ndvi_array, frame_id='base')

        while not rospy.is_shutdown():
            pc_pub.publish(msg)
            rospy.sleep(5)
            print('PC published!')

    def find_thresh(self):
        path = '/home/jelena/bags/agrosparc_test/IMG_0003_4.tif' 
        img = Image(path).undistorted_reflectance()

        cv2.imshow("test", img)
        cv2.waitKey()



    # reflectance, not raw!
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
                        point_transformed = np.dot(self.T[n], point)

                        u, v = self.ms_camera[n].get_pixel_coordinates(
                            point_transformed)
                        if u is not None and v is not None:
                            self.ms_array[i, j, n] = self.images[n][v, u]

    def capture(self):
        self.stereo_client.update_configuration({'emitter_enabled': 0})
        rospy.sleep(2)
        self.r = requests.get('http://192.168.10.254/capture', {'block': True, })
        self.stereo_client.update_configuration({'emitter_enabled': 1})
        print("Capture complete")


def main():
    rospy.init_node('data_collector', anonymous=True)
    
    path = '/home/jelena/bags/agrosparc_test/IMG_0003_4.tif' 

    # img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    # for y in range(960):
    #     for x in range(1280):
    #         nir = img[y,x]
    #         if nir < 30000: # manje
    #             img[y,x] = 0

    # img = Image(path).undistorted_reflectance()

    # for y in range(960):
    #     for x in range(1280):
    #         nir = img[y,x]
    #         if nir < 0.15:
    #             img[y,x] = 0


    # cv2.imshow("test", img)
    # cv2.imwrite('/home/jelena/bags/agrosparc_test/mask.tif', img)
    # cv2.waitKey()

    
    collector = DataCollector()
    collector.calculate_indices_test()

if __name__ == "__main__":
    main()
