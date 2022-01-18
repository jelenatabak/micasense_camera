#!/usr/bin/env python3

from msd import CameraModel

import cv2
import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix


class NDVI:
    def __init__(self):
        self.width = 1280
        self.height = 960

        self.ms_camera = []
        for i in range(1, 6):
            xml_path = '/home/jelena/catkin_ws_old/src/micasense_camera/intrinsics/ms_red_band_' + \
                str(i) + '.xml'
            self.ms_camera.append(CameraModel(xml_path))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(2)
        self.camera_frame = 'ms_red_band4'

        self.tfs = []
        for i in range(1, 6):
            ms_frame = 'ms_red_band' + str(i)
            try:
                trans = self.tf_buffer.lookup_transform(
                    ms_frame, self.camera_frame, rospy.Time(), rospy.Duration(2))
                self.tfs.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Did not get transform")
                return

        self.T = []
        for i in range(5):
            trans = self.tfs[i].transform.translation
            rot = self.tfs[i].transform.rotation
            Tx = np.eye(4, dtype="float32")
            Tx = np.asmatrix(quaternion_matrix([rot.x, rot.y, rot.z, rot.w]))
            Tx[0:3, 3] = np.resize([trans.x, trans.y, trans.z], (3, 1))
            self.T.append(Tx)

        self.images = []
        for i in range(1, 6):
            path = "/home/jelena/catkin_ws_old/src/micasense_camera/ndvi_paprika1/IMG_0002_" + \
                str(i) + '.tif'
            self.images.append(cv2.imread(path, cv2.IMREAD_UNCHANGED))

        # for img in self.images:
        #     cv2.imshow("test", img)
        #     cv2.waitKey(0)

    def ndvi(self):
        sum = 0
        cnt = 0
        thresh = 32000
        img = self.images[3].copy()
        mask = np.zeros_like(img, dtype=bool)
        for i in range(self.height):
            for j in range(self.width):
                px = img[i,j]
                if px > thresh:
                    #print("Truee")
                    mask[i,j] = True
                #else:
                    #mask[i,j] = 0
                #sum += px
                #cnt += 1

        #print(sum/cnt)

        img[~mask] = 0

        cv2.imshow("test", img)
        cv2.waitKey(0)


        # za band 4, maskiraj pixele ispod thresha
        # matrica (broj preostalih pix x5)
        # 4 stupac su ir pixeli
        # za svaki pixel:
            # prebaci ga u coord svijeta (nova funkcija u CameraModel?)
            # transformiraj u ks drugog banda
            # prebaci u pixele
            # dohvati pixel s te slike, stavi u tablicu
    


def main():
    rospy.init_node("ndvi")
    print("You gave up on this!!!!")
    ndvi = NDVI()
    ndvi.ndvi()


if __name__ == "__main__":
    main()
