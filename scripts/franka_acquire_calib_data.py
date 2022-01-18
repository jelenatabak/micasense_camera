#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import tf
import dynamic_reconfigure.client

import requests


class AcquireData:
    def __init__(self):
        self.sub = rospy.Subscriber("/acquire_data", Bool, self.callback)
        self.listener = tf.TransformListener()
        self.stereo_client = dynamic_reconfigure.client.Client(
                '/camera/stereo_module')
        self.i = 1
        r = requests.get('http://192.168.10.254/capture',
                         {'block': True, 'preview': True})
        rospy.loginfo("Init image captured. Publish on /acquire_data when ready...")
        rospy.spin()

    def callback(self, msg):
        self.stereo_client.update_configuration({'emitter_enabled' : 0})
        requests.get('http://192.168.10.254/capture',
                         {'block': True, 'preview': True})
        self.stereo_client.update_configuration({'emitter_enabled' : 1})
        try:
            (trans, rot) = self.listener.lookupTransform(
                '/panda_link0', 'panda_camera', rospy.Time(0))
        except:
            rospy.loginfo("Ups! Could not get tf!")

        rpy = tf.transformations.euler_from_quaternion(rot)
        self.write_yaml(trans, rpy)
        rospy.loginfo("Recorded data pair number %d", self.i)
        self.i += 1

    def write_yaml(self, trans, rpy):
        f = open('pose_fPe_' + str(self.i) + '.yaml', 'x')
        f.write('rows: 6\n')
        f.write('cols: 1\n')
        f.write('data:\n')
        for t in trans:
            f.write('  - [' + str(t) + ']\n')
        for r in rpy:
            f.write('  - [' + str(r) + ']\n')
        f.close()


def main():
    rospy.init_node('calib_data_acquisition', anonymous=True)
    acquisition = AcquireData()


if __name__ == '__main__':
    main()
