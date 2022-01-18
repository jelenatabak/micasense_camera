#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import dynamic_reconfigure.client

import requests
import os


class MicasenseCapture:

    def __init__(self, preview, realsense):
        self.srv = rospy.Service('capture', Trigger, self.capture)
        self.preview = preview
        self.realsense = realsense
        self.addr = '192.168.10.254'
        if realsense:
            self.stereo_client = dynamic_reconfigure.client.Client(
                '/camera/stereo_module')
        rospy.spin()

    def capture(self, req):
        if self.realsense:
            self.stereo_client.update_configuration({'emitter_enabled': 0})
            rospy.sleep(1)

        res = 1
        while not res:
            res = os.system("ping -c 1 " + self.addr)
            print("Wating for camera...")
     
        print("Camera is up!")
        requests.get('http://192.168.10.254/capture',
                    {'block': True, 'preview': self.preview})
            
        if self.realsense:
            self.stereo_client.update_configuration({'emitter_enabled': 1})

        print("Capture complete")
        return TriggerResponse(True, 'Capture complete.')


def main():
    rospy.init_node('micasense_capture', anonymous=True)
    camera = MicasenseCapture(preview=True, realsense=True)


if __name__ == '__main__':
    main()
