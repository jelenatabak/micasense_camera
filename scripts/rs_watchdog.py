#!/usr/bin/env python3

import rospy
import rostopic
import roslaunch
from rospkg import RosPack
import os

def main():
    rospy.init_node('watchdog', anonymous=True)
    topic = '/camera/depth_registered/points'
    r = rostopic.ROSTopicHz(-1)
    s = rospy.Subscriber(topic, rospy.AnyMsg, r.callback_hz, callback_args=topic)  

    rp = RosPack()
    launch_file_path = os.path.join(rp.get_path('realsense2_camera'), 'launch', 'rs_rgbd.launch')


    while not rospy.is_shutdown():
        rospy.sleep(1)  
        hz = r.get_hz(topic)
        
        if hz is None or hz[0] < 1:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
            launch.start()
            rospy.loginfo("Started realsense")


if __name__ == "__main__":
    main()