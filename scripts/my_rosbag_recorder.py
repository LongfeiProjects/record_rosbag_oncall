#!/usr/bin/env python
import os
import signal
import datetime

import rospy
import rosbag

# from robike_msgs.msg import RobikeStatusStamped
from test_msgs.msg import RobikeStatusStamped

import struct
import subprocess


class RosBagRecorder:
    def __init__(self):

        self.bag_no = 0
        self.is_recording = False
        self.topic_to_record = '/data_to_xavier'
        self.bag_name_prefix = ''
        self.bagfile_dir = os.path.dirname(__file__) + '/../bagfiles'
        self.subprocess_pid = 0
        ##subscribe the imu and motordata
        rospy.Subscriber("/data_to_xavier", RobikeStatusStamped, self.callback)

        if not os.path.isdir(self.bagfile_dir):
            os.mkdir(self.bagfile_dir)


    #delete function
    def __del__(self):
        exit(0)


    def callback(self, msg):
        status = msg.status.status
        status_bit_str = '{0:032b}'.format(status)

        if status_bit_str[0] == '0':
            if self.is_recording:
                rospy.logdebug('status_bit_str[0] = 0 and is recording')
                try:
                    subprocess_group_pid = os.getpgid(self.subprocess_pid)
                    os.killpg(os.getpgid(self.subprocess_pid), signal.SIGINT)

                    rospy.loginfo('Killed subprocess_group_pid: %d. Bag file is saved in directory: %s/ \n', subprocess_group_pid, self.bagfile_dir)
                    self.is_recording = False
                except:
                    rospy.logerr('kill subprocess failed, subprocess_pid is %d', self.subprocess_pid)
            
        elif status_bit_str[0] == '1':
            if not self.is_recording:
                rospy.logdebug('status_bit_str[0] = 1 and is NOT recording')

                record_cmd = 'rosbag record /data_to_xavier -O ' + self.bag_name_prefix + datetime.datetime.now().strftime('%H%M%S-%Y%m%d') + '.bag'
                rospy.loginfo('record_cmd is" $ ' + record_cmd)

                p = subprocess.Popen(record_cmd, stdin=subprocess.PIPE, shell=True, preexec_fn=os.setsid, cwd=self.bagfile_dir)
                self.subprocess_pid = p.pid
                rospy.loginfo('recorder started with PID: %d', self.subprocess_pid)

                self.is_recording = True
                self.bag_no += 1

                rospy.loginfo('start to record ...')
        
        else:
            rospy.logerr('record_demand_str should be 32bit integer')


if __name__ == "__main__":
    rospy.init_node('rosbag_recorder', log_level=rospy.INFO)
    rospy.loginfo('rosbag_recorder started')
    recorder = RosBagRecorder()
    rospy.spin()

    

