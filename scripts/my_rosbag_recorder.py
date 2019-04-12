#!/usr/bin/env python
import datetime

import rospy
import rosbag

from test_msgs.msg import RobikeStatus
from record_rosbag_oncall.msg import RecordDemand

import struct
import subprocess


class RosBagRecorder:
    def __init__(self):

        self.bag_no = 0
        self.is_recording = False
        self.topic_to_record = '/data_to_xavier'
        self.bag_name_prefix = ''
        self.bagfile_dir = '/home/longfei/test_ws/src/record_rosbag_oncall/bagfiles'
        self.subprocess_pid = 0

        ##subscribe the imu and motordata
        rospy.Subscriber("/data_to_xavier", RobikeStatus, self.callback)

    
    #delete function
    def __del__(self):
        exit(0)


    def callback(self, msg):
        status = msg.status
        status_bit_str = '{0:032b}'.format(status)
        print('\n====================================')
        print('status_bit_str is: ', status_bit_str)

        if status_bit_str[0] == '0':
            if self.is_recording:
                rospy.loginfo('status_bit_str[0] = 0 and is recording')
                try:
                    kill_cmd = 'kill -SIGINT ' + str(self.subprocess_pid)
                    rospy.loginfo('killed process with PID: %d', self.subprocess_pid)
                    self.is_recording = False
                except:
                    rospy.logerr('kill subprocess failed, subprocess_pid is %d', self.subprocess_pid)
            else:
                rospy.loginfo('status_bit_str[0] = 0 and is NOT recording')
                pass
            
        elif status_bit_str[0] == '1':
            if not self.is_recording:
                rospy.loginfo('status_bit_str[0] = 1 and is NOT recording')

                record_cmd = 'rosbag record /data_to_xavier -O ' + self.bag_name_prefix + datetime.datetime.now().strftime('%H%M%S-%Y%m%d') + '.bag'
                rospy.loginfo('record_cmd is" $ ' + record_cmd)

                p = subprocess.Popen(record_cmd, stdin=subprocess.PIPE, shell=True, cwd=self.bagfile_dir)
                self.subprocess_pid = p.pid
                rospy.loginfo('recorder started with PID: %d', self.subprocess_pid)

                self.is_recording = True
                self.bag_no += 1
            else:
                rospy.loginfo('status_bit_str[0] = 1 and is recording')
                pass
        
        else:
            rospy.logerr('record_demand_str should be 32bit integer')


if __name__ == "__main__":
    rospy.init_node('rospy_test')
    recorder = RosBagRecorder()
    rospy.spin()

    

