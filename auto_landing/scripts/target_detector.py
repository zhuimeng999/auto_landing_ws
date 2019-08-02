#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import argparse

def callback(data):
    print(data)

def listener(uav_name):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('target_detector', anonymous=True)

    rospy.Subscriber('/'+uav_name+'/usb_cam/image_raw', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='target detecter module')
    parser.add_argument('-n', '--name', metavar='uav prefix', type=str, nargs='?',
                        help='name of uav')
    args = parser.parse_args()
    args.name = args.name or 'uav0'
    listener(args.name)
