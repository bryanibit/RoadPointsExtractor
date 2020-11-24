#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rosbag
import cv2
import argparse
from rcs_msg_wrapper.msg import dwdx
def main():
    parser = argparse.ArgumentParser(description="extract dwdw from rosbag")
    parser.add_argument("--bag_file", help="Input rosbag")
    parser.add_argument("--output_file", help="Output file")
    parser.add_argument("--extract_topic", help="extracted file")
    args = parser.parse_args()
    try:
        bag = rosbag.Bag(args.bag_file, "r")
        count = 0
        with open(args.output_file, 'w') as out:
            for topic, msg, t in bag.read_messages():
                if topic == args.extract_topic:
                    out.write('{0},{1},0,2\n'.format(msg.longitude, msg.latitude))
            print('finish write and close file')
    finally:
        bag.close()
    return
if __name__=='__main__':
    main()
