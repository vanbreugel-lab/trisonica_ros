#!/usr/bin/env python

import serial
import rospy
from optparse import OptionParser
from trisonica_ros.msg import trisonica_msg

class Trisonica(object):
    def __init__(self, port="/dev/ttyUSB0", topic='/trisonica'):
        baud = 115200
        print('Connecting to: ', port)
        self.connection = serial.Serial(port, baud, timeout=0.01)
        self.connection.flush()
        print('Connected.')
        self.publisher = rospy.Publisher(topic, trisonica_msg, queue_size=10)

    def main(self):
        rate = rospy.Rate(80) # Hz, trisonica set to 40 Hz
        while not rospy.is_shutdown():
            data = self.connection.readline()
            if data is not None and len(data) > 10:
                if data[0] == 'S':
                    msg = trisonica_msg()
                    msg.header.stamp.secs = rospy.Time.now().secs
                    msg.header.stamp.nsecs = rospy.Time.now().nsecs

                    msg.northsouth  = float( data.split('U ')[1].lstrip().split(' ')[0] )
                    msg.westeast    = float( data.split('V ')[1].lstrip().split(' ')[0] )
                    msg.updown      = float( data.split('W ')[1].lstrip().split(' ')[0] )
                    msg.temperature = float( data.split('T ')[1].lstrip().split(' ')[0] )

                    self.publisher.publish(msg)

            rate.sleep()

        self.connection.close()

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default='/dev/ttyUSB0',
                        help="port to which trisonica is connected")
    parser.add_option("--topic", type="str", dest="topic", default='/trisonica',
                        help="rostopic to publish to")

    (options, args) = parser.parse_args()

    rospy.init_node('trisonica', anonymous=True)

    trisonica = Trisonica(port=options.port, topic='/trisonica')
    trisonica.main()

