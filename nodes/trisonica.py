#!/usr/bin/env python

import serial
import numpy as np
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
        msg = trisonica_msg()
        rate = rospy.Rate(80) # Hz, trisonica set to 40 Hz
        while not rospy.is_shutdown():
            data = self.connection.readline()
            if data is not None and len(data) > 10:
                if 1: #data[0] == 'S':
                    msg.header.stamp.secs = rospy.Time.now().secs
                    msg.header.stamp.nsecs = rospy.Time.now().nsecs
                    try:
                        msg.speed       = float( data.split('S ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.speed = np.nan 
                        pass

                    try:
                        msg.speed2d       = float( data.split('S2 ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.speed2d = np.nan  
                        pass

                    try:
                        msg.direction   = float( data.split('D ')[1].lstrip().split(' ')[0])
                    except:
                        #msg.direction = np.nan   
                        pass

                    try:
                        msg.northsouth  = float( data.split('U ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.northsouth = np.nan  
                        pass

                    try:
                        msg.westeast    = float( data.split('V ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.westeast = np.nan  
                        pass

                    try:
                        msg.updown      = float( data.split('W ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.updown = np.nan  
                        pass

                    try:
                        msg.temperature = float( data.split('T ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.temperature = np.nan  
                        pass

                    try:
                        msg.pressure = float( data.split('P ')[1].lstrip().split(' ')[0] )                
                    except:
                        #msg.pressure = np.nan  
                        pass

                    try:
                        msg.humidity = float( data.split('H ')[1].lstrip().split(' ')[0] )                
                    except:
                        #msg.humidity = np.nan   
                        pass

                    try:
                        msg.pitch = float( data.split('P ')[2].lstrip().split(' ')[0] )
                    except:
                        try:
                            msg.pitch = float( data.split('PI ')[1].lstrip().split(' ')[0] )
                        except:
                            #msg.pitch = np.nan  
                            pass

                    try:
                        msg.roll = float( data.split('RO ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.roll = np.nan   
                        pass

                    try:
                        msg.heading = float( data.split('MD ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.heading = np.nan   
                        pass

                    try:
                        msg.levelx = float( data.split('AX ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levelx = np.nan   
                        pass

                    try:
                        msg.levely = float( data.split('AY ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levely = np.nan   
                        pass

                    try:
                        msg.levelz = float( data.split('AZ ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levelz = np.nan   
                        pass




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
