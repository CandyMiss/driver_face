#!/usr/bin/env python
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GObject, Gst, GstRtspServer

def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge

    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    x, y = cv_img.shape[0:2]
    cv_img=cv2.resize(cv_img,(640,360))
    cv2.imshow("frame" , cv_img)
    cv2.waitKey(1)
    print "send......\n"

    out_send.write(cv_img)
    cv2.waitKey(30) 

 
def displayWebcam():
    rospy.init_node('rtsp_node', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/render/frames', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':
    out_send = cv2.VideoWriter('appsrc is-live=true ! videoconvert ! \
                                omxh265enc bitrate=1200000 ! video/x-h265, \
                                stream-format=byte-stream ! rtph265pay pt=96 ! \
                                udpsink host=127.0.0.1 port=5401 async=false',
                                cv2.CAP_GSTREAMER, 0, 15, (640,360), True)

  
 
    if not out_send.isOpened():
        print 'VideoWriter not opened'
        exit(0)
 
    rtsp_port_num = 8554 
 
    server = GstRtspServer.RTSPServer.new()
    server.props.service = "%d" % rtsp_port_num
    server.attach(None)
    
    factory = GstRtspServer.RTSPMediaFactory.new()
    factory.set_launch("(udpsrc name=pay0 port=5401 buffer-size=524288 \
                        caps=\"application/x-rtp, media=video, clock-rate=90000, \
                        encoding-name=(string)H265, payload=96 \")")
                        
    factory.set_shared(True)
    server.get_mount_points().add_factory("/cam", factory)
 
    print "\n *** Launched RTSP Streaming at rtsp://localhost:%d/ds-test ***\n\n" % rtsp_port_num   
 
    #cap = cv2.VideoCapture(0)

    # Test....



    displayWebcam()
