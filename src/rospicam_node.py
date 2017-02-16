#!/usr/bin/env python

from picamera import PiCamera
from io import BytesIO

# ros imports
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage

class RosPiCam:
    def __init__(self):
        print "rospicam start"

        # publishers
        self.image_pub = rospy.Publisher('/rospicam/image/compressed', CompressedImage, queue_size=10)

        # init the node
        rospy.init_node('rospicam', anonymous=True)

        stream = BytesIO()
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 30
        
        for foo in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
            stream.truncate()
            stream.seek(0)

            self.publish_image(stream)

            if rospy.is_shutdown():
                break

        stream.close()
        camera.close()

        print "rospicam stop"

    def publish_image(self, stream):
        image_msg = CompressedImage()
        image_msg.header = Header()
        image_msg.format = 'jpeg'
        image_msg.data = stream.getvalue()

        self.image_pub.publish(image_msg)
        return rospy.is_shutdown()


if __name__ == '__main__':
    try:
        RosPiCam()
    except rospy.ROSInterruptException:
        pass
