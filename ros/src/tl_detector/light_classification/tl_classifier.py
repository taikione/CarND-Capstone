from styx_msgs.msg import TrafficLight
import time
import cv2
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.count = 0
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # make dataset
        if self.count == 3:
            filename = "tl_images/{}.jpg".format(time.time())
            # cv2.imwrite(filename, image)
            rospy.loginfo("Saved image")
            self.count = 0

        elif self.count > 3:
            self.count = 0

        else:
            self.count += 1
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
