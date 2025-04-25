#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageCapture:
    def __init__(self, topic_name, output_file="captured_image.jpg"):
        # Create a CvBridge object
        self.bridge = CvBridge()
        # Flag to ensure only one image is captured per instance
        self.image_captured = False
        # Store output file name
        self.output_file = output_file
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(topic_name, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to {topic_name} for capturing one image.")

    def image_callback(self, data):
        if not self.image_captured:
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
                # Save the image to the specified file
                cv2.imwrite(self.output_file, cv_image)
                rospy.loginfo(f"Image captured and saved as '{self.output_file}'")
                # Mark as captured
                self.image_captured = True
                # Unsubscribe from the topic to stop further callbacks
                self.image_sub.unregister()
                rospy.loginfo("Unsubscribed from topic after capturing image.")
            except CvBridgeError as e:
                rospy.logerr(f"Error converting image: {e}")

    def is_done(self):
        # Method to check if the capture is complete
        return self.image_captured

def main():
    # Initialize the ROS node (only once for the application)
    if not rospy.core.is_initialized():
        rospy.init_node('image_capture_node', anonymous=True)

    # Example usage: Capture images one at a time from the same topic
    topic_name = "/camera/color/image_raw"  # Replace with your topic name

    # First instance: Capture one image
    capture = ImageCapture(topic_name)
    while not capture.is_done() and not rospy.is_shutdown():
        rospy.sleep(0.1)  # Wait briefly for the callback to process
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
