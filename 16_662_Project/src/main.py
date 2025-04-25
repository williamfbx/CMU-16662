from dotenv import load_dotenv
from gpt_parser import Parser
import os
import matplotlib.pyplot as plt
import numpy as np

import rospy
from sensor_msgs.msg import Image

from single_image_capture import ImageCapture

from segment import Segment

from franka_control import FrankaMoveIt

from calibration import pick_up

# import pytesseract

# from autolab_core import RigidTransform

def main():
    # get input from user
    print("Please enter a command:")
    sentence = input("> ")
    parsed_result = parser.parse_sentence(sentence)
    print(parsed_result)

    # Photo taking position
    fa.move_to_reset_position()

    # Capture image
    if not rospy.core.is_initialized():
        rospy.init_node("neurogrip", anonymous=True)
    topic_name = "/camera/color/image_raw"
    capture = ImageCapture(topic_name, image_path)

    while not capture.is_done() and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # Segment image
    try:
        results, center_point = seg.segment(image_path, parsed_result["object"])
    except Exception as e:
        print(f"Specified item not found. Try again.")
        fa.reset_joints()
        return None
    
    # print(center_point)
    
    x, y = pick_up([center_point[1], center_point[0]])
    
    # print(x, y)

    # Pre-position for pick
    fa.move_to_pickup_position(x, y)
    
    # Pick
    fa.pickup()
    
    # Pre-position for drop
    fa.move_to_dropoff_position(parsed_result["destination"])
    
    # Drop
    fa.dropoff()
    

if __name__ == "__main__":
    load_dotenv()

    api_key = os.getenv("OPENAI_API_KEY")
    parser = Parser(api_key)

    seg = Segment()
    image_path = "captured_image.jpg"

    # initialize franka
    fa = FrankaMoveIt()
    rate = rospy.Rate(10)
    fa.move_to_reset_position()

    while not rospy.is_shutdown():
        main()
