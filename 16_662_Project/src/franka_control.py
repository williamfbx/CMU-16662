import rospy
from geometry_msgs.msg import Pose
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

class FrankaMoveIt:
    def __init__(self):
        self.franka_arm = FrankaArm()
        
    def move_to_reset_position(self):
        # Reset position on top of pickup grid on the table
        reset_pose = RigidTransform(rotation=np.array(
                                    [[ 0.99981387, -0.01266928, 0.01387308],
                                     [-0.01286038, -0.99981281, 0.01377354],
                                     [0.01369598, -0.01394939, -0.9998089 ]]),
                                    translation=np.array([0.57575525, 0.32240383, 0.36945362]),
                                    from_frame='franka_tool', to_frame='world')

        self.move_arm(reset_pose)
        self.franka_arm.open_gripper()
        
    def move_to_pickup_position(self, x, y):
        # Move to 10cm above the object
        pose = RigidTransform(rotation=np.array(
                                    [[ 0.99981387, -0.01266928, 0.01387308],
                                     [-0.01286038, -0.99981281, 0.01377354],
                                     [0.01369598, -0.01394939, -0.9998089 ]]),
                                    translation=np.array([x, y, 0.1]),
                                    from_frame='franka_tool', to_frame='world')
        
        self.move_arm(pose)
        self.franka_arm.open_gripper()
    
    def pickup(self):
        # Pick up the object and return to the reset position
        pose = self.franka_arm.get_pose()
        pose = RigidTransform(rotation=pose.rotation, 
                              translation=pose.translation - np.array([0, 0, 0.07]), from_frame='franka_tool', to_frame='world')
        self.move_arm(pose)
        self.franka_arm.close_gripper()
        intermediate_pose = RigidTransform(rotation=np.array(
                                    [[ 0.99981387, -0.01266928, 0.01387308],
                                     [-0.01286038, -0.99981281, 0.01377354],
                                     [0.01369598, -0.01394939, -0.9998089 ]]),
                                    translation=np.array([0.57575525, 0.32240383, 0.36945362]),
                                    from_frame='franka_tool', to_frame='world')

        self.move_arm(intermediate_pose)
    
    def move_to_dropoff_position(self, dest):
        # Move to 5cm in front of the dropoff location
        # {0, 1, 2, 3} corresponds to {top-left, top-right, bottom-left, bottom-right} respectively
        
        # TODO: If the dropoff location is taken, return the item to the rejected items bin
        
        self.franka_arm.reset_joints()
        
        if dest == 0:
            pose = RigidTransform(rotation = np.array(
                                    [[ 0.05591694,  0.9980864,  -0.02603066],
                                    [ 0.00331084, -0.02625641, -0.99964975],
                                    [-0.99842029 , 0.05581118, -0.00477277]]), 
                                  translation=np.array([ 0.60988562, -0.21788976,  0.48560873]),
                                  from_frame='franka_tool', to_frame='world') 
            self.move_arm(pose)
        elif dest == 1:
            pose = RigidTransform(rotation = np.array(
                                    [[ 0.05591694,  0.9980864,  -0.02603066],
                                    [ 0.00331084, -0.02625641, -0.99964975],
                                    [-0.99842029 , 0.05581118, -0.00477277]]), 
                                  translation=np.array([ 0.38308252, -0.19890911,  0.46523199]),
                                  from_frame='franka_tool', to_frame='world') 
            self.move_arm(pose)
        elif dest == 2:
            pose = RigidTransform(rotation = np.array(
                                    [[ 0.05591694,  0.9980864,  -0.02603066],
                                    [ 0.00331084, -0.02625641, -0.99964975],
                                    [-0.99842029 , 0.05581118, -0.00477277]]), 
                                  translation=np.array([ 0.5969059,  -0.21899883,  0.17408864]),
                                  from_frame='franka_tool', to_frame='world') 
            self.move_arm(pose)
        elif dest == 3:
            pose = RigidTransform(rotation = np.array(
                                    [[ 0.05591694,  0.9980864,  -0.02603066],
                                    [ 0.00331084, -0.02625641, -0.99964975],
                                    [-0.99842029 , 0.05581118, -0.00477277]]), 
                                  translation=np.array([ 0.37491001, -0.22476681,  0.17301884]),
                                  from_frame='franka_tool', to_frame='world') 
            self.move_arm(pose)
        else:
            print("Invalid dropoff location")
    
    def dropoff(self):
        # Drop off the object and return to the reset position
        pose_init = self.franka_arm.get_pose()
        pose = RigidTransform(rotation=pose_init.rotation, 
                              translation=pose_init.translation - np.array([0, 0.15, 0]), from_frame='franka_tool', to_frame='world')
        self.move_arm(pose)
        self.franka_arm.open_gripper()
        
        self.move_arm(pose_init)
        self.franka_arm.reset_joints()
        self.move_to_reset_position()

    def move_arm(self, pose):
        self.franka_arm.goto_pose(pose)
        
    def reset_joints(self):
        self.franka_arm.reset_joints()
        
        
if __name__ == "__main__":

    fa = FrankaMoveIt()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        fa.main()
        rate.sleep()