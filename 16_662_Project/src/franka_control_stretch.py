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
                              translation=pose.translation - np.array([0, 0, 0.09]), from_frame='franka_tool', to_frame='world')
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
            
    def pickup_from_shelf(self, dest):
        if dest == 0:
            pose = RigidTransform(rotation = np.array(
                                    [[-0.05418246, 0.99519061, -0.08149097],
                                     [-0.03331916, -0.08336658, -0.99596169],
                                     [-0.99796535, -0.05124845, 0.03767665]]), 
                                  translation=np.array([0.60510356, -0.40945833, 0.46302854]),
                                  from_frame='franka_tool', to_frame='world')
            self.move_arm(pose)
            
        if dest == 1:
            pose = RigidTransform(rotation = np.array(
                                    [[-0.00308822, 0.99977382, 0.02057975],
                                     [-0.02398899, 0.02049966, -0.99950201],
                                     [-0.99969782, -0.00358037, 0.02392072]]), 
                                  translation=np.array([0.39279931, -0.39712991, 0.46067635]),
                                  from_frame='franka_tool', to_frame='world')
            self.move_arm(pose)

        if dest == 2:
            pose = RigidTransform(rotation = np.array(
                                    [[ 0.0049151, 0.99854743, 0.05347591],
                                     [-0.04110084, 0.05363258, -0.99771448],
                                     [-0.99913328, 0.00270597, 0.04130555]]), 
                                  translation=np.array([0.60503709, -0.41474504, 0.17499234]),
                                  from_frame='franka_tool', to_frame='world')
            self.move_arm(pose)

        if dest == 3:
            pose = RigidTransform(rotation = np.array(
                                    [[-0.03741543, 0.99380537, -0.10455586],
                                     [-0.02571732, -0.10555109, -0.99408116],
                                     [-0.99895918, -0.03450508, 0.02950782]]), 
                                  translation=np.array([0.37279638, -0.41601637, 0.1716907]),
                                  from_frame='franka_tool', to_frame='world')
            self.move_arm(pose)

        self.franka_arm.close_gripper()
            
    def dropoff_at_dropzone(self):
        pose = RigidTransform(rotation = np.array(
                                [[0.99972121, 0.02277065, -0.00444375],
                                [0.02251888, -0.99847327, -0.05024758],
                                [-0.00558113, 0.0501335, -0.99872691]]), 
                                translation=np.array([0.51558882, -0.03430918, 0.0531831 ]),
                                from_frame='franka_tool', to_frame='world')
        self.move_arm(pose)
        self.franka_arm.open_gripper()
        
        intermediate_pose = RigidTransform(rotation=pose.rotation, 
                              translation=pose.translation + np.array([0, 0, 0.3]), from_frame='franka_tool', to_frame='world')
        self.move_arm(intermediate_pose)
            
        
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