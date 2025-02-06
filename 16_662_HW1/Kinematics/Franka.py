import numpy as np
import RobotUtil as rt
import math

class FrankArm:
    def __init__(self):
        # Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
        self.Rdesc = [
            [0, 0, 0, 0., 0, 0.333],  # From robot base to joint1
            [-np.pi/2, 0, 0, 0, 0, 0],
            [np.pi/2, 0, 0, 0, -0.316, 0],
            [np.pi/2, 0, 0, 0.0825, 0, 0],
            [-np.pi/2, 0, 0, -0.0825, 0.384, 0],
            [np.pi/2, 0, 0, 0, 0, 0],
            [np.pi/2, 0, 0, 0.088, 0, 0],
            [0, 0, 0, 0, 0, 0.107]  # From joint5 to end-effector center
        ]

        # Define the axis of rotation for each joint
        self.axis = [
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]
        ]

        # Set base coordinate frame as identity - NOTE: don't change
        self.Tbase = [[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]]

        # Initialize matrices - NOTE: don't change this part
        self.Tlink = []  # Transforms for each link (const)
        self.Tjoint = []  # Transforms for each joint (init eye)
        self.Tcurr = []  # Coordinate frame of current (init eye)
        
        for i in range(len(self.Rdesc)):
            self.Tlink.append(rt.rpyxyz2H(
                self.Rdesc[i][0:3], self.Rdesc[i][3:6]))
            self.Tcurr.append([[1, 0, 0, 0], [0, 1, 0, 0],
                              [0, 0, 1, 0.], [0, 0, 0, 1]])
            self.Tjoint.append([[1, 0, 0, 0], [0, 1, 0, 0],
                               [0, 0, 1, 0.], [0, 0, 0, 1]])

        self.Tlinkzero = rt.rpyxyz2H(self.Rdesc[0][0:3], self.Rdesc[0][3:6])

        self.Tlink[0] = np.matmul(self.Tbase, self.Tlink[0])

        # initialize Jacobian matrix
        self.J = np.zeros((6, 7))

        self.q = [0., 0., 0., 0., 0., 0., 0.]
        self.ForwardKin([0., 0., 0., 0., 0., 0., 0.])

    def ForwardKin(self, ang):
        '''
        inputs: joint angles
        outputs: joint transforms for each joint, Jacobian matrix
        '''

        self.q[0:-1] = ang

        # Compute current joint and end effector coordinate frames (self.Tjoint). Remember that not all joints rotate about the z axis!
        
        # Initial joint (joint and array index are offset by 1)
        self.Tjoint[0] = rt.rpyxyz2H([0, 0, self.q[0]], [0, 0, 0])
        self.Tcurr[0] = self.Tlink[0] @ self.Tjoint[0]

        # Subsequent joints
        for i in range(1, len(self.Rdesc)):
            self.Tjoint[i] = rt.rpyxyz2H([0, 0, self.q[i]], [0, 0, 0])
            self.Tcurr[i] = self.Tcurr[i-1] @ self.Tlink[i] @ self.Tjoint[i]

        # Jacobian computation according to DH convention
        for i in range(self.J.shape[1]):
            z = self.Tcurr[i][0:3, 2]
            p = self.Tcurr[-1][0:3, 3] - self.Tcurr[i][0:3, 3]

            self.J[0:3, i] = np.cross(z, p)
            self.J[3:6, i] = z

        return self.Tcurr, self.J

    def IterInvKin(self, ang, TGoal, x_eps=1e-3, r_eps=1e-3):
        '''
        inputs: starting joint angles (ang), target end effector pose (TGoal)

        outputs: computed joint angles to achieve desired end effector pose, 
        Error in your IK solution compared to the desired target
        '''

        W = np.diag([1, 1, 100, 100, 1, 1, 100])
        W[6, 0] = 1
        C = np.diag([1000000, 1000000, 1000000, 1000, 1000, 1000])

        epoch = 1
        max_epochs = 1000

        x_goal = TGoal[0:3, 3]
        x_err = np.inf
        x_step_rate = 10 * x_eps

        r_goal = TGoal[0:3, 0:3]
        r_err = np.inf
        r_step_rate = 10 * x_eps

        while epoch < max_epochs:

            # Check if converged or not
            if (x_err < x_eps and r_err < r_eps):
                print(f"Found IK solution in {epoch} epochs")
                break

            # Obtain current robot pose
            H_curr, J = self.ForwardKin(ang)
            x_curr = H_curr[-1][0:3, 3]
            r_curr = H_curr[-1][0:3, 0:3]

            # Rotational error
            r_err_matrix = r_goal @ r_curr.T
            r_axis, r_angle = rt.R2axisang(r_err_matrix)
            r_err_vector = r_angle * np.array(r_axis)
            r_angle_clipped = np.clip(r_angle, -r_step_rate, r_step_rate)
            r_err_vector_clipped = r_angle_clipped * np.array(r_axis)

            # Translational error
            x_err_vector = x_goal - x_curr
            x_err_vector_clipped = np.clip(x_err_vector, -x_step_rate, x_step_rate)

            # Total error
            x_err = np.linalg.norm(x_err_vector)
            r_err = r_angle
            err_vector = np.concatenate([x_err_vector_clipped, r_err_vector_clipped])

            # Compute pseudo-inverse Jacobian and iterate desired angle
            ps_J = np.linalg.inv(W) @ J.T @ np.linalg.inv(J @ np.linalg.inv(W) @ J.T + np.linalg.inv(C))
            ang += (ps_J @ err_vector)
            epoch += 1

        Err = np.concatenate([x_err_vector, r_err_vector])

        return self.q[0:-1], Err
