#!/home/boxiangf/anaconda3/envs/16662_env/bin/python

import mujoco as mj
from mujoco import viewer
import numpy as np
import math
import quaternion

# Set the XML filepath
xml_filepath = "../franka_emika_panda/panda_nohand_torque_sine_board.xml"

################################# Control Callback Definitions #############################

def get_board_control(model, data):
    # Instantite a handle to the desired body on the robot
    omega = 0.4
    data.ctrl[7] = 0.15*np.sin(omega*data.time) + 0.15

# Control callback for gravity compensation
def gravity_comp(model, data):
    # data.ctrl exposes the member that sets the actuator control inputs that participate in the
    # physics, data.qfrc_bias exposes the gravity forces expressed in generalized coordinates, i.e.
    # as torques about the joints

    data.ctrl[:7] = data.qfrc_bias[:7]

# Force control callback
def force_control(model, data):
    # Copy the code that you used to implement the controller from Part1 here

    # Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # Get the Jacobian for the desired location on the robot (The end-effector)

    # This function works by taking in return parameters!!! Make sure you supply it with placeholder
    # variables
    body_id = body.id
    point = body.xpos
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    mj.mj_jac(model, data, jacp, jacr, point, body_id)

    jac = np.vstack((jacp, jacr))

    # Specify the desired force in global coordinates
    force_des = np.array([15, 0, 0, 0, 0, 0])

    # Compute the required control input using desired force values
    tau = jac.T @ force_des

    # Set the control inputs
    data.ctrl[:7] = tau[:7] + data.qfrc_bias[:7]

    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION

    # Force readings updated here
    force[:] = np.roll(force, -1)[:]
    force[-1] = data.sensordata[2]

    # Update control inputs to the whiteboard
    get_board_control(model, data)

# Control callback for an impedance controller
def impedance_control(model, data):
    # Copy the code that you used to implement the controller from Part1 here

    # PD tuning values
    Kp = 20
    Kd = 0.5

    # Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # Set the desired position
    # The white board is at location [0.6, 0, 0.7], the + 15/Kp is the desired pseudo-force from PD controller in equilibrium
    # when modelled by a spring-damper system
    pos_des = np.array([0.6 + 15/Kp, 0, 0.7])

    # Set the desired velocities (including angular)
    vel_des = np.array([0, 0, 0, 0, 0, 0])

    # Set the desired orientation (Use numpy quaternion manipulation functions)
    # Initial position already has correct pose
    quat_des = np.quaternion(-0.492339943056984, 0.507538422109313, -0.507740563303762, 0.49214393314295)

    # Get the current orientation
    quat_curr_array = body.xquat
    quat_curr = np.quaternion(quat_curr_array[0], quat_curr_array[1], quat_curr_array[2], quat_curr_array[3])

    # Get orientation error. Error between two quaternions is calculated according to
    # https://math.stackexchange.com/questions/3572459/how-to-compute-the-orientation-error-between-two-3d-coordinate-frames
    quat_curr_inv = np.quaternion.conjugate(quat_curr)
    quat_err = quat_des * quat_curr_inv
    theta_err = np.array([quat_err.x, quat_err.y, quat_err.z])

    # Get the position error
    pos_err = pos_des - body.xpos

    task_space_err = np.hstack([pos_err, theta_err])

    # Get the velocity error
    vel_curr = body.cvel
    vel_err = vel_des - vel_curr

    # Get the Jacobian at the desired location on the robot

    # This function works by taking in return parameters!!! Make sure you supply it with placeholder
    # variables
    body_id = body.id
    point = body.xpos
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    mj.mj_jac(model, data, jacp, jacr, point, body_id)

    jac = np.vstack((jacp, jacr))

    # Compute the impedance control input torques
    tau = jac.T @ (Kp * task_space_err + Kd * vel_err)

    # Set the control inputs

    data.ctrl[:7] = tau[:7] + data.qfrc_bias[:7]

    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION

    # Update force sensor readings
    force[:] = np.roll(force, -1)[:]
    force[-1] = data.sensordata[2]

    # Update control inputs to the whiteboard
    get_board_control(model, data)

def position_control(model, data):

    # Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # Set the desired joint angle positions
    desired_joint_positions = np.array(
        [0, 0, 0, -1.57079, 0, 1.57079, -0.7853])

    # Set the desired joint velocities
    desired_joint_velocities = np.array([0, 0, 0, 0, 0, 0, 0])

    # Desired gain on position error (K_p)
    Kp = 1000

    # Desired gain on velocity error (K_d)
    Kd = 1000

    # Set the actuator control torques
    data.ctrl[:7] = data.qfrc_bias[:7] + Kp * \
        (desired_joint_positions-data.qpos[:7]) + Kd * \
        (np.array([0, 0, 0, 0, 0, 0, 0])-data.qvel[:7])


####################################### MAIN #####################################
if __name__ == "__main__":
    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)

    # Set the simulation scene to the home configuration
    mj.mj_resetDataKeyframe(model, data, 0)

    ################################# Swap Callback Below This Line #################################
    # This is where you can set the control callback. Take a look at the Mujoco documentation for more
    # details. Very briefly, at every timestep, a user-defined callback function can be provided to
    # mujoco that sets the control inputs to the actuator elements in the model. The gravity
    # compensation callback has been implemented for you. Run the file and play with the model as
    # explained in the PDF

    mj.set_mjcb_control(impedance_control)  # TODO:

    ################################# Swap Callback Above This Line #################################

    # Initialize variables to store force and time data points
    force_sensor_max_time = 10
    force = np.zeros(int(force_sensor_max_time/model.opt.timestep))
    time = np.linspace(0, force_sensor_max_time, int(
        force_sensor_max_time/model.opt.timestep))

    # Launch the simulate viewer
    viewer.launch(model, data)

    # Save recorded force and time points as a csv file
    force = np.reshape(force, (5000, 1))
    time = np.reshape(time, (5000, 1))
    plot = np.concatenate((time, force), axis=1)
    # np.savetxt('force_vs_time_force_ctrl_sine_init.csv', plot, delimiter=',')
    # np.savetxt('force_vs_time_impedance_ctrl_sine_init.csv', plot, delimiter=',')
    # np.savetxt('force_vs_time_force_ctrl_sine_stable.csv', plot, delimiter=',')
    np.savetxt('force_vs_time_impedance_ctrl_sine_stable.csv', plot, delimiter=',')
