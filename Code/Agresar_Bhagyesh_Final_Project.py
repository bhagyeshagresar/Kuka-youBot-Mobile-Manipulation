#Import the required libraries
import modern_robotics as mr
import numpy as np
from math import sin, cos, pi
import matplotlib.pyplot as plt
import logging

logging.basicConfig(filename="runscript.log", level=logging.INFO)


logging.info("Generating Reference Trajectory")


#Initial End Effector Configuration wrt Space Frame
Tse_initial = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])


#Initial Configuration of Cube wrt Space Frame
Tsc_initial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])

#Final Configuration of Cube wrt Space Frame
Tsc_goal = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])


theta = pi/2 + pi/6
#rotated end effector standoff configuration wrt cube frame at Standoff and Grasping Location
Tce_standoff = np.array([[cos(theta), 0, sin(theta), 0], [0, 1, 0, 0], [-sin(theta), 0, cos(theta), 0.3], [0, 0, 0, 1]])
Tce_grasp = np.array([[cos(theta), 0, sin(theta), 0], [0, 1, 0, 0], [-sin(theta), 0, cos(theta), 0], [0, 0, 0, 1]])
k =1

#TrajectoryGenerator - Milestone 2
def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k):

        """

        :param Tse_initial:  Initial End Effector Configuration wrt Space Frame
        :param Tsc_initial: Initial Configuration of Cube wrt Space Frame
        :param Tsc_goal: Final Configuration of Cube wrt Space Frame
        :param Tce_grasp: end effector standoff configuration wrt cube frame at Grasping Location
        :param Tce_standoff: end effector standoff configuration wrt cube frame at standoff Location
        :param k: The number of trajectory reference configurations per 0.01 seconds
        :return: stack : Nx13 matrix where each row represents the configuration of the end effector frame wrt space frame at every instant in time
        """

        # End Effector Configuration wrt space Frame at standoff location for initial position of cube
        Tse_standoff = Tsc_initial @ Tce_standoff

        # End Effector Configuration wrt space frame at Grasping Location for initial position of cube
        Tse_grasp = Tsc_initial @ Tce_grasp

        # End Effector Configuration wrt space frame at Standoff location for final position of cube
        Tse_standoff_final = Tsc_goal @ Tce_standoff

        # End Effector Configuration wrt space frame at Grasping location for final position of cube
        Tse_grasp_final = Tsc_goal @ Tce_grasp

        #Time taken for the gripper to follow each segment and
        Tf1 = 5
        Tf2 = 3
        Tf3 = 1
        Tf4 = 3
        Tf5 = 5
        Tf6 = 3
        Tf7 = 1
        Tf8 = 3
        N1 = int(((Tf1*k)/0.01))
        N2 = int(((Tf2*k)/0.01))
        N3 = int(((Tf3*k)/0.01))
        N4 = int(((Tf4*k)/0.01))
        N5 = int(((Tf5*k)/0.01))
        N6 = int(((Tf6*k)/0.01))
        N7 = int(((Tf7*k)/0.01))
        N8 = int(((Tf8*k)/0.01))



        """
        Generate Trajectory for each segment using ScrewTrajectory Function from Modern Robotics library
        """


        # segment 1
        traj1 = mr.ScrewTrajectory(Tse_initial, Tse_standoff, Tf= Tf1, N = N1, method=5)  # stage1
        gripper_state = [0, 1]
        traj_list1 = np.zeros((N1, 13), dtype=float)

        for j in range(N1):
                traj_list1[j][0] = traj1[j][0][0]
                traj_list1[j][1] = traj1[j][0][1]
                traj_list1[j][2] = traj1[j][0][2]
                traj_list1[j][3] = traj1[j][1][0]
                traj_list1[j][4] = traj1[j][1][1]
                traj_list1[j][5] = traj1[j][1][2]
                traj_list1[j][6] = traj1[j][2][0]
                traj_list1[j][7] = traj1[j][2][1]
                traj_list1[j][8] = traj1[j][2][2]
                traj_list1[j][9] = traj1[j][0][3]
                traj_list1[j][10] = traj1[j][1][3]
                traj_list1[j][11] = traj1[j][2][3]
                traj_list1[j][12] = gripper_state[0]

        #Segment 2

        traj2 = mr.ScrewTrajectory(Tse_standoff, Tse_grasp, Tf= Tf2, N = N2, method=5)
        gripper_state = [0, 1]
        traj_list2 = np.zeros((N2, 13), dtype=float)

        for j in range(N2):
                traj_list2[j][0] = traj2[j][0][0]
                traj_list2[j][1] = traj2[j][0][1]
                traj_list2[j][2] = traj2[j][0][2]
                traj_list2[j][3] = traj2[j][1][0]
                traj_list2[j][4] = traj2[j][1][1]
                traj_list2[j][5] = traj2[j][1][2]
                traj_list2[j][6] = traj2[j][2][0]
                traj_list2[j][7] = traj2[j][2][1]
                traj_list2[j][8] = traj2[j][2][2]
                traj_list2[j][9] = traj2[j][0][3]
                traj_list2[j][10] = traj2[j][1][3]
                traj_list2[j][11] = traj2[j][2][3]
                traj_list2[j][12] = gripper_state[0]

        # Segment 3

        traj3 = mr.ScrewTrajectory(Tse_grasp, Tse_grasp, Tf=Tf3, N=N3, method=5)
        gripper_state = [0, 1]

        traj_list3 = np.zeros((N3, 13), dtype=float)

        for j in range(N3):
                traj_list3[j][0] = traj3[j][0][0]
                traj_list3[j][1] = traj3[j][0][1]
                traj_list3[j][2] = traj3[j][0][2]
                traj_list3[j][3] = traj3[j][1][0]
                traj_list3[j][4] = traj3[j][1][1]
                traj_list3[j][5] = traj3[j][1][2]
                traj_list3[j][6] = traj3[j][2][0]
                traj_list3[j][7] = traj3[j][2][1]
                traj_list3[j][8] = traj3[j][2][2]
                traj_list3[j][9] = traj3[j][0][3]
                traj_list3[j][10] = traj3[j][1][3]
                traj_list3[j][11] = traj3[j][2][3]
                traj_list3[j][12] = gripper_state[1]

        # Segment 4


        traj4 = mr.ScrewTrajectory(Tse_grasp, Tse_standoff, Tf=Tf4, N=N4, method=5)
        gripper_state = [0, 1]
        traj_list4 = np.zeros((N4, 13), dtype=float)
        for j in range(N4):
                traj_list4[j][0] = traj4[j][0][0]
                traj_list4[j][1] = traj4[j][0][1]
                traj_list4[j][2] = traj4[j][0][2]
                traj_list4[j][3] = traj4[j][1][0]
                traj_list4[j][4] = traj4[j][1][1]
                traj_list4[j][5] = traj4[j][1][2]
                traj_list4[j][6] = traj4[j][2][0]
                traj_list4[j][7] = traj4[j][2][1]
                traj_list4[j][8] = traj4[j][2][2]
                traj_list4[j][9] = traj4[j][0][3]
                traj_list4[j][10] = traj4[j][1][3]
                traj_list4[j][11] = traj4[j][2][3]
                traj_list4[j][12] = gripper_state[1]


        # Segment 5



        traj5 = mr.ScrewTrajectory(Tse_standoff, Tse_standoff_final, Tf=Tf5, N=N5, method=5)
        gripper_state = [0, 1]
        traj_list5 = np.zeros((N5, 13), dtype=float)

        for j in range(N5):
                traj_list5[j][0] = traj5[j][0][0]
                traj_list5[j][1] = traj5[j][0][1]
                traj_list5[j][2] = traj5[j][0][2]
                traj_list5[j][3] = traj5[j][1][0]
                traj_list5[j][4] = traj5[j][1][1]
                traj_list5[j][5] = traj5[j][1][2]
                traj_list5[j][6] = traj5[j][2][0]
                traj_list5[j][7] = traj5[j][2][1]
                traj_list5[j][8] = traj5[j][2][2]
                traj_list5[j][9] = traj5[j][0][3]
                traj_list5[j][10] = traj5[j][1][3]
                traj_list5[j][11] = traj5[j][2][3]
                traj_list5[j][12] = gripper_state[1]

        # Segment 6


        traj6 = mr.ScrewTrajectory(Tse_standoff_final, Tse_grasp_final, Tf=Tf6, N=N6, method=5)
        gripper_state = [0, 1]
        traj_list6 = np.zeros((N6, 13), dtype=float)

        for j in range(N6):
                traj_list6[j][0] = traj6[j][0][0]
                traj_list6[j][1] = traj6[j][0][1]
                traj_list6[j][2] = traj6[j][0][2]
                traj_list6[j][3] = traj6[j][1][0]
                traj_list6[j][4] = traj6[j][1][1]
                traj_list6[j][5] = traj6[j][1][2]
                traj_list6[j][6] = traj6[j][2][0]
                traj_list6[j][7] = traj6[j][2][1]
                traj_list6[j][8] = traj6[j][2][2]
                traj_list6[j][9] = traj6[j][0][3]
                traj_list6[j][10] = traj6[j][1][3]
                traj_list6[j][11] = traj6[j][2][3]
                traj_list6[j][12] = gripper_state[1]


        #Segment 7
        traj7 = mr.ScrewTrajectory(Tse_grasp_final, Tse_grasp_final, Tf=Tf7, N=N7, method=5)

        gripper_state = [0, 1]

        traj_list7 = np.zeros((N7, 13), dtype=float)

        for j in range(N7):
                traj_list7[j][0] = traj7[j][0][0]
                traj_list7[j][1] = traj7[j][0][1]
                traj_list7[j][2] = traj7[j][0][2]
                traj_list7[j][3] = traj7[j][1][0]
                traj_list7[j][4] = traj7[j][1][1]
                traj_list7[j][5] = traj7[j][1][2]
                traj_list7[j][6] = traj7[j][2][0]
                traj_list7[j][7] = traj7[j][2][1]
                traj_list7[j][8] = traj7[j][2][2]
                traj_list7[j][9] = traj7[j][0][3]
                traj_list7[j][10] = traj7[j][1][3]
                traj_list7[j][11] = traj7[j][2][3]
                traj_list7[j][12] = gripper_state[0]

        #Segment 8
        traj8 = mr.ScrewTrajectory(Tse_grasp_final, Tse_standoff_final, Tf=Tf8, N=N8, method=5)

        gripper_state = [0, 1]

        traj_list8 = np.zeros((N8, 13), dtype=float)

        for j in range(N8):
                traj_list8[j][0] = traj8[j][0][0]
                traj_list8[j][1] = traj8[j][0][1]
                traj_list8[j][2] = traj8[j][0][2]
                traj_list8[j][3] = traj8[j][1][0]
                traj_list8[j][4] = traj8[j][1][1]
                traj_list8[j][5] = traj8[j][1][2]
                traj_list8[j][6] = traj8[j][2][0]
                traj_list8[j][7] = traj8[j][2][1]
                traj_list8[j][8] = traj8[j][2][2]
                traj_list8[j][9] = traj8[j][0][3]
                traj_list8[j][10] = traj8[j][1][3]
                traj_list8[j][11] = traj8[j][2][3]
                traj_list8[j][12] = gripper_state[0]


        """
        Stack all the configurations in a single array 'stack'
        """

        stack = np.vstack((traj_list1, traj_list2, traj_list3, traj_list4, traj_list5, traj_list6, traj_list7, traj_list8))



        return stack


trajectory = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k = 1)

logging.info("saving the reference trajectory")
"""
Save the array in 'reference_trajectory.csv' file
"""
np.savetxt("reference_trajectory.csv", trajectory, delimiter=",")

#Milestone 3
def FeedbackControl(X, Xd, Xd_next, kp, ki, time_step):
    """
    Function to calculate the kinematic task-space feedforward plus feedback control law

    :param X: current actual end-effector configuration
    :param Xd: current end-effector reference configuration
    :param Xd_next: end-effector reference configuration at the next timestep in the reference trajectory
    :param kp: Proportional Gain Kp
    :param ki: Integral Gain Ki
    :param time_step: The time between the reference trajectory configurations
    :return:
    V_t: Commanded End-Effector twist V_t expressed in end-effector frame
    V_d: Feedforward reference twist
    Xerr: 6 vector that represents error twist that takes X to Xd in unit time
    """
    t1 = mr.TransInv(X)
    t2 = Xd
    T = t1 @ t2
    Xerr = mr.se3ToVec(mr.MatrixLog6(T)) # Calculate Xerr

    V_d = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Xd) @ Xd_next) / time_step)
    T1 = mr.TransInv(X) @ Xd
    V_t = np.dot(mr.Adjoint(T1), V_d) + (kp @ Xerr) + ((ki @ Xerr) * time_step) #Feedforward plus Feedback control law

    return V_t, V_d, Xerr


# Milestone 1
def Nextstate(current_configuration, control_variables, time_step, speed_limit):
    """
    Function to calculate configuration of the robot time_step later.

    :param current_configuration: A vector of 12 values representing the current configuration of the robot
    :param control_variables: A vector of 9 values. 4 values represent wheel speeds and remaining 5 values remaining joint speeds
    :param time_step: Time between each configuration of the robot in secs (s)
    :param speed_limit: Maximum angular speed of the arm joints and wheels (rad/s)
    :return:
    new_config : A vector of New Configuration of the robot representing the 12 values i.e. 3 values representing chassis position and orientation,
    5 values representing joint angles and 4 values representing wheel angles
    """

    # Account for speed limit
    for i in range(len(control_variables)):
        if control_variables[i] > speed_limit:
            control_variables[i] = speed_limit
        elif control_variables[i] < -speed_limit:
            control_variables[i] = -speed_limit

    # current_configuration = [phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4]
    # Get current configurations of the arm and wheels from the 12 vector of current_configurations
    current_chassis_configuration = current_configuration[0:3]
    current_arm_angles = current_configuration[3:8]
    current_wheel_angles = current_configuration[8:]

    current_joint_speeds = control_variables[:5]
    current_wheel_speeds = control_variables[5:]


    # Get new arm angles and new wheel angles from current angles and current speeds using Euler Step
    new_arm_angles = current_arm_angles + (current_joint_speeds*time_step)
    new_wheel_angles = current_wheel_angles + (current_wheel_speeds*time_step)


    #Odometry

    l = 0.47/2
    r = 0.0475
    w = 0.3/2

    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])

    twist_vb = F @ current_wheel_speeds # Twist Vb
    twist_vb = time_step * twist_vb



    twist_vb6 = np.array([0, 0, twist_vb[0], twist_vb[1], twist_vb[2], 0]) # Twist Vb6




    if twist_vb6[2] == 0:
        delta_qb = np.array([0, twist_vb6[3], twist_vb6[4]]) # delta_qb represents change in coordinates relative to body frame {b}

    else:
        delta_qb = np.array([twist_vb6[2],
                             (twist_vb6[3]*sin(twist_vb6[2]) + twist_vb6[4]*(cos(twist_vb6[2]) - 1))/twist_vb6[2],
                             (twist_vb6[4]*sin(twist_vb6[2]) + twist_vb6[3]*(1-cos(twist_vb6[2])))/twist_vb6[2]])


    chassis_matrix = np.array([[1, 0, 0], [0, cos(current_chassis_configuration[0]), -sin(current_chassis_configuration[0])],
                               [0, sin(current_chassis_configuration[0]), cos(current_chassis_configuration[0])]])


    #Transforming delta_qb in {b} to delta_q in fixed frame {s}
    delta_q = chassis_matrix @ delta_qb

    # Updated Odometry of the chassis configuration
    new_chassis_configuration = current_chassis_configuration + delta_q

    # New configuration of the robot
    new_config = np.concatenate((new_chassis_configuration, new_arm_angles, new_wheel_angles), axis=None)



    return new_config




logging.info("Specifying all inputs")

# current_config = [phi (rad), x (m), y (m), j1, j2, j3, j4, j5, w1, w2, w3, w4]
#Accounting for the 30 degrees orientation error and 0.2 m position error
current_config = np.array([0.5236, 0.2, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0])


#configuration of the frame {b} of the mobile base relative to the frame {s}
T_sb = np.array([[cos(current_config[0]), -sin(current_config[0]), 0, current_config[1]],
               [sin(current_config[0]), cos(current_config[0]), 0, current_config[2]],
               [0, 0, 1, 0.0963],
               [0, 0, 0, 1]])

#Offset from the chassis frame {b} to base of the frame of the arm {0}
T_b0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026],
                                                   [0, 0, 0, 1]])


#Home configuration of the end effector relative to the base frame of the arm {0}
M_0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])

# List of all joint screw axes
Blist = np.array([[0, 0, 0, 0, 0], [0, -1, -1, -1, 0], [1, 0, 0, 0, 1], [0, -0.5076, -0.3526, -0.2176, 0],
                 [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])


# List of joint angles of the arm
thetalist = current_config[3:8]
time_step = 0.01
speed_limit = 1000
logging.info("Enter kp and ki")
x = float(input("Enter value for kp: "))
y = float(input("Enter value for ki: "))
kp = x*np.identity(6) # Proportinal Gain
ki = y*np.identity(6) # Integral Gain

r = 0.0475
l = 0.47 / 2
w = 0.3 / 2

F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                        [1, 1, 1, 1],
                        [-1, 1, -1, 1]])

F6 = np.zeros((6,4))
F6[2:5, :] = F


stack = []
Xerr_list = []
time_step_list = []

logging.info("Loop over the reference trajectory")
for i in range(len(trajectory) - 1):
        Xd = np.array([[trajectory[i, 0], trajectory[i,1], trajectory[i, 2], trajectory[i, 9]],
                        [trajectory[i, 3], trajectory[i, 4], trajectory[i, 5], trajectory[i, 10]],
                       [trajectory[i, 6], trajectory[i, 7], trajectory[i, 8], trajectory[i, 11]],
                       [0, 0, 0, 1]])
        Xd_next = np.array([[trajectory[i+1, 0], trajectory[i+1,1], trajectory[i+1, 2], trajectory[i+1, 9]],
                        [trajectory[i+1, 3], trajectory[i+1, 4], trajectory[i+1, 5], trajectory[i+1, 10]],
                       [trajectory[i+1, 6], trajectory[i+1, 7], trajectory[i+1, 8], trajectory[i+1, 11]],
                       [0, 0, 0, 1]])

        # configuration of the frame {b} of the mobile base relative to the frame {s}
        T_sb = np.array([[cos(current_config[0]), -sin(current_config[0]), 0, current_config[1]],
                         [sin(current_config[0]), cos(current_config[0]), 0, current_config[2]],
                         [0, 0, 1, 0.0963],
                         [0, 0, 0, 1]])
        thetalist = current_config[3:8]

        # Configuration of the end effector frame {e} relative to the base frame of the arm {0}
        T_0e = mr.FKinBody(M_0e, Blist, thetalist)

        X = T_sb @ T_b0 @ T_0e


        #Calculating Twist V_t
        V_t, _, Xerr= FeedbackControl(X, Xd, Xd_next, kp, ki, time_step)

        #Calculating Pseudoinverse of Jacobian Je
        J_base = mr.Adjoint(np.dot(mr.TransInv(T_0e), mr.TransInv(T_b0))) @ F6
        J_arm = mr.JacobianBody(Blist, thetalist)
        Je = np.hstack((J_base, J_arm))
        J_inv = np.linalg.pinv(Je)

        #Calculating control variables
        velocities = np.dot(J_inv, V_t)

        #velocities = [u1 (rad/s), u2 (rad/s), u3 (rad/s), u4 (rad/s), thetadot1 (rad/s), thetadot2 (rad/s), thetadot3 (rad/s), thetadot4 (rad/s), thetadot5 (rad/s)]
        #control_variables = [thetado1, thetadot2, thetadot3, thetadot4, thetadot5, u1, u2, u3, u4]
        #Getting velocities in correct order for Nextstate function
        wheel_velocities = velocities[0:4]
        arm_velocities = velocities[4:]
        control_variables = np.concatenate((arm_velocities, wheel_velocities))

        # new_chassis_config = Nextstate(current_config, control_variables, time_step, speed_limit)


        new_config = Nextstate(current_config, control_variables, time_step, speed_limit)

        # Append Gripper state to the configuration of the robot
        if trajectory[i, 12] == 1:
            new_chassis_config = np.append(new_config, 1)
        else:
            new_chassis_config = np.append(new_config, 0)

        # Stack every ith Configuration
        stack.append(new_chassis_config)

        #Update the configuration of the robot
        current_config = new_config

        #Store every ith 6 vector error
        Xerr_list.append(Xerr)

        time_step_list.append(i)


logging.info("saving the animation file and the error list")
"""
Save the final trajector 'stack' in a csv file
"""
np.savetxt("final_trajectory.csv", stack, delimiter=",")
np.savetxt("error_list.csv", Xerr_list, delimiter=",")




n = len(time_step_list)
Xerr_array = np.array(Xerr_list)


logging.info("plotting the evolution of the error over time")
plt.figure()
t = np.linspace(0,n,n)


plt.plot(t, Xerr_array[:,0], label="wx")
plt.plot(t, Xerr_array[:,1], label="wy")
plt.plot(t, Xerr_array[:,2], label="wz")
plt.plot(t, Xerr_array[:,3], label="vx")
plt.plot(t, Xerr_array[:,4], label="vy")
plt.plot(t, Xerr_array[:,5], label="vz")
plt.xlabel("Number of Iterations")
plt.ylabel("Error Twist (rad/s, m/s)")
plt.title("Plotting the graph of error over time")
plt.legend()
plt.show()

logging.info("finished")





