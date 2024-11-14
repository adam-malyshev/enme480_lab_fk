import rclpy
from rclpy.node import Node
from ur3e_mrc.msg import PositionUR3e, CommandUR3e
import sys
import math
import numpy as np
from enme480_lab_fk.submodules.kinematic_functions import KinematicFunctions

class InverseKinematicsUR3e(Node):

    def __init__(self):
        super().__init__('ur3e_ik_publisher')
        self.publisher_ = self.create_publisher(CommandUR3e, '/ur3/command', 10)

        # Initialize with zeros or your desired values
        #self.send_command()

    def send_command(self, joint_positions):

        # We are not coverting the angles to radians here since your IK script sends the output in radians

        for angle in joint_positions:
            if abs(angle) > 3.15:
                joint_positions = [0.0]*6
                print(f'Joint Angle is greater than pi')

        if joint_positions[1] > 0.0873:
            self.get_logger().info(f'z is going below the workbench. Resetting to Zero configuration')


        ur3e_matrix = KinematicFunctions().correct_calculate_dh_transform(joint_positions)
        if ur3e_matrix[2,3] < 0.01:
            self.get_logger().info(f'z is going below the workbench. Resetting to Zero configuration')
            joint_positions = [0.0]*6  

        ######################################### YOUR CODE STARTS ########################################

        # Create a CommandUR3e message
        # SET v AND a TO BE === 0.1

        msg = CommandUR3e(destination = joint_positions, v= 0.1,a = 0.1,io_0 = False)
        
        # Publish the command
        self.publisher_.publish(msg)



        ######################################### END OF YOUR CODE ########################################
        ### DO NOT CHANGE ###

        # Calculate DH transformation
        dh_matrix = self.calculate_dh_transform(joint_positions)

        print("--------------------- \n Your Laser Prediction \n---------------------")
        KinematicFunctions().predict_laser_position(dh_matrix)

    
    
    def calculate_dh_transform(self, joint_positions):
        L1 = 0.15185
        L2 = 0.120
        L3 = 0.24355
        L4 = 0.093
        L5 = 0.2132
        L6 = 0.104
        L7 = 0.08535
        L8 = 0.0921
        L9 = 0.0535
        L10 = 0.052


        ################## YOUR CODE STARTS HERE ###################################
        # Calculate the FINAL TRANSFORMATION MATRIX here using DH parameters
        # Calculate the FINAL TRANSFORMATION MATRIX here using DH parameters
        A0 = np.array([[1, 0, 0, -0.15], [0, 1, 0, 0.15], [0, 0, 1, 0.01], [0, 0, 0, 1]])
        A1 = self.get_a_matrix(0, -np.pi/2, L1, joint_positions[0])
        A2 = self.get_a_matrix(L3, 0, 0, joint_positions[1])
        A3 = self.get_a_matrix(L5, 0, 0, joint_positions[2])
        A4 = self.get_a_matrix(0, np.pi/2, 0.13105, joint_positions[3] + np.pi/2)
        A5 = self.get_a_matrix(0, -np.pi/2, L7, joint_positions[4])
        A6 = self.get_a_matrix(0, 0, L8, joint_positions[5])
        A7 = self.get_a_matrix(L9, 0, L10, np.pi)
        transform = A0@A1@A2@A3@A4@A5@A6@A7


        ################################ YOUR CODE ENDS HERE #########################
        self.get_logger().info(f'Your DH Transformation Matrix:\n{transform}')

        return transform
    def get_a_matrix(self, r, alpha, d, theta):
        return  np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),   np.sin(theta)*np.sin(alpha),    r*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha),    -np.cos(theta)*np.sin(alpha),   r*np.sin(theta)], 
            [0          ,   np.sin(alpha)              ,    np.cos(alpha),                  d],
            [0          ,   0                           ,   0                           ,   1]
        ])

    def rot_z(self, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

    def inverse_kinematics(self, xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
        
        return_value = np.array([0, 0, 0, 0, 0, 0])

        # Angles returned should be in radians

        ############################# Your Code Starts Here ###################
        
        #all lengths in meters
        #all angles in radians

        L1 = 0.15185
        L2 = 0.120
        L3 = 0.24355
        L4 = 0.093
        L5 = 0.2132
        L6 = 0.104
        L7 = 0.08535
        L8 = 0.0921
        L9 = 0.0535
        L10 = 0.052

        # Step 1: find gripper position relative to the base of UR3,
        # and set theta_5 equal to -pi/2
        
        xgrip = xWgrip + 0.15
        ygrip = yWgrip - 0.15
        zgrip = zWgrip - 0.01

        yawgrip = yawWgrip*np.pi/180

        theta_5 = -np.pi/2

        
        # Step 2: find x_cen, y_cen, z_cen
        

        z_cen = zgrip
        x_cen = xgrip - L9*np.cos(yawgrip)
        y_cen = ygrip - L9*np.sin(yawgrip)


        # Step 3: find theta_1
        
        #beta is the angle from the x axis to the vector (x_cen, y_cen)        
        beta = np.arctan2(y_cen, x_cen)

        dy = L2 - L4 + L6
        r = np.sqrt(x_cen**2 + y_cen**2)

        #alpha is the angle from the x axis to vector when theta_1 is 0
        alpha = np.arcsin(dy/r)
        print(alpha*180/np.pi)
        theta_1 = beta - alpha


        # Step 4: find theta_6 

        #yaw + theta_6 = theta_1 + pi/2

        theta_6 = theta_1 + np.pi/2 - yawgrip


        # Step 5: find x3_end, y3_end, z3_end

        R01 = self.rot_z(theta_1)

        cen1 = R01.T @ np.array([x_cen, y_cen, z_cen]).T
        
        _3_end1 = np.array([cen1[0] - L7, cen1[1] - L6 - 0.027, cen1[2] + L8 + L10])
        
        _3_end0 = R01 @ _3_end1.T

        x3_end = _3_end0[0]
        y3_end = _3_end0[1]
        z3_end = _3_end0[2]


        # Step 6: find theta_2, theta_3, theta_4

        # theta_3 - theta_2 = theta_4

        # theta_2, theta_3 can be found same way as two link robot in text
        # where "y" is z3_end - L1 and "x" is r = sqrt(x3_end**2 + y3_end**2)
        

        r = np.sqrt(x3_end**2+y3_end**2)
        
        s = z3_end - L1


        D = (r**2 + s**2 - L3**2 - L5**2)/(2*L3*L5)
        theta_3 = np.arctan2(-np.sqrt(1-D**2), D)
        theta_2 = np.arctan2(s, r) - np.arctan2(L5*np.sin(theta_3), L3+L5*np.cos(theta_3))

        
        theta_3 = -theta_3
        theta_2 = -theta_2

        theta_4 = -(np.abs(theta_3) - np.abs(theta_2))

        ################# Your Code Ends Here #########################

        # printing theta values (in degree) calculated from inverse kinematics
        
        print("Your Joint Angles: ")
        print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
                str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
                str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))

        # obtain return_value from forward kinematics function
        return_value = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

        # Angles returned should be in radians

        return return_value


def main(args=None):
    rclpy.init(args=args)

    # Check command-line arguments
    if len(sys.argv) != 5:
        print("Usage: ros2 run <package_name> <script_name> <x> <y> <z> <Yaw>")
        return

    try:
        # Read joint positions from command-line arguments
        robot_position = [float(arg) for arg in sys.argv[1:5]]
    except ValueError:
        print("All joint positions must be numbers.")
        return
    
    node = InverseKinematicsUR3e()
    joint_positions = node.inverse_kinematics(robot_position[0], robot_position[1], robot_position[2], robot_position[3])
    # print(f'Your Joint Positions: \n {joint_positions}')
    correct_joint_positions = KinematicFunctions().correct_inverse_kinematics(robot_position[0], robot_position[1], robot_position[2], robot_position[3])
    valid = 1
    for i in range(6):
        if abs(joint_positions[i] - correct_joint_positions[i]) > 0.0873:
            valid = 2
            break
        elif abs(joint_positions[i] - correct_joint_positions[i]) > 0.0174533:
            valid = 3
            break
    # print(f'Correct Joint Positions: \n {correct_joint_positions}')
    try:
        if valid == 2:
            print("Error in joint angles from IK is large, consult TA")
            print("Moving to the given location regardless")
            ur3e_matrix = KinematicFunctions().correct_calculate_dh_transform(correct_joint_positions)
            node.send_command(correct_joint_positions)
            print("--------------------- \n Approximately Correct Laser Prediction \n---------------------")
            KinematicFunctions().predict_laser_position(ur3e_matrix)
            print("Error in joint angles from IK is large, check your math")
        elif valid == 3:
            ur3e_matrix = KinematicFunctions().correct_calculate_dh_transform(correct_joint_positions)
            node.send_command(joint_positions)  # Send the command with updated joint positions
            #rclpy.spin(node)
            print("--------------------- \n Approximately Correct Laser Prediction \n---------------------")
            KinematicFunctions().predict_laser_position(ur3e_matrix)
            print("Slight Error in IK calculation, check your calculations")
        elif valid == 1:
            ur3e_matrix = KinematicFunctions().correct_calculate_dh_transform(correct_joint_positions)
            node.send_command(joint_positions)  # Send the command with updated joint positions
            #rclpy.spin(node)
            print("--------------------- \n Approximately Correct Laser Prediction \n---------------------")
            KinematicFunctions().predict_laser_position(ur3e_matrix)
            print(" Finished successfully. Check DH table in case of a large deviation from prediction ")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    

if __name__ == '__main__':
    main()