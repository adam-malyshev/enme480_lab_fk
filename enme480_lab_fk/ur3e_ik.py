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


        
        # Publish the command



        ######################################### END OF YOUR CODE ########################################
        ### DO NOT CHANGE ###

        # Calculate DH transformation
        dh_matrix = self.calculate_dh_transform(joint_positions)

        print("--------------------- \n Your Laser Prediction \n---------------------")
        KinematicFunctions().predict_laser_position(dh_matrix)

    
    
    def calculate_dh_transform(self, joint_positions):


        ################## YOUR CODE STARTS HERE ###################################
        # Calculate the FINAL TRANSFORMATION MATRIX here using DH parameters


        ################################ YOUR CODE ENDS HERE #########################
        self.get_logger().info(f'Your DH Transformation Matrix:\n{transform}')

        return transform

    def inverse_kinematics(self, xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
        
        return_value = np.array([0, 0, 0, 0, 0, 0])

        # Angles returned should be in radians

        #############################3 Your Code Starts Here ###################

        # Step 1: find gripper position relative to the base of UR3,
        # and set theta_5 equal to -pi/2

        # Step 2: find x_cen, y_cen, z_cen

        # Step 3: find theta_1

        # Step 4: find theta_6 

        # Step 5: find x3_end, y3_end, z3_end

        # Step 6: find theta_2, theta_3, theta_4

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