import rclpy
from rclpy.node import Node
from ur3e_mrc.msg import PositionUR3e, CommandUR3e
import sys
import math
import numpy as np
from enme480_lab_fk.submodules.kinematic_functions import KinematicFunctions

class ForwardKinematicsUR3e(Node):

    def __init__(self):
        super().__init__('ur3e_fk_publisher')
        self.publisher_ = self.create_publisher(CommandUR3e, '/ur3/command', 10)

        # Initialize with zeros or your desired values
        #self.send_command()

    def send_command(self, joint_positions):
        
        for angle in joint_positions:
            if abs(angle) > 181:
                joint_positions = [0.0]*6

        if joint_positions[1] > 5:
            self.get_logger().info(f'z is going below the workbench. Resetting to Zero configuration')


        ur3e_matrix = KinematicFunctions().correct_calculate_dh_transform(joint_positions)
        if ur3e_matrix[2,3] < 0.01:
            self.get_logger().info(f'z is going below the workbench. Resetting to Zero configuration')
            joint_positions = [0.0]*6  

        ######################################### YOUR CODE STARTS ########################################

        # Convert angles to radians


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



def main(args=None):
    rclpy.init(args=args)

    # Check command-line arguments
    if len(sys.argv) != 7:
        print("Usage: ros2 run <package_name> <script_name> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6>")
        return

    try:
        # Read joint positions from command-line arguments
        joint_positions = [float(arg) for arg in sys.argv[1:7]]
    except ValueError:
        print("All joint positions must be numbers.")
        return

    node = ForwardKinematicsUR3e()

    try:
    	node.send_command(joint_positions)  # Send the command with updated joint positions
        #rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
    	node.destroy_node()
    	rclpy.shutdown()

    

if __name__ == '__main__':
    main()