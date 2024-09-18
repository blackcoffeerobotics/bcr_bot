#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
class AMCLWrapper(Node):

    def __init__(self):
        super().__init__("amcl_wrapper_node")
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)
        
    def amcl_callback(self,msg):
        covariance_matrix = msg.pose.covariance

        confidence = self.calculate_confidence(covariance_matrix)
        variance_x = covariance_matrix[0]  # Variance in the x direction
        variance_y = covariance_matrix[7]  # Variance in the y direction

        # Calculate standard deviations
        std_dev_x = math.sqrt(variance_x)
        std_dev_y = math.sqrt(variance_y)

        # Log results
        self.get_logger().info(f'Standard Deviation in x: {std_dev_x}')
        self.get_logger().info(f'Standard Deviation in y: {std_dev_y}')
        self.get_logger().info(f"Localization Confidence: {confidence:.2f}%")

    
    def calculate_confidence(self, covariance_matrix):
        covariance_matrix = np.array(covariance_matrix).reshape(6,6)
        pose_cov_matrix = covariance_matrix[:2,:2]
        det = np.linalg.det(pose_cov_matrix)
        max_uncertainty = 1.0

        confidence = max(0.0, 100*(1-det/max_uncertainty))

        return confidence

def main(args=None):
    rclpy.init(args=args)
    
    localization_confidence = AMCLWrapper()
    rclpy.spin(localization_confidence)
    rclpy.shutdown()

if __name__ == '__main__':
    main()