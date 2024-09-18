#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy as np
import open3d as o3d
import tf2_ros
from tf_transformations import quaternion_matrix
import tf_transformations

class ScanMapICP(Node):

    def __init__(self):
        super().__init__('scan_map_icp')

        # Parameters
        self.declare_parameter('base_laser_frame', 'two_d_lidar')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('use_sim_time', True)

        self.base_laser_frame = self.get_parameter('base_laser_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.transformation = np.eye(4)

        # ROS Publishers and Subscribers
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/bcr_bot/scan', self.laser_scan_callback, 1)

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 1)
        # self.call_icp_sub = self.create_subscription(
        #     String, 'call_icp', self.call_icp_callback, 1)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initial_pose', 1)
        # self.map_to_cloud_point_pub = self.create_publisher(
        #     PointCloud2, 'map_point', 1)
        # self.scan_to_cloud_point_pub = self.create_publisher(
        #     PointCloud2, 'scan_point', 1)

        # Point Cloud and Transform Storage
        self.map_cloud =  o3d.io.read_point_cloud("/home/devyani.g/ros2_ws/src/bcr_bot/config/data.ply")
        self.scan_cloud = o3d.geometry.PointCloud()
        self.listener = tf2_ros.TransformListener(tf2_ros.Buffer())

        self.get_logger().info('ScanMapICP node has been started')

    # def map_callback(self, msg):
    #     """Convert OccupancyGrid to a PointCloud for map."""
    #     self.map_cloud.clear()

    #     resolution = msg.info.resolution
    #     width = msg.info.width
    #     height = msg.info.height

    #     origin_x = msg.info.origin.position.x
    #     origin_y = msg.info.origin.position.y

    #     points = []
    #     for y in range(height):
    #         for x in range(width):
    #             if msg.data[x + y * width] == 100:  # Occupied cells
    #                 points.append([
    #                     (x + 0.5) * resolution + origin_x,
    #                     (y + 0.5) * resolution + origin_y,
    #                     0.0
    #                 ])
        
    #     self.map_cloud.points = o3d.utility.Vector3dVector(np.array(points))

    #     # Publish the map point cloud as PointCloud2
    #     map_pc2 = self.create_pointcloud2(self.map_cloud)
    #     self.map_to_cloud_point_pub.publish(map_pc2)
    #     self.get_logger().info('Map converted to point cloud')
    def amcl_pose_callback(self, msg):
        position  = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        translation = np.array([position.x, position.y, position.z])
        quaternion = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

        rotation_matrix = quaternion_matrix(quaternion)

        self.transformation[:3, :3] = rotation_matrix[:3, :3]
        self.transformation[:3, 3] = translation


    def laser_scan_callback(self, msg):
        """Convert LaserScan data to PointCloud and transform to map frame."""
        scan_time = msg.header.stamp

        # Try to get the transformation from the laser to the base frame
        try:
            trans = self.listener.lookup_transform(self.odom_frame, self.base_laser_frame, scan_time)
            # self.get_logger().info(f"Transform available from {self.base_laser_frame} to {self.map_frame}")

        except:
            self.get_logger().warn('Could not transform from base to laser frame')
            return

        points = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y, 0.0])

        transformed_points = []
        for p in points:
            transformed_point = self.transform_point(p, trans)
            transformed_points.append(transformed_point)

        self.scan_cloud.points = o3d.utility.Vector3dVector(np.array(points))
        self.run_icp()

    # def call_icp_callback(self, msg):
    #     """Run ICP when triggered by an external topic."""
    #     self.run_icp()

    def transform_point(self, point, transform):
        """Transform a point from one frame to another using TF."""
        # Extract translation and rotation from the transform
        translation = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]
        rotation = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]

        # Use tf transformations to apply the transform to the point
        point_homogeneous = np.array([point[0], point[1], point[2], 1.0])  # Make it homogeneous
        translation_matrix = tf_transformations.translation_matrix(translation)
        rotation_matrix = tf_transformations.quaternion_matrix(rotation)
        transform_matrix = np.dot(translation_matrix, rotation_matrix)

        transformed_point_homogeneous = np.dot(transform_matrix, point_homogeneous)
        transformed_point = transformed_point_homogeneous[:3]  # Back to Cartesian coordinates
        return transformed_point


    def run_icp(self):
        """Perform ICP alignment between scan and map point clouds."""
        if len(self.scan_cloud.points) == 0 or len(self.map_cloud.points) == 0:
            self.get_logger().warn('Missing map or scan data for ICP')
            return

        # ICP in Open3D
        icp_result = o3d.pipelines.registration.registration_icp(
            self.scan_cloud, self.map_cloud, 0.5,
            self.transformation,  # Initial transformation
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        if not icp_result.fitness:
            self.get_logger().warn('ICP did not converge')
            return

        self.get_logger().info(f'ICP converged with score: {icp_result.fitness}')

        # Extract the transformation matrix
        new_transformation = icp_result.transformation

        # Publish new pose if successful
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = new_transformation[0, 3]
        pose.pose.pose.position.y = new_transformation[1, 3]
        self.initial_pose_pub.publish(pose)
        self.get_logger().info('Updated robot pose published')


def main(args=None):
    rclpy.init(args=args)
    node = ScanMapICP()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
