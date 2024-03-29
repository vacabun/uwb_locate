#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uwb_interfaces.msg import UWBDistanceMatrix
from geometry_msgs.msg import Point
import numpy as np
from sklearn.manifold import MDS


class Locate(Node):
    def __init__(self):
        super().__init__('locate')
        self.publisher_dict = {}

        self.declare_parameter('distance_matrix_topic_name', '/uwbData/px4_2')
        self.distance_matrix_topic_name = self.get_parameter(
            'distance_matrix_topic_name').value
        self.get_logger().info(
            f'UWB Distance Matrix Topic Name: {self.distance_matrix_topic_name}')

        self.declare_parameter(
            'reference_odometry_topic_name_1', '/model/x500_1/odometry')
        self.declare_parameter(
            'reference_odometry_topic_name_2', '/model/x500_2/odometry')
        self.declare_parameter(
            'reference_odometry_topic_name_3', '/model/x500_3/odometry')
        self.declare_parameter(
            'reference_odometry_topic_name_4', '/model/x500_4/odometry')
        self.reference_odometry_topic_name_1 = self.get_parameter(
            'reference_odometry_topic_name_1').value
        self.reference_odometry_topic_name_2 = self.get_parameter(
            'reference_odometry_topic_name_2').value
        self.reference_odometry_topic_name_3 = self.get_parameter(
            'reference_odometry_topic_name_3').value
        self.reference_odometry_topic_name_4 = self.get_parameter(
            'reference_odometry_topic_name_4').value
        self.get_logger().info(
            f'Reference Odometry Topic Name 1: {self.reference_odometry_topic_name_1}')
        self.get_logger().info(
            f'Reference Odometry Topic Name 2: {self.reference_odometry_topic_name_2}')
        self.get_logger().info(
            f'Reference Odometry Topic Name 3: {self.reference_odometry_topic_name_3}')
        self.get_logger().info(
            f'Reference Odometry Topic Name 4: {self.reference_odometry_topic_name_4}')

        self.declare_parameter('reference_id_1', 1)
        self.declare_parameter('reference_id_2', 2)
        self.declare_parameter('reference_id_3', 3)
        self.declare_parameter('reference_id_4', 4)
        self.reference_id_1 = self.get_parameter('reference_id_1').value
        self.reference_id_2 = self.get_parameter('reference_id_2').value
        self.reference_id_3 = self.get_parameter('reference_id_3').value
        self.reference_id_4 = self.get_parameter('reference_id_4').value
        self.get_logger().info(f'Reference ID 1: {self.reference_id_1}')
        self.get_logger().info(f'Reference ID 2: {self.reference_id_2}')
        self.get_logger().info(f'Reference ID 3: {self.reference_id_3}')
        self.get_logger().info(f'Reference ID 4: {self.reference_id_4}')

        self.reference_position_dict = {}

        self.distance_matrix_subscription = self.create_subscription(
            UWBDistanceMatrix,
            self.distance_matrix_topic_name,
            self.distance_matrix_listener_callback,
            10)
        self.distance_matrix_subscription  # prevent unused variable warning

        self.reference_odometry_subscription_1 = self.create_subscription(
            Point,
            self.reference_odometry_topic_name_1,
            self.reference_odometry_listener_callback_1,
            10)
        self.reference_odometry_subscription_2 = self.create_subscription(
            Point,
            self.reference_odometry_topic_name_2,
            self.reference_odometry_listener_callback_2,
            10)
        self.reference_odometry_subscription_3 = self.create_subscription(
            Point,
            self.reference_odometry_topic_name_3,
            self.reference_odometry_listener_callback_3,
            10)
        self.reference_odometry_subscription_4 = self.create_subscription(
            Point,
            self.reference_odometry_topic_name_4,
            self.reference_odometry_listener_callback_4,
            10)

        self.reference_position_dict[self.reference_id_1] = [0.0, 0.0, 0.0]
        self.reference_position_dict[self.reference_id_2] = [0.0, 0.0, 0.0]
        self.reference_position_dict[self.reference_id_3] = [0.0, 0.0, 0.0]
        self.reference_position_dict[self.reference_id_4] = [0.0, 0.0, 0.0]

        self.reference_odometry_subscription_1
        self.reference_odometry_subscription_2
        self.reference_odometry_subscription_3
        self.reference_odometry_subscription_4

    def reference_odometry_listener_callback_1(self, msg):
        self.reference_odometry_listener_handler(msg, self.reference_id_1)

    def reference_odometry_listener_callback_2(self, msg):
        self.reference_odometry_listener_handler(msg, self.reference_id_2)

    def reference_odometry_listener_callback_3(self, msg):
        self.reference_odometry_listener_handler(msg, self.reference_id_3)

    def reference_odometry_listener_callback_4(self, msg):
        self.reference_odometry_listener_handler(msg, self.reference_id_4)

    def reference_odometry_listener_handler(self, msg, reference_id):
        # self.get_logger().info(f'Received Reference Odometry {reference_id}: {self.reference_position_dict[reference_id]}')
        position = [msg.x, msg.y, msg.z]
        self.reference_position_dict[reference_id] = position

    def kabsch(self, P, Q):
        """
        kabsch算法（点云配准算法）：有两个点集P和Q，这两个点集的点一一对应，但是P和Q并不重合
        假设P受到一根骨骼的影响，那么如何旋转和平移这根骨骼，使得最终变换后的P'与Q的差异最小？
        Computes the optimal translation and rotation matrices that minimize the 
        RMS deviation between two sets of points P and Q using Kabsch's algorithm.
        More here: https://en.wikipedia.org/wiki/Kabsch_algorithm
        Inspiration: https://github.com/charnley/rmsd

        inputs: P  N x 3 numpy matrix representing the coordinates of the points in P
                Q  N x 3 numpy matrix representing the coordinates of the points in Q

        return: A 4 x 3 matrix where the first 3 rows are the rotation and the last is translation
        """
        if (P.size == 0 or Q.size == 0):
            raise ValueError("Empty matrices sent to kabsch")
        centroid_P = np.mean(P, axis=0)
        centroid_Q = np.mean(Q, axis=0)
        # 均值归一到0
        # Center both matrices on centroid
        P_centered = P - centroid_P
        Q_centered = Q - centroid_Q
        H = P_centered.T.dot(Q_centered)                  # covariance matrix
        U, S, VT = np.linalg.svd(H)                        # SVD
        # calculate optimal rotation
        R = U.dot(VT).T
        # 这里变换为右手系
        if np.linalg.det(R) < 0:                          # correct rotation matrix for
            VT[2, :] *= -1  # right-hand coordinate system
            R = U.dot(VT).T
        t = centroid_Q - R.dot(centroid_P)                # translation vector
        # 反正最终返回的就是一个transform，注意这是一个右乘矩阵，即使一个4行3列的矩阵

        return R.T, t
    
    def apply_transform(self, P, R, t):
        """
        将旋转 R 和平移 t 应用于点集 P。

        参数:
        - P: 形状为(N,3)的点集。
        - R: 旋转矩阵。
        - t: 平移向量。

        返回:
        - P_transformed: 转换后的点集。
        """
        # 直接应用旋转矩阵R，不使用转置
        P_transformed = np.dot(P, R) + t
        return P_transformed

    def distance_matrix_listener_callback(self, msg):
        node_list = msg.address_list
        rows = msg.distance_mult_array.layout.dim[0].size
        cols = msg.distance_mult_array.layout.dim[1].size
        data = msg.distance_mult_array.data
        dist_matrix = np.array(data).reshape((rows, cols))

        mds = MDS(n_components=3, random_state=42, dissimilarity="precomputed",
                  normalized_stress=False, n_init=1, max_iter=1000, eps=1e-9, n_jobs=1)
        estimated_coords = mds.fit_transform(dist_matrix)
        coords_known = [self.reference_position_dict[self.reference_id_1],
                        self.reference_position_dict[self.reference_id_2],
                        self.reference_position_dict[self.reference_id_3],
                        self.reference_position_dict[self.reference_id_4]
                        ]
        estimated_coords_known = [estimated_coords[node_list.index(self.reference_id_1)],
                                  estimated_coords[node_list.index(
                                      self.reference_id_2)],
                                  estimated_coords[node_list.index(
                                      self.reference_id_3)],
                                  estimated_coords[node_list.index(
                                      self.reference_id_4)]
                                  ]
        coords_known = np.array(coords_known)
        estimated_coords_known = np.array(estimated_coords_known)
        estimated_coords = np.array(estimated_coords)
        
        R, t = self.kabsch(estimated_coords_known, coords_known)
        transformed_coords = self.apply_transform(estimated_coords, R, t)
        
        for node_id in node_list:
            if node_id not in self.publisher_dict:
                self.publisher_dict[node_id] = self.create_publisher(
                    Point, f'/locate/px4_{node_id}/position', 10)
            point = Point()
            point.x = transformed_coords[node_list.index(node_id)][0]
            point.y = transformed_coords[node_list.index(node_id)][1]
            point.z = transformed_coords[node_list.index(node_id)][2]
            self.publisher_dict[node_id].publish(point)

        self.get_logger().info(f'Reference Position:')
        for key, value in self.reference_position_dict.items():
            self.get_logger().info(f'{key}: {value}')

        self.get_logger().info('Estimated Coordinates:')
        for i, coord in enumerate(estimated_coords):
            self.get_logger().info(f'{node_list[i]}: {coord}')

        self.get_logger().info('Transformed Coordinates:')
        for i, coord in enumerate(transformed_coords):
            self.get_logger().info(f'{node_list[i]}: {coord}')




def main(args=None):
    rclpy.init(args=args)
    locate = Locate()
    rclpy.spin(locate)
    locate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
