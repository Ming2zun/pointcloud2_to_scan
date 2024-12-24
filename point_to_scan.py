import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.node import Node
from typing import Tuple


class PointCloudToScanWithHeightFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan_with_height_filter')
        self.subscription = self.create_subscription(PointCloud2, '/points_raw', self.callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.angle_threshold = 0.01 #两点小于此值，被视为一条线上的激光点
    def callback(self, cloud_msg: PointCloud2):
        MIN_HEIGHT, MAX_HEIGHT = 0.8, 2.0 #选择你想要保留高度范围的点云
        points = np.frombuffer(cloud_msg.data, dtype=np.float32).reshape(-1, 4)
        height_mask = (MIN_HEIGHT <= points[:, 2]) & (points[:, 2] <= MAX_HEIGHT)
        filtered_points = points[height_mask]
        distances, angles = self.cartesian_to_polar(filtered_points[:, :2])
         # 去除角度几乎相同的点，保留距离较小的那个点
        unique_points = self.remove_duplicate_points(angles, distances)

        # 创建激光扫描消息
        scan_msg = LaserScan()
        scan_msg.header = cloud_msg.header
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 6.283185307179586
        scan_msg.angle_increment = 0.02616666666666667##################(根据你实际的3D激光数据进行换算)
        scan_msg.range_min = 0.0
        scan_msg.range_max = 100.0
        # 生成理想的角度序列
        ideal_angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment, scan_msg.angle_increment)
        # 与提取后的点进行比较并提取
        ranges, intensities = self.match_points_to_ideal_angles(unique_points[1], unique_points[0], ideal_angles)
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = intensities.tolist()
        self.publisher.publish(scan_msg)
    def match_points_to_ideal_angles(self, angles: np.ndarray, distances: np.ndarray, ideal_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # 初始化列表来存储距离和强度值
        matched_distances = []
        matched_intensities = []
        # 对于每个理想的角度值，找到最接近该角度的原始点
        for ideal_angle in ideal_angles:
            # 找到最接近理想角度的原始角度索引
            closest_idx = np.argmin(np.abs(angles - ideal_angle))
            # 检查是否找到了合适的点
            if np.isclose(angles[closest_idx], ideal_angle, atol=self.angle_threshold):
                # 添加距离和强度值
                matched_distances.append(distances[closest_idx])
                matched_intensities.append(1.0)  # 假设所有点的强度均为 1.0
            else:
                # 如果没有找到匹配的点，则设置为最大距离或无穷远
                matched_distances.append(np.inf)
                matched_intensities.append(0.0)  # 设置强度为 0.0 表示无效点

        # 转换为 NumPy 数组
        matched_distances = np.array(matched_distances)
        matched_intensities = np.array(matched_intensities)
        return matched_distances, matched_intensities
    def remove_duplicate_points(self, angles: np.ndarray, distances: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # 将距离和角度组合成一个数组
        combined = np.column_stack((angles, distances))
        # 按角度排序
        sorted_combined = combined[np.argsort(combined[:, 0])]
        sorted_angles = sorted_combined[:, 0]
        sorted_distances = sorted_combined[:, 1]
        # 初始化列表来存储唯一的点
        unique_distances = []
        unique_angles = []
        # 遍历排序后的点
        prev_angle = None
        for angle, distance in zip(sorted_angles, sorted_distances):
            # 如果这是第一个点，或者当前角度与前一个点的角度差异足够大
            if prev_angle is None or abs(angle - prev_angle) > self.angle_threshold:
                unique_distances.append(distance)
                unique_angles.append(angle)
                prev_angle = angle
            elif distance < unique_distances[-1]:  # 如果当前点的距离更小
                # 更新最后一个点
                unique_distances[-1] = distance
                unique_angles[-1] = angle
        # 返回保留下来的点的唯一距离值和角度值
        return np.array(unique_distances), np.array(unique_angles)
    @staticmethod
    def cartesian_to_polar(xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        r = np.sqrt(xy[:, 0]**2 + xy[:, 1]**2)
        theta = np.arctan2(xy[:, 1], xy[:, 0])
        return r, theta

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = PointCloudToScanWithHeightFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
