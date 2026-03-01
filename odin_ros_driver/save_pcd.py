import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sys

class PCDSaver(Node):
    def __init__(self):
        super().__init__('pcd_saver')
        # 订阅你的 SLAM 点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/odin1/cloud_slam',
            self.listener_callback,
            10)
        self.get_logger().info('正在等待 /odin1/cloud_slam 话题的数据...')

    def listener_callback(self, msg):
        num_points = msg.width * msg.height
        self.get_logger().info(f'接收到点云数据，包含 {num_points} 个点。正在保存...')
        
        filename = "my_map_captured.pcd"
        
        # 写入 PCD 文件头 (根据二进制格式)
        with open(filename, 'w') as f:
            f.write("VERSION .7\n")
            # 注意：如果你的点云不仅包含 xyz，还包含 intensity 或 rgb，
            # 可以在这里修改 FIELDS。为了兼容性，这里按最基本的二进制块写入
            f.write("FIELDS x y z rgb\n") 
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {num_points}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {num_points}\n")
            f.write("DATA binary\n")
            
        # 写入二进制核心数据
        with open(filename, 'ab') as f:
            f.write(bytearray(msg.data))
            
        self.get_logger().info(f'保存成功！文件已存为: {filename}')
        
        # 保存完成后退出节点
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    pcd_saver = PCDSaver()
    try:
        rclpy.spin(pcd_saver)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
