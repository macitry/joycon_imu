import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import sys
sys.path.append('/home/mac/joycon_pro')
from driver.joycon import Joycon

class JoyCon_Node(Node):
    def __init__(self):
        super().__init__('JoyCon_Node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_transform)
        self.joycon = Joycon()
    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_link'
        
        # 设置平移部分
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        Quaternion=self.joycon.get_Quaternion()
        qx= Quaternion[1]
        qy= Quaternion[2]
        qz= Quaternion[3]
        qw= Quaternion[0]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = JoyCon_Node()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()