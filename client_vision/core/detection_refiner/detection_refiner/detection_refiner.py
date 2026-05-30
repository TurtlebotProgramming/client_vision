import rclpy
from rclpy.node import Node

from client_vision_interfaces.msg import TurtlebotDetection
from turtlebot_interfaces.msg import Refiner2taskplanner
from ..util.util import calc_object_distance


class DetectionRefiner(Node):
    def __init__(self):
        super().__init__('detection_refiner')

        self.sub_ = self.create_subscription(
            TurtlebotDetection,
            '/detection',
            self._detection_cb,
            1
        )
        self.pub_ = self.create_publisher(
            Refiner2taskplanner,
            '/vision2taskplanner',
            1
        )

    def _detection_cb(self, msg: TurtlebotDetection):
        if not msg.class_ids:
            return

        # 가장 높은 confidence 객체 선택
        best_idx = int(max(range(len(msg.score)), key=lambda i: msg.score[i]))

        # 바운딩박스 하단 중심 픽셀
        u = (msg.x1[best_idx] + msg.x2[best_idx]) / 2.0
        v = float(msg.y2[best_idx])

        pos = calc_object_distance(u, v)

        out = Refiner2taskplanner()
        out.object_dist  = float(pos.dist)
        out.object_theta = float(pos.theta)
        out.object_x = float(pos.x)
        out.object_y = float(pos.y)
        self.pub_.publish(out)

        self.get_logger().debug(
            f'class={msg.class_ids[best_idx]}  dist={pos.dist:.3f}m  theta={pos.theta:.1f}deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionRefiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
