import rclpy
from rclpy.node import Node

from client_vision_interfaces.msg import TurtlebotDetection
from turtlebot_interfaces.msg import Refiner2taskplanner
from turtlebot_interfaces.msg import Ui2Taskplanner
from ..util.util import calc_object_distance


class DetectionRefiner(Node):
    def __init__(self):
        super().__init__('detection_refiner')

        self.target_object_id = -1

        self.detection_sub_ = self.create_subscription(
            TurtlebotDetection,
            '/detection',
            self._detection_cb,
            1
        )
        self.ui_sub_ = self.create_subscription(
            Ui2Taskplanner,
            '/ui_msg',
            self._ui_msg_cb,
            1
        )
        self.pub_ = self.create_publisher(
            Refiner2taskplanner,
            '/vision2taskplanner',
            1
        )

    def _ui_msg_cb(self, msg: Ui2Taskplanner):
        if msg.deliver_flag:
            self.target_object_id = msg.deliver_object_id
            self.get_logger().info(f'Target object set: {self.target_object_id}')
        else:
            self.target_object_id = -1
            self.get_logger().info('Target object cleared')

    def _detection_cb(self, msg: TurtlebotDetection):
        out = Refiner2taskplanner()

        candidates = [] if self.target_object_id == -1 else [
            i for i, cid in enumerate(msg.class_ids)
            if cid == self.target_object_id
        ]

        if candidates:
            best_idx = max(candidates, key=lambda i: msg.score[i])
            u = (msg.x1[best_idx] + msg.x2[best_idx]) / 2.0
            v = float(msg.y2[best_idx])
            pos = calc_object_distance(u, v)
            out.object_dist  = float(pos.dist)
            out.object_theta = float(pos.theta)
            out.object_x     = float(pos.x)
            out.object_y     = float(pos.y)
            self.get_logger().debug(
                f'class={msg.class_ids[best_idx]}  dist={pos.dist:.3f}m  theta={pos.theta:.1f}deg'
            )

        self.pub_.publish(out)
def main(args=None):
    rclpy.init(args=args)
    node = DetectionRefiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
