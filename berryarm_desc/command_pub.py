from cmath import nanj
import imp
import rclpy
from rclpy.time import Time
from rclpy.clock import Clock

import rclpy.node
import rclpy.qos
import rclpy.duration
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ROS2Pub(rclpy.node.Node):

    def __init__(self, *args):
        super(ROS2Pub, self).__init__("topic_pub")
        self.my_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Started the node")

    def timer_callback(self):
        points = JointTrajectory()
        point = JointTrajectoryPoint()
        points.joint_names.append("joint_0")
        points.joint_names.append("joint_1")
        points.joint_names.append("joint_2")

        point.positions.append(int(Clock().now().nanoseconds % 10) * 0.1)
        point.positions.append(0.2)
        point.positions.append(1.0)
        point.time_from_start = rclpy.duration.Duration(
            seconds=0.1, nanoseconds=0).to_msg()
        points.points.append(point)
        self.my_pub.publish(points)
        self.get_logger().info("Published a point")


def main():
    rclpy.init()
    node = ROS2Pub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
