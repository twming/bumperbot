import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SimpleTfKinematic(Node):
    def __init__(self):
        super().__init__('simplef_tf_kinematics')

        self.static_tf_broadcaster_=StaticTransformBroadcaster(self)

        self.static_transform_stamped_ = TransformStamped()
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0.0 
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info("Publishing static transform between %s and %s" % (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))


def main():
    rclpy.init()
    simple_tf_kinematics=SimpleTfKinematic()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.detroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()