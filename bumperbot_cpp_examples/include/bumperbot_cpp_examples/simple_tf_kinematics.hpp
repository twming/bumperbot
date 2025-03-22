#ifndef SIMPLE_TF_KINEMATICS_HPP_
#define SIMPLE_TF_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>


class SimpleTfkinematics : public rclcpp::Node
{
    public:
        SimpleTfkinematics(const std::string &name);

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        geometry_msgs::msg::TransformStamped static_transform_stamped_;

};

#endif