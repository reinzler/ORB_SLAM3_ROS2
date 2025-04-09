#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>  // Add this for PoseStamped

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    // Add publisher for PoseStamped messages
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;  // New publisher for Pose

};

#endif

