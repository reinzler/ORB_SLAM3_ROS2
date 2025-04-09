#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <sophus/se3.hpp> // Include Sophus for SE3 type

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    // Create a publisher for the pose messages
    m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);

    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout << "one frame has been sent" << std::endl;

    // Track monocular frame and get camera pose
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    // Prepare pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();  // Timestamp the message
    pose_msg.header.frame_id = "camera_frame";  // Reference frame for the pose

    // Set the position from the translation part of the SE3 pose
    pose_msg.pose.position.x = Tcw.translation().x();
    pose_msg.pose.position.y = Tcw.translation().y();
    pose_msg.pose.position.z = Tcw.translation().z();
    
    // Convert rotation matrix to quaternion and set the orientation part
    Eigen::Quaternionf q(Tcw.rotationMatrix());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Publish the pose message
    m_pose_publisher->publish(pose_msg);
}
