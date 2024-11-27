#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rmw/types.h"
#include "rclcpp/qos.hpp"

#include <cv_bridge/cv_bridge.h>

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void publish_position_as_transform (Sophus::SE3f &position);
    tf2::Transform transform_from_mat (Sophus::SE3f &se3);
    void publish_map(std::vector<ORB_SLAM3::MapPoint*> &k);
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud (const std::vector<ORB_SLAM3::MapPoint*> &map_points);

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // rosparams
    std::string camera_frame_id_param_, target_frame_id_param_, map_frame_id_param_;

    tf2::TimePoint current_frame_time_;
    tf2::TimePoint previous_frame_time_;
    tf2::Transform previous_transform_;
    bool first_run_ = true;

    int min_observations_per_point_ = 2;

};

#endif
