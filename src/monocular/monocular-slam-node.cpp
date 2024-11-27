#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    camera_frame_id_param_ = "camera";
    target_frame_id_param_ = "camera";
    map_frame_id_param_    = "map";
    // std::cout << "slam changed" << std::endl;

    std::string topic_image = this->declare_parameter<std::string>("topic_image", "/dji/image");
    std::string topic_pointcloud = this->declare_parameter<std::string>("topic_pointcloud", "/map_points");
    std::string topic_pose = this->declare_parameter<std::string>("topic_pose", "/pose");
    std::string topic_odom = this->declare_parameter<std::string>("topic_odom", "/visual_odometry");
    target_frame_id_param_ = this->declare_parameter<std::string>("frame_id", "viz_odom");

    // Display parameters
    RCLCPP_INFO(this->get_logger(), " topic_image : %s", topic_image.c_str());
    RCLCPP_INFO(this->get_logger(), " topic_pointcloud : %s", topic_pointcloud.c_str());
    RCLCPP_INFO(this->get_logger(), " topic_pose : %s", topic_pose.c_str());
    RCLCPP_INFO(this->get_logger(), " topic_odom : %s", topic_odom.c_str());
    RCLCPP_INFO(this->get_logger(), " Frame id : %s", target_frame_id_param_.c_str());

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    first_run_ = true;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        topic_image,
        qos,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    map_points_publisher_=this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pointcloud, 1);
    pose_publisher_=this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_pose, 1);
    odometry_publisher_=this->create_publisher<nav_msgs::msg::Odometry>(topic_odom, 1);
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

    Sophus::SE3f raw_pose = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    std::vector<ORB_SLAM3::MapPoint*> k = m_SLAM->GetTrackedMapPoints();
    current_frame_time_ = tf2_ros::fromMsg(msg->header.stamp);

    publish_position_as_transform(raw_pose);
    publish_map(k);

}



void MonocularSlamNode::publish_position_as_transform (Sophus::SE3f &position){
  // Get transform from map to camera frame
  tf2::Transform tf_transform = transform_from_mat(position);


  // Make transform from camera frame to target frame
//   tf2::Transform tf_map2target = transform_to_target(tf_transform, camera_frame_id_param_, target_frame_id_param_);

  // Make message
  tf2::Stamped<tf2::Transform> tf_map2target_stamped;

  tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_transform, current_frame_time_, map_frame_id_param_);
  geometry_msgs::msg::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
  msg.child_frame_id = target_frame_id_param_;

  // Broadcast tf
  static tf2_ros::TransformBroadcaster tf_broadcaster(*this);
  tf_broadcaster.sendTransform(msg);    

  // Publish pose stamped as well.
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = msg.header;
  pose_msg.pose.position.x = msg.transform.translation.x;
  pose_msg.pose.position.y = msg.transform.translation.y;
  pose_msg.pose.position.z = msg.transform.translation.z;
  pose_msg.pose.orientation.x = msg.transform.rotation.x;
  pose_msg.pose.orientation.y = msg.transform.rotation.y;
  pose_msg.pose.orientation.z = msg.transform.rotation.z;
  pose_msg.pose.orientation.w = msg.transform.rotation.w;
  pose_publisher_->publish(pose_msg);


  // Publish odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = msg.header;
  odom_msg.child_frame_id = target_frame_id_param_;
  odom_msg.pose.pose = pose_msg.pose;

  // compute twist  from previous pose

  if (!first_run_) {
    // Compute time difference


    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_frame_time_ - previous_frame_time_).count();
    if (dt == 0) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Time difference is zero. Odometry velocities cant be computed, Check image timestamp.");
      return;
    }

    // Compute velocity
    tf2::Vector3 linear_velocity = (tf_transform.getOrigin() - previous_transform_.getOrigin()) / dt;

    tf2::Quaternion delta_rotation = tf_transform.getRotation() * previous_transform_.getRotation().inverse();
    tf2::Vector3 angular_velocity = (delta_rotation.getAxis() * delta_rotation.getAngle()) / dt;

    // Fill in the twist part of the odometry message
    odom_msg.twist.twist.linear.x = linear_velocity.x();
    odom_msg.twist.twist.linear.y = linear_velocity.y();
    odom_msg.twist.twist.linear.z = linear_velocity.z();
    odom_msg.twist.twist.angular.x = angular_velocity.x();
    odom_msg.twist.twist.angular.y = angular_velocity.y();
    odom_msg.twist.twist.angular.z = angular_velocity.z();
  } else {
    first_run_ = false;
  }

  // Update previous transform and time
  previous_transform_ = tf_transform;
  previous_frame_time_ = current_frame_time_;

  // Publish the odometry message
  odometry_publisher_->publish(odom_msg);

}


sensor_msgs::msg::PointCloud2 MonocularSlamNode::MapPointsToPointCloud (const std::vector<ORB_SLAM3::MapPoint*> &map_points) {
  if (map_points.size() <= 1) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  const int num_channels = 3; // x y z
 

  cloud.header.stamp = tf2_ros::toMsg(current_frame_time_);
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  
  for (unsigned int i=0; i<cloud.width; i++) {
    ORB_SLAM3::MapPoint* pMP = map_points[i];
    if(pMP){
        if(!pMP->isBad())
            {
      
                data_array[0] = pMP->GetWorldPos()(2); //x. Do the transformation by just reading at the position of z instead of x
                data_array[1] = -1.0* pMP->GetWorldPos()(0); //y. Do the transformation by just reading at the position of x instead of y
                data_array[2] = -1.0* pMP->GetWorldPos()(1); //z. Do the transformation by just reading at the position of y instead of z

                memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        // }
            }
        }
  }
  return cloud;
}

  



tf2::Transform MonocularSlamNode::transform_from_mat (Sophus::SE3f &se3) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  Eigen::Matrix4f transformation_matrix = se3.matrix();
  tf2::Matrix3x3 tf_camera_rotation (transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                    transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                    transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2)
                                   );

  tf2::Vector3 tf_camera_translation (se3.translation().x(), se3.translation().y(), se3.translation().z());

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}


// void MonocularSlamNode::initialize_scale(){

//     // subscribe to z position,
//     // based simple division find the scaling factor 
//     // update the scale
//     // should keep estimating scale till convergence is found, keeping it within a threshold
// }


// tf2::Transform Node::TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
//   // Transform tf_in from frame_in to frame_target
//   tf2::Transform tf_map2orig = tf_in;
//   tf2::Transform tf_orig2target;
//   tf2::Transform tf_map2target;

//   tf2::Stamped<tf2::Transform> transformStamped_temp;
//   try {
//     // Get the transform from camera to target
//     geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, ros::Time(0));
//     // Convert to tf2
//     tf2::fromMsg(tf_msg, transformStamped_temp);
//     tf_orig2target.setBasis(transformStamped_temp.getBasis());
//     tf_orig2target.setOrigin(transformStamped_temp.getOrigin());

//   } catch (tf2::TransformException &ex) {
//     ROS_WARN("%s",ex.what());
//     //ros::Duration(1.0).sleep();
//     tf_orig2target.setIdentity();
//   }
//   // Transform from map to target
//   tf_map2target = tf_map2orig * tf_orig2target;
//   return tf_map2target;
// }

void MonocularSlamNode::publish_map(std::vector<ORB_SLAM3::MapPoint*> &k){
        if(k.size()> 0){
            sensor_msgs::msg::PointCloud2 msg = MapPointsToPointCloud(k);
            map_points_publisher_->publish(msg);
        }
}