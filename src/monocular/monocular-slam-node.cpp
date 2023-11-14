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
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "dji/image",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    map_points_publisher_=this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 1);
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

    std::cout<<"one frame has been sent"<<std::endl;


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
}


sensor_msgs::msg::PointCloud2 MonocularSlamNode::MapPointsToPointCloud (const std::vector<ORB_SLAM3::MapPoint*> &map_points) {
  if (map_points.size() <= 1) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  const int num_channels = 3; // x y z
 

  std::cout << "check 1 " << std::endl;
  cloud.header.stamp = tf2_ros::toMsg(current_frame_time_);
  std::cout <<"check 2 " << std::endl;
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
    std::cout <<"check 3 " << std::endl;
    std::cout <<map_points.size() << std::endl;
    // if (map_points.at(i)->nObs >= min_observations_per_point_) {
    // unique_lock<mutex> lock(ORB_SLAM3::MapPoint::mGlobalMutex);
    ORB_SLAM3::MapPoint* pMP = map_points[i];
    std::cout <<"check 4 " << std::endl;
    if(pMP){
        if(!pMP->isBad())
            {
                std::cout <<"check 5" << std::endl;
                data_array[0] = pMP->GetWorldPos()(2); //x. Do the transformation by just reading at the position of z instead of x
                data_array[1] = -1.0* pMP->GetWorldPos()(0); //y. Do the transformation by just reading at the position of x instead of y
                data_array[2] = -1.0* pMP->GetWorldPos()(1); //z. Do the transformation by just reading at the position of y instead of z
                //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

                memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        // }
            }
        }
  }
  std::cout <<"check 5 " << std::endl;
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