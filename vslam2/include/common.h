#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "System.h"
#include "ImuTypes.h"

extern ORB_SLAM3::System::eSensor sensor_type;
extern std::string world_frame_id, cam_frame_id, imu_frame_id;
extern Sophus::SE3f Tc0w;

extern ros2::Publisher pose_pub, map_points_pub;

void setup_ros_publishers(ros::NodeHandle &, image_transport::ImageTransport &, Eigen::Vector3d = Eigen::Vector3d::Zero());

void publish_ros_camera_pose(Sophus::SE3f, ros::Time);
void publish_ros_tracked_mappoints(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);
void publish_ros_tf_transform(Sophus::SE3f, string, string, ros::Time);

tf::Transform SE3f_to_tfTransform(Sophus::SE3f);
sensor_msgs::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);