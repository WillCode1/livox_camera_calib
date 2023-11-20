#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

string bag_file;
string lidar_topic;
string image_topic;
string result_path;
bool is_custom_msg;
int lidar_frame_cnt;
int image_frame_cnt;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("bag_file", bag_file, "");
  nh.param<string>("result_path", result_path, "");
  nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
  nh.param<string>("image_topic", image_topic, "/image_raw");
  nh.param<bool>("is_custom_msg", is_custom_msg, false);
  nh.param<int>("lidar_frame_cnt", lidar_frame_cnt, 20);
  nh.param<int>("image_frame_cnt", image_frame_cnt, 10);
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_)
  {
    std::string msg = "Loading the rosbag " + bag_file + " failue";
    ROS_ERROR_STREAM(msg.c_str());
    return -1;
  }
  ROS_INFO("Loading the rosbag %s", bag_file.c_str());
  rosbag::Bag bag;
  try
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e)
  {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return -1;
  }
  std::vector<string> topic_vec;
  topic_vec.push_back(lidar_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topic_vec));
  for (const rosbag::MessageInstance &m : view)
  {
    if (--lidar_frame_cnt == 0)
      break;

    if (is_custom_msg)
    {
      livox_ros_driver::CustomMsg livox_cloud_msg =
          *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
      for (uint i = 0; i < livox_cloud_msg.point_num; ++i)
      {
        pcl::PointXYZI p;
        p.x = livox_cloud_msg.points[i].x;
        p.y = livox_cloud_msg.points[i].y;
        p.z = livox_cloud_msg.points[i].z;
        p.intensity = livox_cloud_msg.points[i].reflectivity;
        output_cloud.points.push_back(p);
      }
    }
    else
    {
      sensor_msgs::PointCloud2 livox_cloud;
      livox_cloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(livox_cloud, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, cloud);
      for (uint i = 0; i < cloud.size(); ++i)
      {
        output_cloud.points.push_back(cloud.points[i]);
      }
    }
  }
  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  pcl::io::savePCDFileASCII(result_path + "/0.pcd", output_cloud);
  string msg = "Sucessfully save point cloud to pcd file: " + result_path + "/0.pcd";
  ROS_WARN_STREAM(msg.c_str());

  topic_vec[0] = image_topic;
  rosbag::View view2(bag, rosbag::TopicQuery(topic_vec));
  for (const rosbag::MessageInstance &m : view2)
  {
    if (--image_frame_cnt == 0)
      break;

    sensor_msgs::Image image = *(m.instantiate<sensor_msgs::Image>());
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(image, "bgr8");
    cv::imwrite(result_path + "/" + std::to_string(image_frame_cnt) + ".png", ptr->image);
  }
  return 0;
}