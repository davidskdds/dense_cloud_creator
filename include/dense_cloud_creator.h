/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include "helpers.h"
#include "PointCloudPlus.h"
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <boost/math/interpolators/barycentric_rational.hpp>
#include <pcl/io/pcd_io.h>

using namespace Eigen;
using namespace pcl;

// Function to split the input string into a vector of strings
inline std::vector<std::string> splitIntoWords(const std::string &str)
{
  std::istringstream iss(str);
  std::vector<std::string> words;
  std::string word;

  while (iss >> word)
  {
    words.push_back(word);
  }

  return words;
}

class dense_cloud_creator
{

public:
  dense_cloud_creator();
  ~dense_cloud_creator();
  void spin();

private:
  ros::NodeHandle nh_;

  std::vector<std::string> bagnames;

  std::string lidarSubTopicName;

  std::string pose_file_dir;
  std::string result_dir;
  std::string sensor;

  float grid_size;

  int processedPcCounter = 0;

  ros::Publisher pubCurrCloud;

  Eigen::Matrix4f lidar2imu;
  Eigen::Matrix4f imu2lidar;

  // interpolated poses
  Eigen::VectorXd highResStamps;
  std::vector<Eigen::Matrix4f> highResTransforms;

  int startFromId = 0;

  double lastPcMsgStamp = -1.0;

  double dt = 0.0001;

  int max_num_points;

  bool reachedMaxNumPoints = false;

  float min_dist;
  float max_dist;

  PointCloud<PointXYZI> globalPoints;
  PointCloud<PointXYZI> filteredPoints;

  void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void createHighResPoses(Eigen::MatrixXd &sparsePoses);
};
