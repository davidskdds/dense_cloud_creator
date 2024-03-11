/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */
#include "dense_cloud_creator.h"
#include <ros/console.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace Eigen;
using namespace pcl;

dense_cloud_creator::dense_cloud_creator()
{
    ros::NodeHandle nh("~");

    // load parameters
    std::cout << "Current parameter config:\n";

    nh.getParam("lidar_topic", lidarSubTopicName);
    std::cout << "lidar_topic: " << lidarSubTopicName << std::endl;

    nh.getParam("pose_file_dir", pose_file_dir);
    std::cout << "pose_file_dir: " << pose_file_dir << std::endl;

    nh.getParam("result_dir", result_dir);
    std::cout << "result_dir: " << result_dir << std::endl;

    std::string bag_dirs;
    nh.getParam("bag_dirs", bag_dirs);
    std::cout << "bag_dirs: " << bag_dirs << std::endl;

    bagnames = splitIntoWords(bag_dirs);

    nh.getParam("grid_size", grid_size);
    std::cout << "grid_size: " << grid_size << std::endl;

    nh.getParam("max_num_points", max_num_points);
    std::cout << "max_num_points: " << max_num_points << std::endl;

    nh.getParam("min_dist", min_dist);
    std::cout << "min_dist: " << min_dist << std::endl;

    // init transform in imu frame
    imu2lidar = Matrix4f::Identity();

    Eigen::Quaternionf q;
    nh.getParam("q_x", q.x());
    nh.getParam("q_y", q.y());
    nh.getParam("q_z", q.z());
    nh.getParam("q_w", q.w());
    std::cout << "Quaternion from imu frame: " << q << std::endl;

    imu2lidar.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();

    nh.getParam("t_x", imu2lidar(0, 3));
    nh.getParam("t_y", imu2lidar(1, 3));
    nh.getParam("t_z", imu2lidar(2, 3));

    std::cout << "Translation from imu frame: " << imu2lidar.block(0, 3, 3, 1) << std::endl;
    lidar2imu = imu2lidar.inverse();

    nh.getParam("sensor", sensor);
    std::cout << "sensor: " << sensor << std::endl;

    // DEBUG
    if (false)
    {
        std::string bag_dirs2 = "/home/david/Rosbags/Hilti/Additional_Seq/exp04_construction_upper_level.bag";

        bagnames = splitIntoWords(bag_dirs2);

        Eigen::Quaternionf q;

        q.x() = 0.7094397486399825;
        q.y() = -0.7047651311547696;
        q.z() = 0.001135774698382635;
        q.w() = -0.0002509459564800096;

        imu2lidar.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();
        imu2lidar(0, 3) = 0.0;
        imu2lidar(1, 3) = 0.0;
        imu2lidar(2, 3) = 0.055;
        lidar2imu = imu2lidar.inverse();

        lidarSubTopicName = "/hesai/pandar";

        grid_size = 0.05;

        result_dir = "/home/david/optim";

        pose_file_dir = "/home/david/optim/Poses.txt";
    }

    pubCurrCloud = nh.advertise<sensor_msgs::PointCloud2>("/dense_cloud_creator/curr_pc", 1);

    globalPoints.points.reserve(max_num_points);
    filteredPoints.points.reserve(max_num_points);
}

dense_cloud_creator::~dense_cloud_creator()
{
}

void dense_cloud_creator::spin()
{
    // Outputting the words
    std::cout << "You entered the following rosbags:" << std::endl;
    for (const auto &rosbagdir : bagnames)
    {
        std::cout << rosbagdir << std::endl;
    };

    // load poses
    Eigen::MatrixXd sparsePoses = readPosesFromFile(pose_file_dir);

    std::cout << "Loaded " << sparsePoses.rows() << " poses from " << pose_file_dir << std::endl;

    std::cout << "Create high resolution poses . . ." << std::endl;
    createHighResPoses(sparsePoses);

    std::vector<std::string> topics;

    topics.push_back(lidarSubTopicName);

    std::cout << "Process rosbag/s . . ." << std::endl;
    for (auto rosbagDir : bagnames)
    {
        rosbag::Bag bag;

        try
        {
            bag.open(rosbagDir);
        }
        catch (...)
        {
            std::cerr << "Rosbag directory (" << rosbagDir << ") is invalid, processing is aborted\n";
            return;
        }

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // iterate over point clouds
        for (rosbag::MessageInstance const m : view)
        {
            if (reachedMaxNumPoints)
                break;

            sensor_msgs::PointCloud2::ConstPtr pc2Ptr = m.instantiate<sensor_msgs::PointCloud2>();

            if (pc2Ptr != nullptr)
                callbackPointCloud(pc2Ptr);
        }

        bag.close();
    }

    std::cout << "Apply random grid filter last time before saving point cloud . . . " << std::endl;
    randomGridDownsampling(globalPoints.makeShared(), filteredPoints, grid_size);

    // Save the cloud to a .pcd file
    std::string filename = result_dir + "/DensePointCloud.pcd";

    globalPoints.width = filteredPoints.points.size();
    globalPoints.height = 1;

    std::cout << "Save accumulated points to " << filename << " . . ." << std::endl;
    if (io::savePCDFileASCII(filename, filteredPoints) == -1)
    {
        PCL_ERROR("Failed to save PCD file\n");
    }

    std::cout << "Processing of rosbags/s finished . . . " << std::endl;

    ros::Rate rate(1000);

    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

void dense_cloud_creator::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PointCloudPlus::Ptr newPC(new PointCloudPlus);

    newPC->resize(msg->height * msg->width);

    int arrayPosition;
    uint8_t ring_tmp8;
    uint16_t ring_tmp;
    uint32_t relStampNano;
    double stampMsg = msg->header.stamp.toSec();

    if (stampMsg - 0.3 < highResStamps(0))
        return;
    if (stampMsg + 0.3 > highResStamps(highResStamps.size() - 1))
        return;

    // only relevant for unknown sensor type
    if (sensor == "unknown" && lastPcMsgStamp < 0.0)
    {
        lastPcMsgStamp = stampMsg;
        return;
    }

    double deltaTPcs = stampMsg - lastPcMsgStamp;

    float tmpStampFloat;
    double tmpStampDouble;

    for (uint k = 0; k < msg->height * msg->width; ++k)
    {
        arrayPosition = k * msg->point_step;

        newPC->at(k).isStatic = 0;

        // xyz
        memcpy(&newPC->at(k).x, &msg->data[arrayPosition + msg->fields[0].offset], sizeof(float));
        memcpy(&newPC->at(k).y, &msg->data[arrayPosition + msg->fields[1].offset], sizeof(float));
        memcpy(&newPC->at(k).z, &msg->data[arrayPosition + msg->fields[2].offset], sizeof(float));

        if (sensor == "hesai")
        {
            // stamp and ring
            memcpy(&tmpStampDouble, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(double));
            memcpy(&ring_tmp, &msg->data[arrayPosition + msg->fields[5].offset], sizeof(uint16_t));

            newPC->at(k).stamp = tmpStampDouble;
            newPC->at(k).id = (int)ring_tmp;
        }
        else if (sensor == "ouster")
        {
            // stamp and ring
            memcpy(&relStampNano, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(uint32_t));
            memcpy(&ring_tmp8, &msg->data[arrayPosition + msg->fields[6].offset], sizeof(uint8_t));

            tmpStampDouble = stampMsg + 1e-9 * (double)relStampNano;

            newPC->at(k).stamp = tmpStampDouble;
            newPC->at(k).id = (int)ring_tmp8;
        }
        else if (sensor == "robosense")
        {
            // stamp and ring
            memcpy(&tmpStampDouble, &msg->data[arrayPosition + msg->fields[5].offset], sizeof(double));
            memcpy(&ring_tmp, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(uint16_t));

            newPC->at(k).stamp = tmpStampDouble;
            newPC->at(k).id = (int)ring_tmp;
        }
        else if (sensor == "velodyne")
        {
            // stamp and ring
            memcpy(&tmpStampFloat, &msg->data[arrayPosition + msg->fields[5].offset], sizeof(float));
            memcpy(&ring_tmp, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(uint16_t));

            newPC->at(k).stamp = stampMsg + static_cast<double>(tmpStampFloat);
            newPC->at(k).id = (int)ring_tmp;
        }
        else if (sensor == "unknown")
        {
            // use heuristic stamp
            newPC->at(k).stamp = stampMsg + deltaTPcs * (double)k / (double)(msg->height * msg->width);

            // add artificial ring index
            newPC->at(k).id = k % 1000;
        }

        // set padding to 1.0
        newPC->at(k).data[3] = 1.0;

        // transform point to reference frame
        newPC->at(k).getVector4fMap() = lidar2imu * newPC->at(k).getVector4fMap();
    }

    int tformId = startFromId;
    PointXYZ currPoint;

    // transform to world frame
    for (auto &point : newPC->points)
    {
        // skip points that are closer than min_dist
        if (point.getVector3fMap().norm() < min_dist)
        {
            point.getVector3fMap().setZero();
            continue;
        }

        // find transform
        for (tformId = startFromId; tformId < highResStamps.size(); ++tformId)
            if (point.stamp < highResStamps(tformId))
                break;

        // transform to world
        currPoint.getVector4fMap() = highResTransforms[tformId] * point.getVector4fMap();
        point.getVector4fMap() = highResTransforms[tformId] * point.getVector4fMap();

        // add to global points
        globalPoints.points.push_back(currPoint);

        if (static_cast<int>(globalPoints.points.size()) == max_num_points)
        {

            std::cout << "Maximum number of points reached, apply grid filter and continue . . . " << std::endl;

            randomGridDownsampling(globalPoints.makeShared(), filteredPoints, grid_size);

            // not efficient but dont have a better idea atm
            globalPoints = filteredPoints;

            if (static_cast<int>(globalPoints.points.size()) == max_num_points)
            {
                reachedMaxNumPoints = true;
                return;
            }
        }
    }

    startFromId = tformId;

    ++processedPcCounter;

    // not mandatory
    sensor_msgs::PointCloud2 submapMsg;

    pcl::toROSMsg(*newPC, submapMsg);

    submapMsg.header.frame_id = "map";

    pubCurrCloud.publish(submapMsg);

    std::cout << "Num accumulated points: " << globalPoints.points.size() << std::endl;
}

void dense_cloud_creator::createHighResPoses(Eigen::MatrixXd &sparsePoses)
{
    // dirty hack to ensure that the interpolation works, even if the first pose is present twice in the pose file
    sparsePoses(0,0) -= 0.0001;

    double deltaTPoses = sparsePoses(sparsePoses.rows() - 1, 0) - sparsePoses(0, 0);
    int numPosesHighRes = std::round(deltaTPoses / dt) + 1;

    highResStamps = VectorXd::LinSpaced(numPosesHighRes, sparsePoses(0, 0), sparsePoses(sparsePoses.rows() - 1, 0));

    highResTransforms.resize(highResStamps.size());

    int lowerSparsePosesId = 0;

    std::vector<double> translX;
    std::vector<double> translY;
    std::vector<double> translZ;
    std::vector<double> stamps;

    int lowIdTranslInterp = 0;
    int upIdTranslInterp = 0;

    // generate pseudo values for init
    translX.push_back(0.0);
    translX.push_back(1.0);
    stamps.push_back(0.0);
    stamps.push_back(1.0);
    translX.push_back(2.0);
    stamps.push_back(2.0);

    boost::math::barycentric_rational<double> x = boost::math::barycentric_rational<double>(stamps.data(), translX.data(), stamps.size(), 2);
    boost::math::barycentric_rational<double> y = boost::math::barycentric_rational<double>(stamps.data(), translX.data(), stamps.size(), 2);
    boost::math::barycentric_rational<double> z = boost::math::barycentric_rational<double>(stamps.data(), translX.data(), stamps.size(), 2);

    Quaterniond q1, q2, q_interp;
    double dtQuat, dtCurr;

    // create high resolution poses
    for (int k = 0; k < highResTransforms.size(); ++k)
    {
        // update interpolation params
        if ((k == 0 || highResStamps(k) > sparsePoses(lowerSparsePosesId + 1, 0)) && lowerSparsePosesId < sparsePoses.rows() - 1)
        {
            if (k == 0)
                lowerSparsePosesId = 0;
            else
                ++lowerSparsePosesId;

            lowIdTranslInterp = std::max(lowerSparsePosesId - 2, 0);
            upIdTranslInterp = std::min(lowerSparsePosesId + 2, static_cast<int>(sparsePoses.rows()) - 1);

            translX.resize(0);
            translY.resize(0);
            translZ.resize(0);
            stamps.resize(0);

            for (int j = lowIdTranslInterp; j <= upIdTranslInterp; ++j)
            {
                stamps.push_back(sparsePoses(j, 0));
                translX.push_back(sparsePoses(j, 1));
                translY.push_back(sparsePoses(j, 2));
                translZ.push_back(sparsePoses(j, 3));
            }

            x = boost::math::barycentric_rational<double>(stamps.data(), translX.data(), stamps.size(), 2);
            y = boost::math::barycentric_rational<double>(stamps.data(), translY.data(), stamps.size(), 2);
            z = boost::math::barycentric_rational<double>(stamps.data(), translZ.data(), stamps.size(), 2);

            q1.x() = sparsePoses(lowerSparsePosesId, 4);
            q1.y() = sparsePoses(lowerSparsePosesId, 5);
            q1.z() = sparsePoses(lowerSparsePosesId, 6);
            q1.w() = sparsePoses(lowerSparsePosesId, 7);

            q2.x() = sparsePoses(lowerSparsePosesId + 1, 4);
            q2.y() = sparsePoses(lowerSparsePosesId + 1, 5);
            q2.z() = sparsePoses(lowerSparsePosesId + 1, 6);
            q2.w() = sparsePoses(lowerSparsePosesId + 1, 7);

            dtQuat = sparsePoses(lowerSparsePosesId + 1, 0) - sparsePoses(lowerSparsePosesId, 0);
        }

        // init transform
        highResTransforms[k].setIdentity();

        // interpolate rotation
        dtCurr = highResStamps(k) - sparsePoses(lowerSparsePosesId, 0);
        q_interp = q1.slerp(dtCurr / dtQuat, q2);

        AngleAxisd aa_interp(q_interp);
        Vector3d aa_interp_vec = aa_interp.axis() * aa_interp.angle();

        highResTransforms[k].block(0, 0, 3, 3) = axang2rotm(aa_interp_vec).cast<float>();

        // interpolate translation
        highResTransforms[k](0, 3) = static_cast<float>(x(highResStamps(k)));
        highResTransforms[k](1, 3) = static_cast<float>(y(highResStamps(k)));
        highResTransforms[k](2, 3) = static_cast<float>(z(highResStamps(k)));
    }
}
