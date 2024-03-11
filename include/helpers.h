/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef HELPERS_DCC_H
#define HELPERS_DCC_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <unsupported/Eigen/MatrixFunctions>
#include "PointCloudPlus.h"

#define EPSILON_ROT 0.00001

using namespace pcl;
using namespace Eigen;



inline Eigen::MatrixXd readPosesFromFile(const std::string& directory) {
    std::ifstream infile(directory);
    if (!infile.is_open()) {
        std::cerr << "Failed to open poses.txt" << std::endl;
        return Eigen::MatrixXd();  // return an empty matrix
    }

    std::vector<std::vector<double>> data;
    std::string line;

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double value;
        std::vector<double> row;
        while (iss >> value) {
            row.push_back(value);
        }
        if (!row.empty()) {
            data.push_back(row);
        }
    }

    if (data.empty() || data[0].empty()) {
        std::cerr << "The file is empty or improperly formatted." << std::endl;
        return Eigen::MatrixXd();  // return an empty matrix
    }

    Eigen::MatrixXd matrix(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < data[0].size(); ++j) {
            matrix(i, j) = data[i][j];
        }
    }

    return matrix;
}

// Calculate slerp interpolation between two rotations defined by 3D axis-angle vectors
inline Vector3d slerp(const Vector3d &aa1, const Vector3d &aa2, double t)
{
    Quaterniond q1(AngleAxisd(aa1.norm(), aa1.normalized()));
    Quaterniond q2(AngleAxisd(aa2.norm(), aa2.normalized()));

    // Calculate slerp interpolation between q1 and q2
    Quaterniond q_interp = q1.slerp(t, q2);

    // Convert the interpolated quaternion to a 3D axis-angle vector
    AngleAxisd aa_interp(q_interp);
    Vector3d aa_interp_vec = aa_interp.axis() * aa_interp.angle();

    return aa_interp_vec;
}

inline Matrix3d skew(Vector3d vec)
{
    // calc skew symmetric
    Matrix3d skewSym;

    skewSym << 0.0, -vec(2), vec(1),
        vec(2), 0.0, -vec(0),
        -vec(1), vec(0), 0.0;

    return skewSym;
}

inline Matrix3d axang2rotm(Vector3d axang)
{
    if (axang.norm() < EPSILON_ROT)
        return Matrix3d::Identity();

    return skew(axang).exp();
}

inline Vector3d rotm2axang(Matrix3d rotm)
{

    Matrix3d skewSym = rotm.log();

    return Vector3d(skewSym(2, 1), skewSym(0, 2), skewSym(1, 0));
}

inline void randomGridDownsampling(PointCloud<PointXYZ>::Ptr rawPc, PointCloud<PointXYZ>& filteredPc, float gridSize)
{
    pcl::octree::OctreePointCloud<PointXYZ> octree(gridSize); // set voxel size

    // Set the input point cloud to the octree
    octree.setInputCloud(rawPc);

    // Construct the octree
    octree.addPointsFromInputCloud();

    // resize filtered pc
    filteredPc.resize(octree.getLeafCount());

    int filId = 0;
    double r;
    int id;

    // Use current time as seed for random generator
    srand(time(0));

    for (auto it = octree.leaf_depth_begin(); it != octree.leaf_depth_end(); ++it)
    {
        std::vector<int> indices;
        it.getLeafContainer().getPointIndices(indices);

        // get random number
        r = ((double)rand() / (RAND_MAX));

        id = static_cast<int>(r * (double)(indices.size() - 1));

        // add point
        filteredPc.points[filId] = rawPc->points[indices[id]];

        // update index
        ++filId;
    }
}

#endif
