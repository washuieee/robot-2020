/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

// Grid Map
#include <grid_map_msgs/GridMap.h>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

#define ROUNDING(x, dig)    ( floor((x) * pow(float(10), dig) + 0.5f) / pow(float(10), dig) )

using namespace std;
using namespace grid_map;

namespace elevation_mapping {

ElevationMap::ElevationMap(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "variance"}),
      hasUnderlyingMap_(false)
{
  readParameters();
  rawMap_.setBasicLayers({"elevation"});
  clear();

  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
}

ElevationMap::~ElevationMap()
{
}

bool ElevationMap::readParameters()
{
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  setFrameId(frameId);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  setGeometry(length, resolution, position);

  nodeHandle_.param("min_variance", minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", maxHorizontalVariance_, 0.5);
  nodeHandle_.param("surface_normal_estimation_radius", surfaceNormalEstimationRadius_, 0.05);
  nodeHandle_.param("underlying_map_topic", underlyingMapTopic_, string());

  string surfaceNormalPositiveAxis;
  nodeHandle_.param("surface_normal_positive_axis", surfaceNormalPositiveAxis, string("z"));
  if (surfaceNormalPositiveAxis == "z") {
    surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  } else if (surfaceNormalPositiveAxis == "y") {
    surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitY();
  } else if (surfaceNormalPositiveAxis == "x") {
    surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitX();
  } else {
    ROS_ERROR("The surface normal positive axis '%s' is not valid.", surfaceNormalPositiveAxis.c_str());
  }
}

void ElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position)
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  ROS_INFO_STREAM("Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and "  << rawMap_.getSize()(1) << " columns.");
}

bool ElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances)
{
    rawMap_.get("elevation") = rawMap_.get("elevation").setConstant(NAN);
    for (unsigned int i = 0; i < pointCloud->size(); ++i)
    {
        auto& point = pointCloud->points[i];

        Index index;
        Position position(point.x, point.y);

        if (!rawMap_.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

        auto& elevation = rawMap_.at("elevation", index);
        auto& variance = rawMap_.at("variance", index);

        const float& pointVariance = pointCloudVariances(i);

        if (!rawMap_.isValid(index)) {
          // No prior information in elevation map, use measurement.
//          elevation = roundf ( point.z*100 ) / 100;
          elevation = point.z;
          variance = pointVariance;
          continue;
        }

        double mahalanobisDistance = sqrt(pow(point.z - elevation, 2) / variance);

        if (mahalanobisDistance > mahalanobisDistanceThreshold_) {
          // Add noise to cells which have ignored lower values,
          // such that outliers and moving objects are removed.
          variance += multiHeightNoise_;
          continue;
        }

        //Fuse measurement with elevation map data.
//        elevation = roundf ( ((variance * point.z + pointVariance * elevation) / (variance + pointVariance)) * 100 ) / 100;
        elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
        variance = (pointVariance * variance) / (pointVariance + variance);
    }

    clean();

    rawMap_.setTimestamp(1000 * pointCloud->header.stamp); // Point cloud stores time in microseconds.
    return true;
}

bool ElevationMap::clear()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  rawMap_.clearAll();
  rawMap_.resetTimestamp();

  return true;
}

bool ElevationMap::publishRawElevationMap()
{
  if (!hasRawMapSubscribers()) return false;
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
  grid_map::GridMap rawMapCopy = rawMap_;
  scopedLock.unlock(); 

  rawMapCopy.erase("variance");
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(rawMapCopy, message);
  elevationMapRawPublisher_.publish(message);

  ROS_DEBUG("Elevation map raw has been published.");

  return true;
}

grid_map::GridMap& ElevationMap::getRawGridMap()
{
  return rawMap_;
}

ros::Time ElevationMap::getTimeOfLastUpdate()
{
  return ros::Time().fromNSec(rawMap_.getTimestamp());
}

boost::recursive_mutex& ElevationMap::getRawDataMutex()
{
  return rawMapMutex_;
}

bool ElevationMap::clean()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.get("variance") = rawMap_.get("variance").unaryExpr(VarianceClampOperator<float>(minVariance_, maxVariance_));

  return true;
}

void ElevationMap::setFrameId(const std::string& frameId)
{
  rawMap_.setFrameId(frameId);
}

const std::string& ElevationMap::getFrameId()
{
  return rawMap_.getFrameId();
}

bool ElevationMap::hasRawMapSubscribers() const
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

bool ElevationMap::hasFusedMapSubscribers() const
{
  if (elevationMapFusedPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

void ElevationMap::underlyingMapCallback(const grid_map_msgs::GridMap& underlyingMap)
{
  ROS_INFO("Updating underlying map.");
  GridMapRosConverter::fromMessage(underlyingMap, underlyingMap_);
  if (underlyingMap_.getFrameId() != rawMap_.getFrameId()) {
    ROS_ERROR_STREAM("The underlying map does not have the same map frame ('" <<underlyingMap_.getFrameId()
                     << "') as the elevation map ('" << rawMap_.getFrameId() << "').");
    return;
  }
  if (!underlyingMap_.exists("elevation")) {
    ROS_ERROR_STREAM("The underlying map does not have an 'elevation' layer.");
    return;
  }

  underlyingMap_.setBasicLayers(rawMap_.getBasicLayers());
  hasUnderlyingMap_ = true;
  rawMap_.addDataFrom(underlyingMap_, false, false, true);
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

} /* namespace */
