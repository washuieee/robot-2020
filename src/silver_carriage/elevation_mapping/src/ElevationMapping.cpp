/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_mapping/ElevationMapping.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/sensor_processors/KinectSensorProcessor.hpp"
//#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
//#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
//#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>

using namespace std;
using namespace grid_map;
using namespace ros;
using namespace tf;
using namespace pcl;

namespace elevation_mapping {

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle),
      isContinouslyFusing_(false),
      ignoreRobotMotionUpdates_(true)
{
  ROS_INFO("Elevation mapping node started.");

  readParameters();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
}

ElevationMapping::~ElevationMapping()
{
  fusionServiceQueue_.clear();
  fusionServiceQueue_.disable();
  nodeHandle_.shutdown();
  fusionServiceThread_.join();
}

bool ElevationMapping::readParameters()
{
  // ElevationMapping parameters.
  nodeHandle_.param("point_cloud_topic", pointCloudTopic_, string("/points"));
  nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string(""));
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/robot"));
  nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 0);
  ROS_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 15.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  nodeHandle_.param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);

  double fusedMapPublishingRate;
  nodeHandle_.param("fused_map_publishing_rate", fusedMapPublishingRate, 0.0);
  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_.fromSec(0.0);
    ROS_WARN("Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is called.");
  } else if (std::isinf(fusedMapPublishingRate)){
    isContinouslyFusing_ = true;
    fusedMapPublishTimerDuration_.fromSec(0.0);
  } else {
    fusedMapPublishTimerDuration_.fromSec(1.0 / fusedMapPublishingRate);
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  map_.setFrameId(frameId);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  map_.setGeometry(length, resolution, position);

  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);
  nodeHandle_.param("surface_normal_estimation_radius", map_.surfaceNormalEstimationRadius_, 0.05);
  nodeHandle_.param("underlying_map_topic", map_.underlyingMapTopic_, string());

  string surfaceNormalPositiveAxis;
  nodeHandle_.param("surface_normal_positive_axis", surfaceNormalPositiveAxis, string("z"));
  if (surfaceNormalPositiveAxis == "z") {
    map_.surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  } else if (surfaceNormalPositiveAxis == "y") {
    map_.surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitY();
  } else if (surfaceNormalPositiveAxis == "x") {
    map_.surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitX();
  } else {
    ROS_ERROR("The surface normal positive axis '%s' is not valid.", surfaceNormalPositiveAxis.c_str());
  }

  // SensorProcessor parameters.
  string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, string("Kinect"));
  if (sensorType == "Kinect") {
    sensorProcessor_.reset(new KinectSensorProcessor(nodeHandle_, transformListener_));
  } /*else if (sensorType == "Stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "Laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "Perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, transformListener_));
  } */else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) return false;

  return true;
}


void ElevationMapping::pointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{
  stopMapUpdateTimer();

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(rawPointCloud, pcl_pc);
  PointCloud<PointXYZRGB>::Ptr pointCloud(new PointCloud<PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  ros::Time time;
  time.fromNSec(1000 * pointCloud->header.stamp);

  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!ignoreRobotMotionUpdates_) {
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
    if (!poseMessage) {
      ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloud<PointXYZRGB>::Ptr pointCloudProcessed(new PointCloud<PointXYZRGB>);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed,
                                 measurementVariances)) {
    ROS_ERROR("Point cloud could not be processed.");
//    resetMapUpdateTimer();
    return;
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances)) {
    ROS_ERROR("Adding point cloud to elevation map failed.");
//    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.publishRawElevationMap();

  resetMapUpdateTimer();
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  if (ignoreRobotMotionUpdates_) return true;

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  return true;
}

bool ElevationMapping::updateMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = ros::Time(0);
//  convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  return true;
}

bool ElevationMapping::clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Clearing map.");
  return map_.clear();
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace */
