/*
 * VisualOdometryNode.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "VisualOdometryNode.h"

#include <vector>

#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace roamfree_visualodometry {

VisualOdometryNode::VisualOdometryNode() :
    _nh("~"), _filter(NULL), _imuHandler(NULL), _tracksHandler(NULL) {
  //setup roamfree
  initRoamfree();

  //setup the handlers
  _tracksHandler = new ROAMvision::FHPFeatureHandler(10.0);
  _tracksHandler->setTimestampOffsetTreshold(1.0 / 30.0 / 2.0);
  _imuHandler = new ImuHandler(_filter);

  //subscribe to sensor topics
  std::string imuTopic;
  if (!_nh.getParam("IMU_topic", imuTopic)) {
    ROS_FATAL("parameter IMU_topic undefined");
  }

  std::string magTopic;
  if (!_nh.getParam("MAG_topic", magTopic)) {
    ROS_FATAL("parameter MAG_topic undefined");
  }

  _imu_sub = _nh.subscribe(imuTopic, 60000, &VisualOdometryNode::imuCb, this);
  _mag_sub = _nh.subscribe(magTopic, 256, &VisualOdometryNode::magCb, this);

  _features_sub = _nh.subscribe("/MA/features", 60000,
      &VisualOdometryNode::featuresCb, this);

  _markers_pub = _nh.advertise<visualization_msgs::Marker>(
      "/visualization/features", 1);

  //Set all sensors as uninitialized
  isCameraInitialized = false;
  isIMUinitialized = false;

  /*
   ros::Time now = ros::Time(0);
   _tf_listener.waitForTransform(_camera_frame_id, imu_frame_id, now,
   ros::Duration(1.0));
   _tf_listener.lookupTransform(_camera_frame_id, imu_frame_id, now,
   _T_C_IMU);
   */

  // TODO: this is default transofrmation for which the camera looks towards the x axis of the IMU
  _T_R_IMU = tf::Transform(tf::Quaternion(0.5, -0.5, 0.5, 0.5),
      tf::Vector3(0.0, 0.0, 0.0));

}

VisualOdometryNode::~VisualOdometryNode() {
  if (_filter != NULL)

    delete _filter;

  if (_imuHandler)
    delete _imuHandler;

  if (_tracksHandler)
    delete _tracksHandler;

}

void VisualOdometryNode::run() {
  ros::Rate rate(5);

  while (ros::ok()) {

    rate.sleep();

    ros::spinOnce();

    if (isCameraInitialized && isIMUinitialized) {
      if (_tracksHandler->getNActiveFeatures() >= 3
          && _filter->getWindowLenght() > 1.0) {

        _filter->getNthOldestPose(0)->setFixed(true);

        ROS_INFO("Run estimation");
        bool ret = _filter->estimate(50);

        assert(ret);

        //filter->forgetOldNodes(2.0);
      }

      // publish stuff only if we have at least a pose in the filter
      if (_filter->getOldestPose()) {
        ROS_INFO("Publishing camera pose and features");

        publishFeatureMarkers();
        publishCameraPose();
      }
    }
  };

}

void VisualOdometryNode::imuCb(const sensor_msgs::Imu& msg) {
  //ROS_INFO("imu callback");

  if (!isIMUinitialized) {
    if (!isCameraInitialized)
      return;

    double t = msg.header.stamp.toSec();
    std::string imu_frame_id = msg.header.frame_id;

    tf::Transform T_W_R(tf::Quaternion(-0.5, 0.5, -0.5, 0.5),
        tf::Vector3(0.0, 0.0, 0.0));

    _imuHandler->init(_T_R_IMU, T_W_R, t);

    isIMUinitialized = true;
  }

  // fill temporaries with measurements
  double za[] = { msg.linear_acceleration.x, msg.linear_acceleration.y,
      msg.linear_acceleration.z };
  double zw[] = { msg.angular_velocity.x, msg.angular_velocity.y,
      msg.angular_velocity.z };

  _imuHandler->addInertialMeasurement(za, zw);

}

void VisualOdometryNode::magCb(const sensor_msgs::MagneticField& msg) {
  //ROS_INFO("magnetic callback");
  double t = msg.header.stamp.toSec();

  // fill temporaries with measurements
  double zh[] = { msg.magnetic_field.x, msg.magnetic_field.y,
      msg.magnetic_field.z };

  _imuHandler->addMagneticFieldMeasurement(zh);
}

void VisualOdometryNode::featuresCb(
    const libviso2_matcher_msgs::FeatureArray& msg) {
  ROS_INFO("Features callback");

  if (!isCameraInitialized) {
    //setup the camera
    initCamera(msg.header.frame_id);
  }

  static const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(2, 2);

  double t = msg.header.stamp.toSec();

  for (auto it = msg.features.begin(); it != msg.features.end(); ++it) {
    Eigen::VectorXd z(2);
    z << it->u, it->v;

    _tracksHandler->addFeatureObservation(it->id, t, z, cov);

  }
}

void VisualOdometryNode::initRoamfree() {
  _filter = ROAMestimation::FactorGraphFilterFactory::getNewFactorGraphFilter();
  _filter->setLowLevelLogging(true); // default log folder
  system("mkdir -p /tmp/roamfree/");
  system("rm -f /tmp/roamfree/*.log");
  _filter->setDeadReckoning(false);
  _filter->setSolverMethod(ROAMestimation::GaussNewton);
}

void VisualOdometryNode::initCamera(const std::string& frame_id) {

  _camera_frame_id = frame_id;

  //the camera intrinsic calibration matrix
  Eigen::VectorXd CM(9);

  //Quad
  //CM << 968.74432, 0.0, 391.2044, 0.0, 979.23602, 240.88485, 0.0, 0.0, 1.0;

  //ardone
  CM << 565.59102697808, 0.0, 337.839450567586, 0.0, 563.936510489792, 199.522081717361, 0.0, 0.0, 1.0;

  //PR2
  //CM << 384.69296, 0.0, 322.12556, 0.0, 384.69296, 252.2468, 0.0, 0.0, 1.0;

  Eigen::VectorXd T_OC(7);
  T_OC << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  _tracksHandler->init(_filter, "Track", T_OC, CM);

  isCameraInitialized = true;
}

void VisualOdometryNode::publishFeatureMarkers() {
  vector<long int> ids;

  _tracksHandler->getFeaturesIds(ids);

  visualization_msgs::Marker msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  //msg.lifetime = ros::Duration(0.2);
  msg.frame_locked = false;
  msg.ns = "roamfree_visualodometry";
  msg.id = 0;
  msg.action = visualization_msgs::Marker::ADD;

  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  msg.scale.x = 0.05;
  msg.scale.y = 0.05;
  msg.scale.z = 0.05;

  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;

  msg.pose.orientation.w = 1.0;

  msg.points.resize(ids.size());

  for (int k = 0; k < ids.size(); ++k) {
    Eigen::VectorXd fw(3);

    _tracksHandler->getFeaturePositionInWorldFrame(ids[k], fw);

    msg.points[k].x = fw(0);
    msg.points[k].y = fw(1);
    msg.points[k].z = fw(2);
  }

  _markers_pub.publish(msg);
}

void VisualOdometryNode::publishCameraPose() {
  ROAMestimation::PoseVertexWrapper_Ptr cameraPose_ptr =
      _filter->getNewestPose();
  const Eigen::VectorXd &camera = cameraPose_ptr->getEstimate();

  tf::Transform T_WR_tf(
      tf::Quaternion(camera(4), camera(5), camera(6), camera(3)),
      tf::Vector3(camera(0), camera(1), camera(2)));

  tf::Transform T_WIMU_tf = T_WR_tf * _T_R_IMU;

  _pose_tf_br.sendTransform(
      tf::StampedTransform(T_WR_tf, ros::Time(cameraPose_ptr->getTimestamp()),
          "world", "camera_link"));

  _pose_tf_br.sendTransform(
      tf::StampedTransform(T_WIMU_tf, ros::Time(cameraPose_ptr->getTimestamp()),
          "world", "imu_link"));

}

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "roamfree_full_slam_imu");

  roamfree_visualodometry::VisualOdometryNode n;

  ROS_INFO("Localization node started");
  n.run();
  ROS_INFO("Localization node shut down");

  return 0;
}
