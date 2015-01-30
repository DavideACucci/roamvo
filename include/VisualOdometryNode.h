/*
 * FullSlamImu.h
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#ifndef FULLSLAMIMU_H_
#define FULLSLAMIMU_H_

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <libviso2_matcher_msgs/FeatureArray.h>

#include "ImuHandler.h"
#include <ROAMvision/ROAMvision.h>
#include <ROAMestimation/ROAMestimation.h>

namespace roamfree_visualodometry {

class VisualOdometryNode {

public:

	VisualOdometryNode();
	virtual ~VisualOdometryNode();
	void run();

protected:
	void imuCb(const sensor_msgs::Imu &msg);
	void magCb(const sensor_msgs::MagneticField &msg);
	void featuresCb(const libviso2_matcher_msgs::FeatureArray &msg);

	void publishFeatureMarkers();
	void publishCameraPose();

private:
	void initRoamfree();
	void initCamera(const std::string& frame_id);

private:
	//ros stuff
	ros::NodeHandle _nh;

	ros::Subscriber _features_sub, _imu_sub, _mag_sub;
	ros::Publisher _markers_pub;

	//Transform handler
	tf::TransformBroadcaster _pose_tf_br;
	tf::TransformListener _tf_listener;

	tf::Transform _T_R_IMU;
	std::string _camera_frame_id;

	//Variables for initialization
	bool isCameraInitialized;
	bool isIMUinitialized;

	//Roamfree objects
	ROAMestimation::FactorGraphFilter* _filter;
	ImuHandler* _imuHandler;
	ROAMvision::ImageFeatureHandler *_tracksHandler;
};

} /* namespace roamfree_c_slam */

#endif /* FULLSLAMIMU_H_ */
