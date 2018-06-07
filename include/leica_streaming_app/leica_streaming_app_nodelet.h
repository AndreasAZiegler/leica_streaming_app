#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "leica_streaming_app/serial_total_station_interface.h"

namespace leica_streaming_app {

class LeicaStreamingAppNodelet : public nodelet::Nodelet {
 public:
  LeicaStreamingAppNodelet();
  ~LeicaStreamingAppNodelet();

 private:
  virtual void onInit();
  void connectCb();
  void disconnectCb();
  void positionCb(const geometry_msgs::PointStamped::ConstPtr& msg);
  void startStopCb(const std_msgs::Bool::ConstPtr& msg);
  void locationTSCallback(double x, double y, double z);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber pos_sub_;
  ros::Subscriber start_stop_sub_;
  ros::Publisher prism_pos_pub_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped transformStamped_;
  tf2::Quaternion q_;

  SerialTSInterface ts_;
};
} // namespace leia_streaming_app
