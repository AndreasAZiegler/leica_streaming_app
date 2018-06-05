#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "leica_streaming_app/tcp_total_station_interface.h"

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
  void locationTSCallback(double x, double y, double z);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber pos_sub_;
  ros::Publisher prism_pos_pub_;

  TCPTSInterface ts_;
};
} // namespace leia_streaming_app
