/*
g++ main.cpp -lboost_system -lboost_thread -lpthread -o leica_streaming_receiver
*/

#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

#include "leica_streaming_app/leica_streaming_app_tcp_nodelet.h"

namespace leica_streaming_app {

LeicaStreamingAppTCPNodelet::LeicaStreamingAppTCPNodelet()
  : ts_(std::bind(&LeicaStreamingAppTCPNodelet::locationTSCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3)) {
}

LeicaStreamingAppTCPNodelet::~LeicaStreamingAppTCPNodelet() {
}

void LeicaStreamingAppTCPNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  std::string ip;
  int port;
  private_nh_.param<std::string>("ip", ip, "10.2.86.54");
  private_nh_.param("port", port, 5001);
  ts_.connect(ip, port);

  prism_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/leica/position", 10,
      boost::bind(&LeicaStreamingAppTCPNodelet::connectCb, this),
      boost::bind(&LeicaStreamingAppTCPNodelet::disconnectCb, this));
}

void LeicaStreamingAppTCPNodelet::connectCb() {
  if (!prism_pos_pub_ && prism_pos_pub_.getNumSubscribers() > 0) {
    NODELET_INFO("Connecting to odom/vicon position topic.");
    pos_sub_ = nh_.subscribe("/paintcopter/position", 10, &LeicaStreamingAppTCPNodelet::positionCb, this);
    start_stop_sub_ = nh_.subscribe("/leica/start_stop", 10, &LeicaStreamingAppTCPNodelet::startStopCb, this);
  }
}

void LeicaStreamingAppTCPNodelet::disconnectCb() {
  if (prism_pos_pub_.getNumSubscribers() == 0) {
    NODELET_INFO("Unsubscribing from odom/vison position topic.");
    pos_sub_.shutdown();
    start_stop_sub_.shutdown();
  }
}

void LeicaStreamingAppTCPNodelet::positionCb(const nav_msgs::Odometry::ConstPtr& msg) {
  ts_.setPrismPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void LeicaStreamingAppTCPNodelet::startStopCb(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    ts_.start();
  } else {
    ts_.end();
  }
}

void LeicaStreamingAppTCPNodelet::locationTSCallback(const double x,
                                                  const double y,
                                                  const double z) {
  /*
  std::cout << "Prism is at x: " << x 
            << " y: " << y
            << " z: " << z << std::endl;
  std::cout << std::endl;
  */
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;

  prism_pos_pub_.publish(msg);

  transformStamped_.header.stamp = ros::Time::now();
  transformStamped_.header.frame_id = "world";
  transformStamped_.child_frame_id = "leica_pos";
  transformStamped_.transform.translation.x = x;
  transformStamped_.transform.translation.y = y;
  transformStamped_.transform.translation.z = z;

  q_.setRPY(0, 0, 0);
  transformStamped_.transform.rotation.x = q_.x();
  transformStamped_.transform.rotation.y = q_.y();
  transformStamped_.transform.rotation.z = q_.z();
  transformStamped_.transform.rotation.w = q_.w();

  br_.sendTransform(transformStamped_);

}

} // namespace leica_streaming_app
PLUGINLIB_DECLARE_CLASS(leica_streaming_app, LeicaStreamingAppTCPNodelet, leica_streaming_app::LeicaStreamingAppTCPNodelet, nodelet::Nodelet);
