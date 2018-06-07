#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv) {
  // Getting the IP address
  /*
  std::string ip;
  std::cout << "Please enter the simulator's IP address: ";
  std::cin >> ip;

  int port = 5001;
  */

  ros::init(argc, argv, "leica_streaming_tcp_app_node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "leica_streaming_app/leica_streaming_app_nodelet", remap, nargv);
  ros::spin();

  /*
  std::string ch;
  while (true) {
    getline(std::cin, ch);

    if (!ch.empty()) {
      std::cout << ch << std::endl;
      switch (ch.c_str()[0]) {
        case 'e':
          ts.end();
          std::cout << "Measurements stopped." << std::endl;
          break;
        case 's':
          ts.start();
          break;
      }
    }
  }
  */

  return 0;
}
