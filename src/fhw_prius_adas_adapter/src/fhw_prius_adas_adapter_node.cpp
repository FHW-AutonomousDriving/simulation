#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <limits>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fhw_prius_adas_adapter_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/lidar/scan", 1);
  boost::function<void (const sensor_msgs::LaserScan&)> callback = [pub](const sensor_msgs::LaserScan& msg) {
    sensor_msgs::LaserScan outmsg = msg;
    for (size_t i = 0; i < 32; i++) {
      outmsg.ranges[i] = std::numeric_limits<float>::infinity();
    };
    for (size_t i = outmsg.ranges.size()-32; i < outmsg.ranges.size(); i++) {
      outmsg.ranges[i] = std::numeric_limits<float>::infinity();
    };
    pub.publish(outmsg);
  };
  auto sub = n.subscribe<sensor_msgs::LaserScan>("/prius/center_laser/scan", 1, callback);
  ros::spin();
  return 0;
}
