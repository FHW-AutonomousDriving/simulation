#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>
#include <std_msgs/Float32.h>
#include <prius_msgs/Control.h>
#include <gazebo_msgs/LinkStates.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fhw_prius_adas_adapter_node");
  ros::NodeHandle n;
  
  // re-publish simulated Prius lidar to behave like real-world ADAS lidar
  ros::Publisher lidar_publisher = n.advertise<sensor_msgs::LaserScan>("/lidar/scan", 1);
  ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/prius/center_laser/scan", 1, static_cast<boost::function<void (const sensor_msgs::LaserScan&)>>([&lidar_publisher](const sensor_msgs::LaserScan& msg) {
    sensor_msgs::LaserScan outmsg = msg;
    for (size_t i = 71; i < 274; i++) { // real-world lidar does never show the insides of the car
      outmsg.ranges[i] = std::numeric_limits<float>::infinity();
    };
    for (float &distance : outmsg.ranges) {
      distance /= 10.0f; // scale
    };
    lidar_publisher.publish(outmsg);
  }));

  // re-publish simulated Prius speed
  ros::Publisher speed_publisher = n.advertise<std_msgs::Float32>("/adas2019/odometry/speed", 1);
  ros::Subscriber speed_subscriber = n.subscribe<std_msgs::Float32>("/prius/speed", 1, static_cast<boost::function<void (const std_msgs::Float32&)>>([&speed_publisher](const std_msgs::Float32& msg){
    std_msgs::Float32 out;
    out.data = msg.data / 10.0f;
    speed_publisher.publish(out);
  }));

  // forward real-world ADAS control messages to simulated Prius
  ros::Publisher control_publisher = n.advertise<prius_msgs::Control>("/prius/control", 1);
  prius_msgs::Control control_msg;
  ros::Subscriber steering_subscriber = n.subscribe<std_msgs::Float32>("/adas2019/actuator/steering", 1, static_cast<boost::function<void (const std_msgs::Float32&)>>([&control_publisher, &control_msg](const std_msgs::Float32& msg){
    control_msg.steer = msg.data / -100.0f; // scale
    control_publisher.publish(control_msg);
  }));
  ros::Subscriber throttle_subscriber = n.subscribe<std_msgs::Float32>("/adas2019/actuator/speed", 1, static_cast<boost::function<void (const std_msgs::Float32&)>>([&control_publisher, &control_msg](const std_msgs::Float32& msg){
    control_msg.throttle = msg.data / 100.0f; // scale
    control_publisher.publish(control_msg);
  }));
  
  ros::spin();
  return 0;
}
