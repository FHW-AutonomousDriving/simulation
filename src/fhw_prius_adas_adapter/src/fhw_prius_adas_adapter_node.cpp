#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <prius_msgs/msg/control.hpp>
#include <std_msgs/msg/float32.hpp>
#include <limits>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("fhw_prius_adas_adapter_node");
  
  // re-publish simulated Prius lidar to behave like real-world ADAS lidar
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher =
    node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan", 1);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription =
    node->create_subscription<sensor_msgs::msg::LaserScan>("/prius/center_laser/scan", 1,
    [&lidar_publisher](sensor_msgs::msg::LaserScan::UniquePtr msg){
      sensor_msgs::msg::LaserScan outmsg = *msg;
      for (size_t i = 71; i < 274; i++) { // real-world lidar does never show the insides of the car
        outmsg.ranges[i] = std::numeric_limits<float>::infinity();
      };
      for (float &distance : outmsg.ranges) {
        distance /= 10.0f; // scale
      };
      lidar_publisher->publish(outmsg);
    }
  );

  // re-publish simulated Prius speed
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher =
    node->create_publisher<std_msgs::msg::Float32>("/adas2019/odometry/speed", 1);
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription =
    node->create_subscription<std_msgs::msg::Float32>("/prius/speed", 1,
    [&speed_publisher](std_msgs::msg::Float32::UniquePtr msg) {
      std_msgs::msg::Float32 out;
      out.data = msg->data / 10.0f; // scale
      speed_publisher->publish(out);
    }
  );

  // forward real-world ADAS control messages to simulated Prius
  prius_msgs::msg::Control control_msg;
  rclcpp::Publisher<prius_msgs::msg::Control>::SharedPtr control_publisher =
    node->create_publisher<prius_msgs::msg::Control>("/prius/control", 1);
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_subscription =
    node->create_subscription<std_msgs::msg::Float32>("/adas2019/actuator/steering", 1,
    [&control_publisher, &control_msg](std_msgs::msg::Float32::UniquePtr msg) {
      control_msg.steer = msg->data / -100.0f; // scale
      control_publisher->publish(control_msg);
    }
  );
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_subscription =
    node->create_subscription<std_msgs::msg::Float32>("/adas2019/actuator/speed", 1,
    [&control_publisher, &control_msg](std_msgs::msg::Float32::UniquePtr msg) {
      control_msg.throttle = msg->data / 100.0f; // scale
      control_publisher->publish(control_msg);
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
