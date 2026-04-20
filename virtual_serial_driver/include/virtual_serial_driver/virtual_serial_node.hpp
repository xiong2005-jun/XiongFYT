#ifndef VIRTUAL_SERIAL_DRIVER_VIRTUAL_SERIAL_NODE_HPP_
#define VIRTUAL_SERIAL_DRIVER_VIRTUAL_SERIAL_NODE_HPP_

// STD
#include <chrono>
#include <memory>
#include <unordered_map>
#include <atomic>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 项目接口
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::serial_driver {

// 模式设置客户端结构体
struct SetModeClient {
  SetModeClient(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr p) : ptr(p) {}
  std::atomic<bool> on_waiting = false;
  std::atomic<int> mode = 0;
  rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
};

class VirtualSerialNode : public rclcpp::Node {
public:
  explicit VirtualSerialNode(const rclcpp::NodeOptions &options);

private:
  // 发送模式设置请求
  void setMode(SetModeClient &client, const uint8_t mode);

  // 心跳发布器
  HeartBeatPublisher::SharedPtr heartbeat_;
  // TF广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // 串口数据发布器
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_pub_;
  // 定时器（1ms循环发布数据）
  rclcpp::TimerBase::SharedPtr timer_;
  // 固定数据消息
  rm_interfaces::msg::SerialReceiveData serial_receive_data_msg_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  // 是否包含能量机关模块
  bool has_rune_;
  // 模式设置客户端映射
  std::unordered_map<std::string, SetModeClient> set_mode_clients_;
};

}  // namespace fyt::serial_driver

#endif  // VIRTUAL_SERIAL_DRIVER_VIRTUAL_SERIAL_NODE_HPP_
