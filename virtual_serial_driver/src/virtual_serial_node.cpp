#include "virtual_serial_driver/virtual_serial_node.hpp"

namespace fyt::serial_driver {

VirtualSerialNode::VirtualSerialNode(const rclcpp::NodeOptions &options)
  : Node("serial_driver", options) {
  // 初始化日志
  FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
  FYT_INFO("serial_driver", "Starting VirtualSerialNode (no real UART) !");

  // 创建发布器
  serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", 10);

  // 创建TF广播器
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 声明参数（可通过launch/配置文件覆盖）
// 声明参数（仅声明未在launch中定义的，避免重复）
this->declare_parameter("vision_mode", static_cast<int>(1));          // 视觉模式
this->declare_parameter("roll", 0.0);                                // 云台roll角
this->declare_parameter("pitch", 0.0);                               // 云台pitch角
this->declare_parameter("yaw", 0.0);                                 // 云台yaw角
this->declare_parameter("bullet_speed", 25.0);                        // 子弹速度
// 注释掉has_rune的declare_parameter（由launch传参）
// this->declare_parameter("has_rune", true);                           
this->declare_parameter("target_frame", std::string("odom"));         // TF父坐标系

  // 获取参数
  //has_rune_ = this->declare_parameter("has_rune", true);
  this->declare_parameter("has_rune", true); // 恢复声明（避免参数系统异常）
  has_rune_ = this->get_parameter("has_rune").as_bool(); // 用get_parameter读取值

  std::string target_frame = this->get_parameter("target_frame").as_string();

  // 初始化固定消息
  serial_receive_data_msg_.header.frame_id = target_frame;
  serial_receive_data_msg_.bullet_speed = this->get_parameter("bullet_speed").as_double();
  serial_receive_data_msg_.mode = this->get_parameter("vision_mode").as_int();
  serial_receive_data_msg_.roll = this->get_parameter("roll").as_double();
  serial_receive_data_msg_.pitch = this->get_parameter("pitch").as_double();
  serial_receive_data_msg_.yaw = this->get_parameter("yaw").as_double();

  // 初始化TF消息
  transform_stamped_.header.frame_id = target_frame;
  transform_stamped_.child_frame_id = "gimbal_link";

  // 初始化心跳
  heartbeat_ = HeartBeatPublisher::create(this);

  // 创建模式设置客户端（适配自瞄/能量机关模块）
  auto autoaim_detector_client = this->create_client<rm_interfaces::srv::SetMode>(
    "armor_detector/set_mode");
  set_mode_clients_.emplace(autoaim_detector_client->get_service_name(), autoaim_detector_client);

  auto autoaim_solver_client = this->create_client<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode");
  set_mode_clients_.emplace(autoaim_solver_client->get_service_name(), autoaim_solver_client);

  // 能量机关模块客户端（可选）
  if (has_rune_) {
    auto rune_detector_client = this->create_client<rm_interfaces::srv::SetMode>(
      "rune_detector/set_mode");
    set_mode_clients_.emplace(rune_detector_client->get_service_name(), rune_detector_client);

    auto rune_solver_client = this->create_client<rm_interfaces::srv::SetMode>(
      "rune_solver/set_mode");
    set_mode_clients_.emplace(rune_solver_client->get_service_name(), rune_solver_client);
  }

  // 1ms定时器循环发布数据
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    [this]() {
      // 更新时间戳
      serial_receive_data_msg_.header.stamp = this->now();
      
      // 动态读取参数（支持运行时修改）
      int mode = this->get_parameter("vision_mode").as_int();
      double roll = this->get_parameter("roll").as_double();
      double pitch = this->get_parameter("pitch").as_double();
      double yaw = this->get_parameter("yaw").as_double();
      
      // 更新消息数据
      serial_receive_data_msg_.mode = mode;
      serial_receive_data_msg_.roll = roll;
      serial_receive_data_msg_.pitch = pitch;
      serial_receive_data_msg_.yaw = yaw;

      // 发布串口数据
      serial_receive_data_pub_->publish(serial_receive_data_msg_);

      // 计算云台TF变换
      tf2::Quaternion q;
      q.setRPY(roll * M_PI / 180.0, -pitch * M_PI / 180.0, yaw * M_PI / 180.0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.stamp = this->now();
      transform_stamped_.child_frame_id = "gimbal_link";
      tf_broadcaster_->sendTransform(transform_stamped_);

      // 发布odom_rectify TF（仅roll角）
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy = utils::getRPY(q_eigen.toRotationMatrix());
      q.setRPY(rpy[0], 0, 0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.child_frame_id = transform_stamped_.header.frame_id + "_rectify";
      tf_broadcaster_->sendTransform(transform_stamped_);

      // 模式切换逻辑
      for (auto &[service_name, client] : set_mode_clients_) {
        if (client.mode.load() != mode && !client.on_waiting.load()) {
          setMode(client, mode);
        }
      }
    });
}

void VirtualSerialNode::setMode(SetModeClient &client, const uint8_t mode) {
  using namespace std::chrono_literals;
  std::string service_name = client.ptr->get_service_name();

  // 等待服务可用
  while (!client.ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      FYT_ERROR("serial_driver", "Interrupted while waiting for service {}! Exiting.", service_name);
      return;
    }
    FYT_INFO("serial_driver", "Service {} not available, waiting...", service_name);
  }

  if (!client.ptr->service_is_ready()) {
    FYT_WARN("serial_driver", "Service {} is not ready!", service_name);
    return;
  }

  // 发送模式设置请求
  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;
  client.on_waiting.store(true);

  // 异步发送请求并处理响应
  auto result = client.ptr->async_send_request(
    req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
        FYT_INFO("serial_driver", "Set mode {} for service success!", mode);
      } else {
        FYT_WARN("serial_driver", "Set mode {} for service failed!", mode);
      }
    });
}

}  // namespace fyt::serial_driver

// 注册ROS2组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::serial_driver::VirtualSerialNode)
