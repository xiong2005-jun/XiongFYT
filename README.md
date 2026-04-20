FYT改
1. 修改串口的函数
void BaseController::serial2joint() {
    static float temp_yaw = 0.0f, last_yaw = 0.0f;
    static int rount = 0;

    temp_yaw = gimbal_receive.yaw;   // 角度制

    // 跨圈处理（保证 yaw 连续）
    if ((last_yaw - temp_yaw) < -180.0f) {
        rount--;
    } else if ((last_yaw - temp_yaw) > 180.0f) {
        rount++;
    }
    float yaw_continuous = rount * 360.0f + temp_yaw;  // 连续角度（度）
    last_yaw = temp_yaw;

    // 创建 JointState 消息
    sensor_msgs::msg::JointState joint{};
    joint.header.stamp = this->get_clock()->now();
    joint.name = {"pitch_joint", "yaw_joint"};
    // 转换为弧度
    joint.position = {
        gimbal_receive.pitch * M_PI / 180.0f,
        yaw_continuous * M_PI / 180.0f
    };
    joint_pub->publish(joint);
}

2. 修改转发适配串口

#1）在头文件 .hpp 添加消息与发布器

#include "def_msg/msg/gimble_control.hpp"
// 类内添加
rclcpp::Publisher<def_msg::msg::GimbleControl>::SharedPtr vision_gimbal_pub_;


#2）在构造函数 .cpp 初始化发布器

vision_gimbal_pub_ = this->create_publisher<def_msg::msg::GimbleControl>("/vision/gimble_control", 10);

#//=======修改以下函数=====================
void ArmorSolverNode::timerCallback() {
  if (solver_ == nullptr) {
    return;
  }

  if (!enable_) {
    return;
  }

  // Init message
  rm_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);

    // ====================== 直接发 /vision/gimble_control ======================
    def_msg::msg::GimbleControl vision_msg;
    vision_msg.header = control_msg.header;
    vision_msg.yaw = control_msg.yaw;
    vision_msg.pitch = control_msg.pitch;
    vision_msg.fire_advice = control_msg.fire_advice ? 1 : 0;
    vision_gimbal_pub_->publish(vision_msg);
    // ========================================================================

    return;
  }

  if (armor_target_.tracking) {
    try {
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_);
    } catch (...) {
      FYT_ERROR("armor_solver", "Something went wrong in solver!");
      control_msg.yaw_diff = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance = -1;
      control_msg.fire_advice = false;
    }
  } else {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.fire_advice = false;
  }

  gimbal_pub_->publish(control_msg);

  // ====================== 直接发 /vision/gimble_control ======================
  def_msg::msg::GimbleControl vision_msg;
  vision_msg.header = control_msg.header;
  vision_msg.yaw = control_msg.yaw;
  vision_msg.pitch = control_msg.pitch;
  vision_msg.fire_advice = control_msg.fire_advice ? 1 : 0;
  vision_gimbal_pub_->publish(vision_msg);
  // ========================================================================

  if (debug_mode_) {
    publishMarkers(armor_target_, control_msg);
  }
}

#3）修改cmakelist
#4) 修改launch
。。。。。

