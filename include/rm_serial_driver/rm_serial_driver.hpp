// 版权所有 (c) 2022 ChenJun
// 采用 Apache-2.0 许可证。

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// ROS 2 消息类型和库
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
// 第三方库
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ 标准库
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// 自定义消息类型
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_serial_driver
{

// RM 串口驱动类
class RMSerialDriver : public rclcpp::Node
{
public:
  // 构造函数，使用给定的节点选项创建 RMSerialDriver 实例
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  // 析构函数，释放资源
  ~RMSerialDriver() override;

private:
  void getParams();                                                // 获取参数
  void receiveData();                                              // 接收数据
  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);  // 发送数据
  void reopenPort();                                               // 重新打开串口
  void setParam(const rclcpp::Parameter & param);                  // 设置参数
  void resetTracker();                                             // 重置跟踪器

  // 串口配置
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // 参数客户端，用于设置检测颜色
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // 服务客户端，用于重置跟踪器
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // 用于可视化的串口接收瞄准点
  visualization_msgs::msg::Marker aiming_point_;

  // 广播从 odom 到 gimbal_link 的 tf
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 目标订阅者
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  // 用于调试的发布者
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // 接收线程
  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_