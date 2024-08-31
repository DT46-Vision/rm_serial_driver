// 版权所有 (c) 2022 ChenJun
// 根据 Apache-2.0 许可证授权。

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ 系统库
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "启动 RMSerialDriver!");

  getParams();

  // TF 广播器
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 创建发布者
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  // 检测参数客户端
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // 跟踪器重置服务客户端
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    // 初始化串口
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open(); // 打开串口
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this); // 启动接收数据的线程
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "创建串口时出错: %s - %s", device_name_.c_str(), ex.what());
    throw ex; // 抛出异常
  }

  aiming_point_.header.frame_id = "odom"; // 设置坐标系
  aiming_point_.ns = "aiming_point"; // 设置命名空间
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE; // 设置标记类型为球体
  aiming_point_.action = visualization_msgs::msg::Marker::ADD; // 添加标记
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12; // 设置标记尺寸
  aiming_point_.color.r = 1.0; // 设置颜色为白色
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0; // 透明度
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1); // 设置标记的生命周期

  // 创建订阅者
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1)); // 绑定回调函数
}
RMSerialDriver::~RMSerialDriver()
{
  // 如果接收线程可连接，则加入线程
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  // 如果串口打开，则关闭串口
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  // 如果拥有的上下文存在，等待退出
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1); // 定义头部
  std::vector<uint8_t> data; // 定义数据
  data.reserve(sizeof(ReceivePacket)); // 预留接收数据的空间

  while (rclcpp::ok()) { // 当节点正常运行时
    try {
      serial_driver_->port()->receive(header); // 接收头部

      if (header[0] == 0x5A) { // 检查头部是否有效
        data.resize(sizeof(ReceivePacket) - 1); // 调整数据大小
        serial_driver_->port()->receive(data); // 接收数据

        data.insert(data.begin(), header[0]); // 插入头部到数据开头
        ReceivePacket packet = fromVector(data); // 从数据向量转换为接收包
        
        // 验证 CRC 校验
        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // 如果是初始参数设置或接收到的颜色与之前不同，更新参数
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color)); // 设置颜色参数
            previous_receive_color_ = packet.detect_color; // 更新之前接收的颜色
          }

          // 如果需要重置跟踪器，调用重置函数
          if (packet.reset_tracker) {
            resetTracker();
          }

          // 创建变换信息
          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom"; // 设置父坐标系
          t.child_frame_id = "gimbal_link"; // 设置子坐标系
          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw); // 设置四元数
          t.transform.rotation = tf2::toMsg(q); // 转换为消息格式
          tf_broadcaster_->sendTransform(t); // 广播变换信息

          // 如果目标坐标有效，则发布标记
          if (abs(packet.aim_x) > 0.01) {
            aiming_point_.header.stamp = this->now(); // 设置时间戳
            aiming_point_.pose.position.x = packet.aim_x; // 设置目标位置
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_); // 发布标记
          }
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!"); // CRC 校验失败
        }
      } else {
        // 如果头部无效，记录警告
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      // 捕获异常并重新打开串口
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  // 定义 ID 和对应的 uint8_t 映射
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet; // 创建发送包
    packet.tracking = msg->tracking; // 设置跟踪状态
    packet.id = id_unit8_map.at(msg->id); // 获取 ID
    packet.armors_num = msg->armors_num; // 设置装甲数量
    packet.x = msg->position.x; // 设置坐标
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    packet.yaw = msg->yaw; // 设置偏航
    packet.vx = msg->velocity.x; // 设置速度
    packet.vy = msg->velocity.y;
    packet.vz = msg->velocity.z;
    packet.v_yaw = msg->v_yaw; // 设置偏航速度
    packet.r1 = msg->radius_1; // 设置半径
    packet.r2 = msg->radius_2;
    packet.dz = msg->dz; // 设置深度
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet)); // 添加 CRC 校验

    std::vector<uint8_t> data = toVector(packet); // 将包转换为数据向量

    serial_driver_->port()->send(data); // 发送数据

    std_msgs::msg::Float64 latency; // 创建延迟消息
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0; // 计算延迟
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms"); // 记录延迟
    latency_pub_->publish(latency); // 发布延迟
  } catch (const std::exception & ex) {
    // 捕获异常并重新打开串口
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl; // 流控类型
  using Parity = drivers::serial_driver::Parity; // 校验位类型
  using StopBits = drivers::serial_driver::StopBits; // 停止位类型

  uint32_t baud_rate{}; // 波特率
  auto fc = FlowControl::NONE; // 默认流控
  auto pt = Parity::NONE; // 默认校验位
  auto sb = StopBits::ONE; // 默认停止位

  try {
    device_name_ = declare_parameter<std::string>("device_name", ""); // 获取设备名称参数
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid"); // 记录错误
    throw ex; // 抛出异常
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0); // 获取波特率参数
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid"); // 记录错误
    throw ex; // 抛出异常
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", ""); // 获取流控参数

    // 根据字符串设置流控类型
    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."}; // 抛出参数不合法异常
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid"); // 记录错误
    throw ex; // 抛出异常
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", ""); // 获取校验位参数

    // 根据字符串设置校验位类型
    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."}; // 抛出参数不合法异常
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid"); // 记录错误
    throw ex; // 抛出异常
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", ""); // 获取停止位参数

    // 根据字符串设置停止位类型
    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."}; // 抛出参数不合法异常
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid"); // 记录错误
    throw ex; // 抛出异常
  }

  // 创建串口配置
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port"); // 记录尝试重新打开端口的警告
  try {
    // 如果串口打开，则关闭串口
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open(); // 重新打开端口
    RCLCPP_INFO(get_logger(), "Successfully reopened port"); // 记录成功信息
  } catch (const std::exception & ex) {
    // 捕获异常并记录错误
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1)); // 等待 1 秒后重试
      reopenPort(); // 递归调用重新打开端口
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  // 如果检测器参数客户端未准备好，记录警告并跳过设置
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  // 如果没有有效的设置参数的未来或已经准备好，则设置参数
  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int()); // 记录设置颜色参数的信息
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        // 遍历结果并检查是否成功
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str()); // 记录失败信息
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int()); // 记录成功信息
        initial_set_param_ = true; // 标记为初始设置完成
      });
  }
}

void RMSerialDriver::resetTracker()
{
  // 如果重置跟踪器服务未准备好，记录警告并跳过重置
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); // 创建请求
  reset_tracker_client_->async_send_request(request); // 异步发送请求
  RCLCPP_INFO(get_logger(), "Reset tracker!"); // 记录重置跟踪器的信息
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// 使用类加载器注册组件。
// 这作为一种入口点，使组件在其库被加载到运行中的进程时可被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)