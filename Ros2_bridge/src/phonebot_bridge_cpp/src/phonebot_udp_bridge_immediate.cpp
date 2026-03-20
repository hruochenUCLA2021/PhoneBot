#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace {

constexpr char MAGIC[4] = {'P', 'B', 'O', 'T'};
constexpr uint8_t VERSION = 3;
constexpr uint8_t MSG_TYPE_SENSOR = 1;
constexpr uint8_t MSG_TYPE_MOTOR = 2;
constexpr uint8_t MSG_TYPE_TORQUE = 3;
constexpr uint8_t MSG_TYPE_MOTOR_STATUS = 4;
constexpr size_t MOTOR_COUNT = 13;

// Fixed sizes (must match Android + udp_protocol.py)
constexpr size_t SENSOR_SIZE = 116;
constexpr size_t MOTOR_SIZE = 176;
constexpr size_t TORQUE_SIZE = 21;  // header(20) + enable(u8)
constexpr size_t MOTOR_STATUS_SIZE = 332;  // header(20) + 6*13*4

inline uint16_t read_u16_le(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
inline uint32_t read_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline uint64_t read_u64_le(const uint8_t* p) {
  uint64_t lo = read_u32_le(p);
  uint64_t hi = read_u32_le(p + 4);
  return lo | (hi << 32);
}
inline float read_f32_le(const uint8_t* p) {
  uint32_t u = read_u32_le(p);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}
inline void write_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}
inline void write_u32_le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
inline void write_u64_le(uint8_t* p, uint64_t v) {
  write_u32_le(p, (uint32_t)(v & 0xFFFFFFFFULL));
  write_u32_le(p + 4, (uint32_t)((v >> 32) & 0xFFFFFFFFULL));
}
inline void write_f32_le(uint8_t* p, float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(uint32_t));
  write_u32_le(p, u);
}

struct SensorPkt {
  uint8_t version{0};
  uint16_t flags{0};
  uint32_t seq{0};
  uint64_t ts_ns{0};
  std::array<float, 3> accel{};
  std::array<float, 3> gyro{};
  float rot_hz{0.0f};
  std::array<float, 4> rot_quat_wxyz{};
  float game_hz{0.0f};
  std::array<float, 4> game_quat_wxyz{};
  float batt_percent{std::numeric_limits<float>::quiet_NaN()};
  uint8_t batt_status{0};
};

struct MotorPkt {
  uint32_t seq{0};
  uint64_t ts_ns{0};
  std::array<float, MOTOR_COUNT> pos{};
  std::array<float, MOTOR_COUNT> vel{};
  std::array<float, MOTOR_COUNT> tau{};
};

struct TorquePkt {
  uint32_t seq{0};
  uint64_t ts_ns{0};
  bool enable{false};
};

std::optional<SensorPkt> try_parse_sensor(const uint8_t* data, size_t n) {
  if (n < SENSOR_SIZE) return std::nullopt;
  if (std::memcmp(data, MAGIC, 4) != 0) return std::nullopt;
  uint8_t version = data[4];
  uint8_t msg_type = data[5];
  if (version != VERSION || msg_type != MSG_TYPE_SENSOR) return std::nullopt;

  SensorPkt pkt;
  pkt.version = version;
  pkt.flags = read_u16_le(data + 6);
  pkt.seq = read_u32_le(data + 8);
  pkt.ts_ns = read_u64_le(data + 12);

  size_t off = 20;  // header size
  for (int i = 0; i < 3; i++, off += 4) pkt.accel[i] = read_f32_le(data + off);
  for (int i = 0; i < 3; i++, off += 4) pkt.gyro[i] = read_f32_le(data + off);

  pkt.rot_hz = read_f32_le(data + off);
  off += 4;
  for (int i = 0; i < 4; i++, off += 4) pkt.rot_quat_wxyz[i] = read_f32_le(data + off);
  off += 12;  // rot_ypr_deg(3f) unused

  pkt.game_hz = read_f32_le(data + off);
  off += 4;
  for (int i = 0; i < 4; i++, off += 4) pkt.game_quat_wxyz[i] = read_f32_le(data + off);
  off += 12;  // game_ypr_deg(3f) unused

  pkt.batt_percent = read_f32_le(data + off);
  off += 4;
  // batt bytes: is_charging, plugged, status, reserved
  uint8_t batt_is_charging = data[off + 0];
  uint8_t batt_plugged = data[off + 1];
  (void)batt_is_charging;
  (void)batt_plugged;
  pkt.batt_status = data[off + 2];
  off += 4;
  return pkt;
}

std::optional<MotorPkt> try_parse_motor_v2(const uint8_t* data, size_t n) {
  if (n < MOTOR_SIZE) return std::nullopt;
  if (std::memcmp(data, MAGIC, 4) != 0) return std::nullopt;
  if (data[4] != VERSION || data[5] != MSG_TYPE_MOTOR) return std::nullopt;
  MotorPkt pkt;
  pkt.seq = read_u32_le(data + 8);
  pkt.ts_ns = read_u64_le(data + 12);
  size_t off = 20;
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) pkt.pos[i] = read_f32_le(data + off);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) pkt.vel[i] = read_f32_le(data + off);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) pkt.tau[i] = read_f32_le(data + off);
  return pkt;
}

std::optional<TorquePkt> try_parse_torque_v2(const uint8_t* data, size_t n) {
  if (n < TORQUE_SIZE) return std::nullopt;
  if (std::memcmp(data, MAGIC, 4) != 0) return std::nullopt;
  if (data[4] != VERSION || data[5] != MSG_TYPE_TORQUE) return std::nullopt;
  TorquePkt pkt;
  pkt.seq = read_u32_le(data + 8);
  pkt.ts_ns = read_u64_le(data + 12);
  pkt.enable = (data[20] != 0);
  return pkt;
}

std::vector<uint8_t> pack_motor(uint32_t seq, uint64_t ts_ns,
                                const std::vector<double>& pos,
                                const std::vector<double>& vel,
                                const std::vector<double>& tau) {
  std::vector<uint8_t> out(MOTOR_SIZE, 0);
  // header
  std::memcpy(out.data(), MAGIC, 4);
  out[4] = VERSION;
  out[5] = MSG_TYPE_MOTOR;
  write_u16_le(out.data() + 6, 0);      // flags
  write_u32_le(out.data() + 8, seq);    // seq
  write_u64_le(out.data() + 12, ts_ns); // ts_ns

  auto fix = [](const std::vector<double>& a) {
    std::array<float, MOTOR_COUNT> r{};
    r.fill(0.0f);
    size_t n = std::min(MOTOR_COUNT, a.size());
    for (size_t i = 0; i < n; i++) r[i] = (float)a[i];
    return r;
  };
  auto p = fix(pos);
  auto v = fix(vel);
  auto t = fix(tau);

  size_t off = 20;
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, v[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, t[i]);
  return out;
}

std::vector<uint8_t> pack_motor_status(
    uint32_t seq, uint64_t ts_ns,
    const std::vector<float>& pwm_percent,
    const std::vector<float>& load_percent,
    const std::vector<float>& pos_rad,
    const std::vector<float>& vel_rad_s,
    const std::vector<float>& vin_v,
    const std::vector<float>& temp_c) {
  std::vector<uint8_t> out(MOTOR_STATUS_SIZE, 0);
  std::memcpy(out.data(), MAGIC, 4);
  out[4] = VERSION;
  out[5] = MSG_TYPE_MOTOR_STATUS;
  write_u16_le(out.data() + 6, 0);
  write_u32_le(out.data() + 8, seq);
  write_u64_le(out.data() + 12, ts_ns);

  auto fix = [](const std::vector<float>& a) {
    std::array<float, MOTOR_COUNT> r{};
    r.fill(0.0f);
    size_t n = std::min(MOTOR_COUNT, a.size());
    for (size_t i = 0; i < n; i++) r[i] = a[i];
    return r;
  };

  auto p_pwm = fix(pwm_percent);
  auto p_load = fix(load_percent);
  auto p_pos = fix(pos_rad);
  auto p_vel = fix(vel_rad_s);
  auto p_vin = fix(vin_v);
  auto p_temp = fix(temp_c);

  size_t off = 20;
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p_pwm[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p_load[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p_pos[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p_vel[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p_vin[i]);
  for (size_t i = 0; i < MOTOR_COUNT; i++, off += 4) write_f32_le(out.data() + off, p_temp[i]);
  return out;
}

geometry_msgs::msg::Vector3 vec3_from_xyz(const std::array<float, 3>& v) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = (double)v[0];
  msg.y = (double)v[1];
  msg.z = (double)v[2];
  return msg;
}

geometry_msgs::msg::Quaternion quat_from_wxyz(const std::array<float, 4>& q) {
  geometry_msgs::msg::Quaternion msg;
  // Android sends [w,x,y,z]. ROS expects x,y,z,w.
  msg.x = (double)q[1];
  msg.y = (double)q[2];
  msg.z = (double)q[3];
  msg.w = (double)q[0];
  return msg;
}

int map_battery_status_code(uint8_t code) {
  using sensor_msgs::msg::BatteryState;
  if (code == 1) return BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  if (code == 2) return BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  if (code == 3) return BatteryState::POWER_SUPPLY_STATUS_FULL;
  if (code == 4) return BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  return BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
}

}  // namespace

class UdpToRos2ImmediateBridgeCpp : public rclcpp::Node {
 public:
  UdpToRos2ImmediateBridgeCpp() : Node("phonebot_udp_bridge_immediate_cpp") {
    this->declare_parameter<std::string>("bind_ip", "0.0.0.0");
    this->declare_parameter<int>("bind_port", 5005);
    this->declare_parameter<std::string>("topic_ns", "phonebot");
    // ROS topic to publish motor commands to (from Android -> UDP -> this bridge)
    this->declare_parameter<std::string>("motor_topic", "/phonebot/motor_cmd");
    // ROS topic to publish torque enable to (from Android -> UDP -> this bridge)
    this->declare_parameter<std::string>("torque_topic", "/phonebot/torque_enable");
    // ROS topic to subscribe motor state from (Pi HWdriver -> ROS2 -> this bridge -> UDP -> Android)
    this->declare_parameter<std::string>("motor_state_full_topic", "/phonebot/motor_state_full");
    this->declare_parameter<std::string>("android_ip", "192.168.20.2");
    this->declare_parameter<int>("android_port", 6006);

    auto bind_ip = this->get_parameter("bind_ip").as_string();
    auto bind_port = this->get_parameter("bind_port").as_int();
    auto topic_ns = this->get_parameter("topic_ns").as_string();
    if (!topic_ns.empty() && topic_ns.front() == '/') topic_ns.erase(0, 1);
    if (!topic_ns.empty() && topic_ns.back() == '/') topic_ns.pop_back();
    auto motor_topic = this->get_parameter("motor_topic").as_string();
    auto torque_topic = this->get_parameter("torque_topic").as_string();
    auto motor_state_full_topic = this->get_parameter("motor_state_full_topic").as_string();
    android_ip_ = this->get_parameter("android_ip").as_string();
    android_port_ = (int)this->get_parameter("android_port").as_int();

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/" + topic_ns + "/imu", 10);
    pub_imu_game_ = this->create_publisher<sensor_msgs::msg::Imu>("/" + topic_ns + "/imu_game", 10);
    pub_batt_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/" + topic_ns + "/battery", 10);
    // (Legacy) `motor_state` from SENSOR packets is no longer used in the current architecture.
    // pub_motor_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/" + topic_ns + "/motor_state", 10);
    pub_motor_cmd_ = this->create_publisher<sensor_msgs::msg::JointState>(motor_topic, 10);
    pub_torque_ = this->create_publisher<std_msgs::msg::Bool>(torque_topic, 10);
    motor_state_full_sub_ = this->create_subscription<motor_interfaces::msg::MotorState>(
        motor_state_full_topic, 10,
        std::bind(&UdpToRos2ImmediateBridgeCpp::on_motor_state_full, this, std::placeholders::_1));

    // RX socket
    rx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_fd_ < 0) throw std::runtime_error("failed to create UDP socket");

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)bind_port);
    addr.sin_addr.s_addr = inet_addr(bind_ip.c_str());

    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(rx_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (::bind(rx_fd_, (sockaddr*)&addr, sizeof(addr)) != 0) {
      throw std::runtime_error("failed to bind UDP socket to " + bind_ip + ":" + std::to_string(bind_port));
    }
    RCLCPP_INFO(this->get_logger(), "Listening UDP (immediate cpp) on %s:%d", bind_ip.c_str(), (int)bind_port);

    // TX socket
    tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_fd_ < 0) throw std::runtime_error("failed to create UDP tx socket");
    std::memset(&android_addr_, 0, sizeof(android_addr_));
    android_addr_.sin_family = AF_INET;
    android_addr_.sin_port = htons((uint16_t)android_port_);
    android_addr_.sin_addr.s_addr = inet_addr(android_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "Motor UDP target (immediate cpp): %s:%d", android_ip_.c_str(), android_port_);

    running_.store(true);
    rx_thread_ = std::thread([this]() { this->rx_loop(); });
  }

  ~UdpToRos2ImmediateBridgeCpp() override {
    running_.store(false);
    if (rx_fd_ >= 0) ::close(rx_fd_);
    if (tx_fd_ >= 0) ::close(tx_fd_);
    if (rx_thread_.joinable()) rx_thread_.join();
  }

 private:
  void rx_loop() {
    std::array<uint8_t, 65535> buf{};
    while (running_.load() && rclcpp::ok()) {
      sockaddr_in src{};
      socklen_t slen = sizeof(src);
      ssize_t n = ::recvfrom(rx_fd_, buf.data(), buf.size(), 0, (sockaddr*)&src, &slen);
      if (n <= 0) continue;
      if (auto s = try_parse_sensor(buf.data(), (size_t)n); s.has_value()) {
        publish_from_sensor(s.value());
        continue;
      }
      if (auto m = try_parse_motor_v2(buf.data(), (size_t)n); m.has_value()) {
        publish_motor_cmd(m.value());
        continue;
      }
      if (auto t = try_parse_torque_v2(buf.data(), (size_t)n); t.has_value()) {
        publish_torque(t.value());
        continue;
      }
    }
  }

  void publish_from_sensor(const SensorPkt& pkt) {
    auto stamp = this->now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "phone";
    imu.orientation = quat_from_wxyz(pkt.rot_quat_wxyz);
    imu.angular_velocity = vec3_from_xyz(pkt.gyro);
    imu.linear_acceleration = vec3_from_xyz(pkt.accel);
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    pub_imu_->publish(imu);

    sensor_msgs::msg::Imu imu_game;
    imu_game.header.stamp = stamp;
    imu_game.header.frame_id = "phone_game";
    imu_game.orientation = quat_from_wxyz(pkt.game_quat_wxyz);
    imu_game.angular_velocity = vec3_from_xyz(pkt.gyro);
    imu_game.linear_acceleration = vec3_from_xyz(pkt.accel);
    imu_game.orientation_covariance[0] = -1.0;
    imu_game.angular_velocity_covariance[0] = -1.0;
    imu_game.linear_acceleration_covariance[0] = -1.0;
    pub_imu_game_->publish(imu_game);

    sensor_msgs::msg::BatteryState b;
    b.header.stamp = stamp;
    b.header.frame_id = "phone";
    if (std::isfinite(pkt.batt_percent) && pkt.batt_percent >= 0.0f) {
      b.percentage = (float)pkt.batt_percent / 100.0f;
    } else {
      b.percentage = std::numeric_limits<float>::quiet_NaN();
    }
    b.power_supply_status = map_battery_status_code(pkt.batt_status);
    pub_batt_->publish(b);

    // No motor state is carried inside SENSOR packets in the current architecture.
  }

  void publish_motor_cmd(const MotorPkt& pkt) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name.resize(MOTOR_COUNT);
    msg.position.resize(MOTOR_COUNT);
    msg.velocity.resize(MOTOR_COUNT);
    msg.effort.resize(MOTOR_COUNT);
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
      msg.name[i] = "motor_" + std::to_string(i + 1);
      msg.position[i] = (double)pkt.pos[i];
      msg.velocity[i] = (double)pkt.vel[i];
      msg.effort[i] = (double)pkt.tau[i];
    }
    pub_motor_cmd_->publish(msg);
  }

  void publish_torque(const TorquePkt& pkt) {
    (void)pkt;
    std_msgs::msg::Bool msg;
    msg.data = pkt.enable;
    pub_torque_->publish(msg);
  }

  void on_motor_state_full(const motor_interfaces::msg::MotorState::SharedPtr msg) {
    // Forward present motor state to Android via UDP using MOTOR_STATUS packet.
    uint64_t ts_ns = (uint64_t)msg->header.stamp.sec * 1000000000ULL + (uint64_t)msg->header.stamp.nanosec;
    std::vector<float> pwm(msg->present_pwm_percent.begin(), msg->present_pwm_percent.end());
    std::vector<float> load(msg->present_load_percent.begin(), msg->present_load_percent.end());
    std::vector<float> pos(msg->present_position_rad.begin(), msg->present_position_rad.end());
    std::vector<float> vel(msg->present_velocity_rad_s.begin(), msg->present_velocity_rad_s.end());
    std::vector<float> vin(msg->present_input_voltage_v.begin(), msg->present_input_voltage_v.end());
    std::vector<float> temp(msg->present_temperature_c.begin(), msg->present_temperature_c.end());
    uint32_t seq = (tx_seq_.fetch_add(1) + 1);
    auto payload = pack_motor_status(seq, ts_ns, pwm, load, pos, vel, vin, temp);
    (void)::sendto(tx_fd_, payload.data(), payload.size(), 0, (sockaddr*)&android_addr_, sizeof(android_addr_));
  }

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_game_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_batt_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_motor_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_motor_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_torque_;
  rclcpp::Subscription<motor_interfaces::msg::MotorState>::SharedPtr motor_state_full_sub_;

  // UDP
  int rx_fd_{-1};
  int tx_fd_{-1};
  std::string android_ip_;
  int android_port_{6006};
  sockaddr_in android_addr_{};

  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  std::atomic<uint32_t> tx_seq_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpToRos2ImmediateBridgeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

