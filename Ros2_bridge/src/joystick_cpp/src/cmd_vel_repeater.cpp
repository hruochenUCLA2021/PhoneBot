#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

struct Cmd {
  float vx{0.f};
  float vy{0.f};
  float wz{0.f};
};

class StdinRaw {
 public:
  StdinRaw() : fd_(::fileno(stdin)), active_(false) {
    if (fd_ < 0) return;
    if (::tcgetattr(fd_, &old_) != 0) return;
    termios raw = old_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &raw) == 0) active_ = true;
  }

  ~StdinRaw() {
    if (active_) {
      (void)::tcsetattr(fd_, TCSANOW, &old_);
    }
  }

 private:
  int fd_;
  termios old_;
  bool active_;
};

class CmdVelRepeater : public rclcpp::Node {
 public:
  CmdVelRepeater() : Node("phonebot_cmd_vel_repeater_cpp") {
    this->declare_parameter<std::string>("mode", "joystick");  // joystick | keyboard
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel");
    this->declare_parameter<std::string>("cmd_vel_out_topic", "/phonebot/cmd_vel");
    this->declare_parameter<double>("vx_max", 0.5);
    this->declare_parameter<double>("vy_max", 0.25);
    this->declare_parameter<double>("wz_max", 1.0);

    mode_ = this->get_parameter("mode").as_string();
    hz_ = this->get_parameter("publish_hz").as_double();
    in_topic_ = this->get_parameter("cmd_vel_in_topic").as_string();
    out_topic_ = this->get_parameter("cmd_vel_out_topic").as_string();
    vx_max_ = static_cast<float>(this->get_parameter("vx_max").as_double());
    vy_max_ = static_cast<float>(this->get_parameter("vy_max").as_double());
    wz_max_ = static_cast<float>(this->get_parameter("wz_max").as_double());

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);

    if (mode_ == "joystick") {
      sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
          in_topic_, 10, std::bind(&CmdVelRepeater::on_cmd_vel, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Mode=joystick, repeating %s -> %s @ %.1f Hz", in_topic_.c_str(),
                  out_topic_.c_str(), hz_);
    } else if (mode_ == "keyboard") {
      RCLCPP_INFO(this->get_logger(), "Mode=keyboard, publishing %s @ %.1f Hz", out_topic_.c_str(), hz_);
      RCLCPP_INFO(this->get_logger(), "Keyboard: arrows for vx/vy, a/d for wz, Shift gives max. Space stops.");
      start_keyboard();
    } else {
      throw std::runtime_error("Unknown mode: " + mode_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(hz_, 1e-3));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&CmdVelRepeater::on_timer, this));
  }

  ~CmdVelRepeater() override { stop_keyboard(); }

 private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lg(m_);
    latest_.vx = static_cast<float>(msg->linear.x);
    latest_.vy = static_cast<float>(msg->linear.y);
    latest_.wz = static_cast<float>(msg->angular.z);
    latest_rx_sec_ = now_sec();
  }

  void on_timer() {
    if (mode_ == "keyboard") {
      const double now = now_sec();
      std::lock_guard<std::mutex> lg(m_);
      if (kbd_vx_ts_ > 0.0 && (now - kbd_vx_ts_) > kbd_timeout_s_) latest_.vx = 0.f;
      if (kbd_vy_ts_ > 0.0 && (now - kbd_vy_ts_) > kbd_timeout_s_) latest_.vy = 0.f;
      if (kbd_wz_ts_ > 0.0 && (now - kbd_wz_ts_) > kbd_timeout_s_) latest_.wz = 0.f;
    }

    Cmd cmd;
    {
      std::lock_guard<std::mutex> lg(m_);
      cmd = latest_;
    }
    geometry_msgs::msg::Twist out;
    out.linear.x = cmd.vx;
    out.linear.y = cmd.vy;
    out.angular.z = cmd.wz;
    pub_->publish(out);
  }

  void start_keyboard() {
    kbd_stop_ = false;
    kbd_thread_ = std::thread([this]() { keyboard_loop(); });
  }

  void stop_keyboard() {
    kbd_stop_ = true;
    if (kbd_thread_.joinable()) {
      kbd_thread_.join();
    }
  }

  static double now_sec() {
    const auto ns = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(ns).count();
  }

  void apply_cmd(float vx, float vy, float wz) {
    std::lock_guard<std::mutex> lg(m_);
    latest_.vx = vx;
    latest_.vy = vy;
    latest_.wz = wz;
    const double now = now_sec();
    kbd_vx_ts_ = now;
    kbd_vy_ts_ = now;
    kbd_wz_ts_ = now;
  }

  void set_vx(float vx) {
    std::lock_guard<std::mutex> lg(m_);
    latest_.vx = vx;
    kbd_vx_ts_ = now_sec();
  }

  void set_vy(float vy) {
    std::lock_guard<std::mutex> lg(m_);
    latest_.vy = vy;
    kbd_vy_ts_ = now_sec();
  }

  void set_wz(float wz) {
    std::lock_guard<std::mutex> lg(m_);
    latest_.wz = wz;
    kbd_wz_ts_ = now_sec();
  }

  void keyboard_loop() {
    StdinRaw raw;
    std::string buf;
    buf.reserve(64);

    while (rclcpp::ok() && !kbd_stop_) {
      char tmp[32];
      const ssize_t n = ::read(::fileno(stdin), tmp, sizeof(tmp));
      if (n <= 0) {
        std::this_thread::sleep_for(50ms);
        continue;
      }
      buf.append(tmp, tmp + n);

      while (!buf.empty()) {
        const unsigned char c0 = static_cast<unsigned char>(buf[0]);
        if (c0 == ' ') {
          apply_cmd(0.f, 0.f, 0.f);
          buf.erase(0, 1);
          continue;
        }
        if (c0 == 'q' || c0 == 0x03) {  // q or Ctrl+C
          rclcpp::shutdown();
          return;
        }
        if (c0 == 'a' || c0 == 'A' || c0 == 'd' || c0 == 'D') {
          const bool full = (c0 == 'A' || c0 == 'D');
          const float scale = full ? 1.0f : 0.5f;
          const float sign = (c0 == 'a' || c0 == 'A') ? 1.0f : -1.0f;
          set_wz(sign * scale * wz_max_);
          buf.erase(0, 1);
          continue;
        }

        if (c0 != 0x1b) {  // ESC
          buf.erase(0, 1);
          continue;
        }

        // Need enough bytes for parsing.
        if (buf.size() < 3) break;

        auto starts = [&](const char* s) -> bool { return buf.rfind(s, 0) == 0; };

        // Standard arrows: ESC [ A/B/C/D
        if (starts("\x1b[A")) {
          set_vx(0.5f * vx_max_);
          buf.erase(0, 3);
          continue;
        }
        if (starts("\x1b[B")) {
          set_vx(-0.5f * vx_max_);
          buf.erase(0, 3);
          continue;
        }
        if (starts("\x1b[D")) {
          set_vy(0.5f * vy_max_);  // left -> +vy
          buf.erase(0, 3);
          continue;
        }
        if (starts("\x1b[C")) {
          set_vy(-0.5f * vy_max_);  // right -> -vy
          buf.erase(0, 3);
          continue;
        }

        // Common shifted arrows: ESC [ 1 ; 2 A/B/C/D
        if (starts("\x1b[1;2A")) {
          set_vx(1.0f * vx_max_);
          buf.erase(0, std::strlen("\x1b[1;2A"));
          continue;
        }
        if (starts("\x1b[1;2B")) {
          set_vx(-1.0f * vx_max_);
          buf.erase(0, std::strlen("\x1b[1;2B"));
          continue;
        }
        if (starts("\x1b[1;2D")) {
          set_vy(1.0f * vy_max_);
          buf.erase(0, std::strlen("\x1b[1;2D"));
          continue;
        }
        if (starts("\x1b[1;2C")) {
          set_vy(-1.0f * vy_max_);
          buf.erase(0, std::strlen("\x1b[1;2C"));
          continue;
        }

        // Unrecognized escape; drop one byte and retry.
        buf.erase(0, 1);
      }
    }
  }

  std::string mode_;
  double hz_{50.0};
  std::string in_topic_;
  std::string out_topic_;
  float vx_max_{0.5f};
  float vy_max_{0.25f};
  float wz_max_{1.0f};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex m_;
  Cmd latest_{};
  double latest_rx_sec_{0.0};

  std::thread kbd_thread_;
  bool kbd_stop_{false};
  double kbd_vx_ts_{0.0};
  double kbd_vy_ts_{0.0};
  double kbd_wz_ts_{0.0};
  double kbd_timeout_s_{0.35};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CmdVelRepeater>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cmd_vel_repeater exception: %s\n", e.what());
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}

