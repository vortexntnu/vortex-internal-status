#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <array>

class PwmCsvLogger : public rclcpp::Node
{
public:
  PwmCsvLogger()
  : Node("pwm_csv_logger")
  {
    output_file_ = this->declare_parameter<std::string>(
      "output_file", "pwm_log.csv");

    msg_id_hex_ = this->declare_parameter<std::string>(
      "msg_id_hex", "0x20");

    msg_name_ = this->declare_parameter<std::string>(
      "msg_name", "PWM_OUTPUTS");

    csv_.open(output_file_, std::ios::out | std::ios::app);
    if (!csv_.is_open()) {
      throw std::runtime_error("Failed to open output file: " + output_file_);
    }

    if (csv_.tellp() == 0) {
      csv_ << "timestamp,msg_id_hex,msg_name,status,channel,code_hex,"
              "i0,i1,i2,i3,i4,i5,i6,i7,raw_payload\n";
      csv_.flush();
    }

    sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/nautilus/pwm",
      10,
      std::bind(&PwmCsvLogger::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Logging /nautilus/pwm to %s", output_file_.c_str());
  }

  ~PwmCsvLogger()
  {
    if (csv_.is_open()) {
      csv_.close();
    }
  }

private:
  std::string format_timestamp() const
  {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t tt = system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
  }

  std::string build_raw_payload(const std::array<int, 8> & pwm) const
  {
    std::ostringstream oss;
    for (size_t i = 0; i < pwm.size(); ++i) {
      if (i > 0) {
        oss << " ";
      }
      oss << pwm[i];
    }
    return oss.str();
  }

  void callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    std::array<int, 8> pwm{0, 0, 0, 0, 0, 0, 0, 0};

    const size_t n = std::min<size_t>(8, msg->data.size());
    for (size_t i = 0; i < n; ++i) {
      pwm[i] = static_cast<int>(msg->data[i]);
    }

    if (msg->data.size() < 8) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Received PWM array with %zu elements, expected 8. Missing values padded with 0.",
        msg->data.size());
    }

    const std::string timestamp = format_timestamp();
    const std::string raw_payload = build_raw_payload(pwm);

    csv_ << "\"" << timestamp << "\","
         << "\"" << msg_id_hex_ << "\","
         << "\"" << msg_name_ << "\","
         << "\"OK\","
         << ",,"
         << pwm[0] << ","
         << pwm[1] << ","
         << pwm[2] << ","
         << pwm[3] << ","
         << pwm[4] << ","
         << pwm[5] << ","
         << pwm[6] << ","
         << pwm[7] << ","
         << "\"" << raw_payload << "\"\n";

    csv_.flush();
  }

  std::string output_file_;
  std::string msg_id_hex_;
  std::string msg_name_;

  std::ofstream csv_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PwmCsvLogger>());
  rclcpp::shutdown();
  return 0;
}
