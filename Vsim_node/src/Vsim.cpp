#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"

class VsimNode : public rclcpp::Node
{
public:
	static constexpr double kEarthRadiusMeters = 6378137.0;
	static constexpr double kDegToRad = M_PI / 180.0;
	static constexpr double kRadToDeg = 180.0 / M_PI;
	static constexpr double kNoiseStdDevMeters = 3.3;

	VsimNode()
	: Node("vsim"),
	  random_engine_(std::random_device{}()),
	  noise_distribution_(0.0, kNoiseStdDevMeters),
	  keep_message_distribution_(0.3),
	  received_count_(0),
	  published_count_(0),
	  dropped_count_(0)
	{
	  publisher_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>(
	    "/novatel/oem7/vsim",
	    rclcpp::SensorDataQoS());

	  subscription_ = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
	    "/novatel/oem7/bestpos",
	    rclcpp::SensorDataQoS(),
	    std::bind(&VsimNode::bestpos_callback, this, std::placeholders::_1));

	  RCLCPP_INFO(this->get_logger(), "VSim node started.");
	  RCLCPP_INFO(this->get_logger(), "Subscribe: /novatel/oem7/bestpos");
	  RCLCPP_INFO(this->get_logger(), "Publish: /novatel/oem7/vsim");
	  RCLCPP_INFO(this->get_logger(), "Gaussian noise: mean=0.0 m, stddev=3.3 m");
	  RCLCPP_INFO(this->get_logger(), "Drop rate: 70%% (keep 30%%)");
	}

private:
	void bestpos_callback(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr msg)
	{
	  ++received_count_;
	  if (received_count_ == 1) {
	    RCLCPP_INFO(this->get_logger(), "First BESTPOS message received.");
	  }

	  if (!keep_message_distribution_(random_engine_)) {
	    ++dropped_count_;
	    if (received_count_ % 100 == 0) {
	      RCLCPP_INFO(
	        this->get_logger(),
	        "Running status: recv=%zu, pub=%zu, drop=%zu",
	        received_count_, published_count_, dropped_count_);
	    }
	    return;
	  }

	  auto output = *msg;

	  // Sample ENU noise in meters, then convert to geodetic increments.
	  const double north_noise_m = noise_distribution_(random_engine_);
	  const double east_noise_m = noise_distribution_(random_engine_);
	  const double up_noise_m = noise_distribution_(random_engine_);

	  const bool looks_like_degrees =
	    std::abs(output.lat) > M_PI || std::abs(output.lon) > 2.0 * M_PI;

	  const double lat_rad = looks_like_degrees ? (output.lat * kDegToRad) : output.lat;
	  const double cos_lat = std::cos(lat_rad);
	  const double safe_cos_lat = std::max(std::abs(cos_lat), 1e-6);

	  const double dlat_rad = north_noise_m / kEarthRadiusMeters;
	  const double dlon_rad = east_noise_m / (kEarthRadiusMeters * safe_cos_lat);

	  const double dlat = looks_like_degrees ? (dlat_rad * kRadToDeg) : dlat_rad;
	  const double dlon = looks_like_degrees ? (dlon_rad * kRadToDeg) : dlon_rad;

	  if (std::abs(dlat) > 0.01 || std::abs(dlon) > 0.01) {
	    RCLCPP_WARN(
	      this->get_logger(),
	      "Abnormal geodetic noise increment detected: dlat=%.6f, dlon=%.6f (%s).",
	      dlat,
	      dlon,
	      looks_like_degrees ? "deg" : "rad");
	  }

	  output.lat += dlat;
	  output.lon += dlon;
	  output.hgt += up_noise_m;

	  publisher_->publish(output);
	  ++published_count_;
	  if (received_count_ % 100 == 0) {
	    RCLCPP_INFO(
	      this->get_logger(),
	      "Running status: recv=%zu, pub=%zu, drop=%zu",
	      received_count_, published_count_, dropped_count_);
	  }
	}

	rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subscription_;
	rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr publisher_;
	std::mt19937 random_engine_;
	std::normal_distribution<double> noise_distribution_;
	std::bernoulli_distribution keep_message_distribution_;
	size_t received_count_;
	size_t published_count_;
	size_t dropped_count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VsimNode>());
	rclcpp::shutdown();
	return 0;
}
