//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/gpsephem.hpp"

namespace novatel_oem7_driver
{
    class GPSEPHEMHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::GPSEPHEM>> GPSEPHEM_pub_;

        void publishGPSEPHEM(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::GPSEPHEM> gpsephem;
          MakeROSMessage(msg, gpsephem);
          GPSEPHEM_pub_->publish(gpsephem);
        }

    public:
        GPSEPHEMHandler() = default;
        ~GPSEPHEMHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          GPSEPHEM_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::GPSEPHEM>>("GPSEPHEM", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({GPSEPHEM_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "GPSEPHEM < [id= " <<  msg->getMessageId() << "]");

          publishGPSEPHEM(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::GPSEPHEMHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
