//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/galfnavephemeris.hpp"

namespace novatel_oem7_driver
{
    class GALFNAVEPHEMERISHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::GALFNAVEPHEMERIS>> GALFNAVEPHEMERIS_pub_;

        void publishGALFNAVEPHEMERIS(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::GALFNAVEPHEMERIS> galFNavEphemeris;
          MakeROSMessage(msg, galFNavEphemeris);
          GALFNAVEPHEMERIS_pub_->publish(galFNavEphemeris);
        }

    public:
        GALFNAVEPHEMERISHandler() = default;
        ~GALFNAVEPHEMERISHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          GALFNAVEPHEMERIS_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::GALFNAVEPHEMERIS>>("GALFNAVEPHEMERIS", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({GALFNAVEPHEMERIS_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "GALFNAVEPHEMERIS < [id= " <<  msg->getMessageId() << "]");

          publishGALFNAVEPHEMERIS(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::GALFNAVEPHEMERISHandler, novatel_oem7_driver::Oem7MessageHandlerIf)

