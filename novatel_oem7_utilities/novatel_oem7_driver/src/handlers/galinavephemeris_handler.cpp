//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/galinavephemeris.hpp"

namespace novatel_oem7_driver
{
    class GALINAVEPHEMERISHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::GALINAVEPHEMERIS>> GALINAVEPHEMERIS_pub_;

        void publishGALINAVEPHEMERIS(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::GALINAVEPHEMERIS> galINavEphemeris;
          MakeROSMessage(msg, galINavEphemeris);
          GALINAVEPHEMERIS_pub_->publish(galINavEphemeris);
        }

    public:
        GALINAVEPHEMERISHandler() = default;
        ~GALINAVEPHEMERISHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          GALINAVEPHEMERIS_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::GALINAVEPHEMERIS>>("GALINAVEPHEMERIS", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({GALINAVEPHEMERIS_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "GALINAVEPHEMERIS < [id= " <<  msg->getMessageId() << "]");

          publishGALINAVEPHEMERIS(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::GALINAVEPHEMERISHandler, novatel_oem7_driver::Oem7MessageHandlerIf)

