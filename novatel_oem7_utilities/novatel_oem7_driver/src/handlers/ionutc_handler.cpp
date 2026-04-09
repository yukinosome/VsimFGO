//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/ionutc.hpp"

namespace novatel_oem7_driver
{
    class IONUTCHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::IONUTC>> IONUTC_pub_;

        void publishIONUTC(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::IONUTC> ionutc;
          MakeROSMessage(msg, ionutc);
          IONUTC_pub_->publish(ionutc);
        }

    public:
        IONUTCHandler() = default;
        ~IONUTCHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          IONUTC_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::IONUTC>>("IONUTC", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({IONUTC_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "IONUTC < [id= " <<  msg->getMessageId() << "]");

          publishIONUTC(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::IONUTCHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
