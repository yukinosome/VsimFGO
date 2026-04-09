//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/dualantennaheading.hpp"

namespace novatel_oem7_driver{

    class DUALANTENNAHEADINGHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::DUALANTENNAHEADING>> DUALANTENNAHEADING_pub_;

        void publishDUALANTENNAHEADING(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::DUALANTENNAHEADING> dualantennaheading;
          MakeROSMessage(msg, dualantennaheading);
          DUALANTENNAHEADING_pub_->publish(dualantennaheading);
        }

    public:
        DUALANTENNAHEADINGHandler() = default;
        ~DUALANTENNAHEADINGHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          DUALANTENNAHEADING_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::DUALANTENNAHEADING>>("DUALANTENNAHEADING", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({DUALANTENNAHEADING_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "DUALANTENNAHEADING < [id= " <<  msg->getMessageId() << "]");

          publishDUALANTENNAHEADING(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::DUALANTENNAHEADINGHandler, novatel_oem7_driver::Oem7MessageHandlerIf)


