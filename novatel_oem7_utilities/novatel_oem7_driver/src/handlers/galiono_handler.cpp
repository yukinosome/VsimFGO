//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/galiono.hpp"

namespace novatel_oem7_driver
{
    class GALIONOHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::GALIONO>> GALIONO_pub_;

        void publishGALIONO(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::GALIONO> galiono;
          MakeROSMessage(msg, galiono);
          GALIONO_pub_->publish(galiono);
        }

    public:
        GALIONOHandler() = default;
        ~GALIONOHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          GALIONO_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::GALIONO>>("GALIONO", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({GALIONO_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "GALIONO < [id= " <<  msg->getMessageId() << "]");

          publishGALIONO(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::GALIONOHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
