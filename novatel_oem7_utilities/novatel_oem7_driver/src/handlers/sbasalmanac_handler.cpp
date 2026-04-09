//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/sbasalmanac.hpp"

namespace novatel_oem7_driver
{
    class SBASALMANACHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::SBASALMANAC>> SBASALMANAC_pub_;

        void publishSBASALMANAC(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::SBASALMANAC> sbasalmanac;
          MakeROSMessage(msg, sbasalmanac);
          SBASALMANAC_pub_->publish(sbasalmanac);
        }

    public:
        SBASALMANACHandler() = default;
        ~SBASALMANACHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          SBASALMANAC_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::SBASALMANAC>>("SBASALMANAC", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({SBASALMANAC_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "SBASALMANAC < [id= " <<  msg->getMessageId() << "]");

          publishSBASALMANAC(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::SBASALMANACHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
