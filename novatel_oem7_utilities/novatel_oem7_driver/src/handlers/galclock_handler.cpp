//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/galclock.hpp"

namespace novatel_oem7_driver{

    class GALCLOCKHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::GALCLOCK>> GALCLOCK_pub_;

        void publishGALCLOCK(Oem7RawMessageIf::ConstPtr& msg){
          std::shared_ptr<novatel_oem7_msgs::msg::GALCLOCK> galclock;
          MakeROSMessage(msg, galclock);
          GALCLOCK_pub_->publish(galclock);
        }

    public:
        GALCLOCKHandler() = default;
        ~GALCLOCKHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          GALCLOCK_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::GALCLOCK>>("GALCLOCK", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({GALCLOCK_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "GALCLOCK < [id= " <<  msg->getMessageId() << "]");

          publishGALCLOCK(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::GALCLOCKHandler, novatel_oem7_driver::Oem7MessageHandlerIf)


