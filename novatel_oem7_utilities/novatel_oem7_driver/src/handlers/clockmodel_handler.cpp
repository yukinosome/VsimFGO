//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/clockmodel.hpp"

namespace novatel_oem7_driver
{
    class CLOCKMODELHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::CLOCKMODEL>> CLOCKMODEL_pub_;

        void publishCLOCKMODEL(Oem7RawMessageIf::ConstPtr& msg)
        {
          std::shared_ptr<novatel_oem7_msgs::msg::CLOCKMODEL> clockmodel;
          MakeROSMessage(msg, clockmodel);
          CLOCKMODEL_pub_->publish(clockmodel);
        }

    public:
        CLOCKMODELHandler() = default;
        ~CLOCKMODELHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          CLOCKMODEL_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::CLOCKMODEL>>("CLOCKMODEL", node);
        }
        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({CLOCKMODEL_OEM7_MSGID});
          return MSG_IDS;
        }

        void handleMsg(Oem7RawMessageIf::ConstPtr msg) override
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "CLOCKMODEL < [id= " <<  msg->getMessageId() << "]");

          publishCLOCKMODEL(msg);
        }
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::CLOCKMODELHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
