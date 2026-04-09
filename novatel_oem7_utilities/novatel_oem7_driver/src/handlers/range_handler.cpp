//
// Created by haoming on 18.04.21.
//

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oem7_ros_publisher.hpp>
#include "novatel_oem7_driver/oem7_ros_messages.hpp"
#include "novatel_oem7_msgs/msg/range.hpp"

namespace novatel_oem7_driver
{
    class RANGEHandler : public Oem7MessageHandlerIf
    {
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::RANGE>> RANGE_pub_;
        std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::RANGE>> RANGE_aux_pub_;
        bool             pub_range_aux_separately_;

        void publishRANGE(Oem7RawMessageIf::ConstPtr msg){
          std::shared_ptr<novatel_oem7_msgs::msg::RANGE> range;
          MakeROSMessage(msg, range);
          if (pub_range_aux_separately_ && range->nov_header.message_name == "RANGE_AUX")
            RANGE_aux_pub_->publish(range);
          else
            RANGE_pub_->publish(range);
        }

    public:
        explicit RANGEHandler(bool pub_range_aux_separately=true) : pub_range_aux_separately_(pub_range_aux_separately){};
        ~RANGEHandler() override = default;

        void initialize(rclcpp::Node& node) override
        {
          pub_range_aux_separately_ = node.declare_parameter<bool>("pub_range_auxiliary_separately", true);
          pub_range_aux_separately_ = node.get_parameter("pub_range_auxiliary_separately").as_bool();

          if(pub_range_aux_separately_)
          {
            RANGE_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::RANGE>>("RANGE", node);
            RANGE_aux_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::RANGE>>("RANGE_AUX", node);
          }
          else
            RANGE_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::RANGE>>("RANGE", node);
        }

        const std::vector<int>& getMessageIds() override
        {
          static const std::vector<int> MSG_IDS({RANGE_OEM7_MSGID});
          return MSG_IDS;
        }
        void handleMsg(Oem7RawMessageIf::ConstPtr msg)
        {
          if(pub_range_aux_separately_)
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "RANGE < [id= " <<  msg->getMessageId() << "]");
          else
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug_logger"), "RANGE_AUX < [id= " <<  msg->getMessageId() << "]");
          publishRANGE(msg);
        }
    };
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::RANGEHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
