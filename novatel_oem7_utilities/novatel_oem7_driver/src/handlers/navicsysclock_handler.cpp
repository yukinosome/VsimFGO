////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////


#include <novatel_oem7_driver/oem7_message_handler_if.hpp>

#include <novatel_oem7_driver/oem7_ros_messages.hpp>
#include <novatel_oem7_msgs/msg/navicsysclock.hpp>

#include <oem7_ros_publisher.hpp>


namespace novatel_oem7_driver
{

  /***
   * Handler of NAVICSYSCLOCK-related messages
   */
  class NAVICSYSCLOCKHandler: public Oem7MessageHandlerIf
  {
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::NAVICSYSCLOCK>> NAVICSYSCLOCK_pub_; ///< Publisher for NMEA sentences;

    void publishNAVICSYSCLOCK(
        Oem7RawMessageIf::ConstPtr msg)
    {
      std::shared_ptr<novatel_oem7_msgs::msg::NAVICSYSCLOCK> navicsysclock_msg;
      MakeROSMessage(msg, navicsysclock_msg);
      NAVICSYSCLOCK_pub_->publish(navicsysclock_msg);
    }

  public:
      NAVICSYSCLOCKHandler()
    {
    }

    ~NAVICSYSCLOCKHandler()
    {
    }

    void initialize(rclcpp::Node& node)
    {
      NAVICSYSCLOCK_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::NAVICSYSCLOCK>>("NAVICSYSCLOCK", node);
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS({NAVICSYSCLOCK_OEM7_MSGID});
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      publishNAVICSYSCLOCK(msg);
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::NAVICSYSCLOCKHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
