#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Rviz2dString : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param: parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " <<param.value_to_string().c_str());
            if (param.get_name() == "string_topic")
            {
                string_topic = param.as_string();
                overlay_text_topic = string_topic + "_overlay_text";
            }
            if (param.get_name() == "fg_color")
            {
                fg_color = param.as_string();
            }            
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "subscribed: " << string_topic << " publishing: " << overlay_text_topic << " fg color: " << fg_color << " ");
        return result;
    }

public:
    Rviz2dString() : Node("rviz2d_from_string_node")
    {      
        this->declare_parameter<std::string>("string_topic", "");
        this->declare_parameter<std::string>("fg_color", "");
        this->get_parameter("string_topic", string_topic);
        this->get_parameter("fg_color", fg_color);
        // if (string_topic == "")
        // {
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter string_topic not set");
        //     string_topic = "/chatter";   
        // }
        overlay_text_topic = diag_topic_name_ + "_overlay_text";

        RCLCPP_WARN(this->get_logger(),
            "\n\t       string_topic: %s" \
            "\n\t    diag_topic_name: %s" \
            "\n\t overlay_text_topic: %s",
            string_topic.data(),
            diag_topic_name_.data(),
            overlay_text_topic.data());

        ov_msg_.fg_color.a = 1.0;
        // https://github.com/jkk-research/colors
        if (fg_color == "r") // red
        {
            ov_msg_.fg_color.r = 0.96f;
            ov_msg_.fg_color.g = 0.22f;
            ov_msg_.fg_color.b = 0.06f;
        }
        else if (fg_color == "g") // green
        {
            ov_msg_.fg_color.r = 0.30f;
            ov_msg_.fg_color.g = 0.69f;
            ov_msg_.fg_color.b = 0.31f;
        }
        else if (fg_color == "b") // blue
        {
            ov_msg_.fg_color.r = 0.02f;
            ov_msg_.fg_color.g = 0.50f;
            ov_msg_.fg_color.b = 0.70f;
        }
        else if (fg_color == "k") // black
        {
            ov_msg_.fg_color.r = 0.19f;
            ov_msg_.fg_color.g = 0.19f;
            ov_msg_.fg_color.b = 0.23f;
        }
        else if (fg_color == "w") // white
        {
            ov_msg_.fg_color.r = 0.89f;
            ov_msg_.fg_color.g = 0.89f;
            ov_msg_.fg_color.b = 0.93f;
        }    
        else if (fg_color == "p") // pink
        {
            ov_msg_.fg_color.r = 0.91f;
            ov_msg_.fg_color.g = 0.12f;
            ov_msg_.fg_color.b = 0.39f;
        }                     
        else
        { // yellow
            ov_msg_.fg_color.r = 0.94f;
            ov_msg_.fg_color.g = 0.83f;
            ov_msg_.fg_color.b = 0.07f;
        }        
        ov_msg_.width = 200;
        ov_msg_.height = 40;

        if (!diag_topic_name_.empty()) {
          diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            diag_topic_name_, 2, std::bind(&Rviz2dString::diagCallback, this, _1));
        }
        if (!string_topic.empty()) {
          sub_st_ = this->create_subscription<std_msgs::msg::String>(
            string_topic, 10, std::bind(&Rviz2dString::strCallback, this, _1));
        }
        pub_ov_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(overlay_text_topic, 1);
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Rviz2dString::parametersCallback, this, std::placeholders::_1));
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //   "Node started: " << this->get_name() << " subscribed: " << string_topic << " publishing: " << overlay_text_topic << " ");
    }

private:
    // Callback for string
    void strCallback(const std_msgs::msg::String &st_msg);

    rviz_2d_overlay_msgs::msg::OverlayText ov_msg_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_st_;
    std::string string_topic = "", overlay_text_topic, fg_color = "r";
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr pub_ov_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    
    std::string diag_topic_name_ = "tros_diagnostics";
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_ = nullptr;
    void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

};

void Rviz2dString::strCallback(const std_msgs::msg::String &st_msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << st_msg.data);
    rviz_2d_overlay_msgs::msg::OverlayText ov_msg = ov_msg_;
    ov_msg.text = st_msg.data;
    pub_ov_->publish(ov_msg);
}

void Rviz2dString::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
  RCLCPP_WARN_ONCE(this->get_logger(),
    "Recved diagnostics from topic %s, this msg appears only once",
    diag_topic_name_.c_str());
  
  rviz_2d_overlay_msgs::msg::OverlayText ov_msg = ov_msg_;
  std::string text = "";
  // text += std::to_string(msg->header.stamp.sec) + "." + std::to_string(msg->header.stamp.nanosec);
  text += std::to_string(msg->header.stamp.sec) + ".";
  std::string str_ns = std::to_string(msg->header.stamp.nanosec);
  for (int i = 0; i < 9 - str_ns.size(); i++) {
    text += "0";
  }
  text += str_ns;

  for (const auto& status : msg->status) {
    text += " " + status.name + " " + status.message;
    for (const auto& key_value : status.values) {
      text += " " + key_value.key + ": [" + key_value.value + "]";
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Publishing: %s", text.c_str());
  ov_msg.text = text;
  pub_ov_->publish(ov_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rviz2dString>());
    rclcpp::shutdown();
    return 0;
}