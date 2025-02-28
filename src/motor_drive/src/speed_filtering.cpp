#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class PriorityCmdVelPublisher : public rclcpp::Node {
public:
    PriorityCmdVelPublisher() : Node("priority_cmd_vel_publisher") {
        // 初始化 cmd_vel 发布器
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 初始化订阅器
        joy_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "joy_cmd_vel", 10, std::bind(&PriorityCmdVelPublisher::joy_callback, this, std::placeholders::_1));
        key_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10, std::bind(&PriorityCmdVelPublisher::key_callback, this, std::placeholders::_1));
        app_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "app_cmd_vel", 10, std::bind(&PriorityCmdVelPublisher::app_callback, this, std::placeholders::_1));
        nav_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "nav_cmd_vel", 10, std::bind(&PriorityCmdVelPublisher::nav_callback, this, std::placeholders::_1));

        // 初始化定时器，用于定期检查优先级并发布消息
        timer_ = this->create_wall_timer(100ms, std::bind(&PriorityCmdVelPublisher::publish_cmd_vel, this));

        // 初始化每个话题的更新时间为一个很早的时间
        rclcpp::Time default_time(0, 0, RCL_ROS_TIME);
        joy_last_update_ = default_time;
        key_last_update_ = default_time;
        app_last_update_ = default_time;
        nav_last_update_ = default_time;
    }

private:
    //拍段消息是否有效
    bool is_valid_msg(const geometry_msgs::msg::Twist &msg){
        const double linear_threshold = 0.01;
        const double angular_threshold = 0.01;
        return std::abs(msg.linear.x) > linear_threshold || std::abs(msg.angular.z) > angular_threshold;
    }

    // 各话题回调函数，接收消息并更新时间戳
    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (is_valid_msg(*msg)){
        joy_msg_ = *msg;
        joy_last_update_ = this->get_clock()->now();
        }
    }

    void key_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (is_valid_msg(*msg)){
        key_msg_ = *msg;
        key_last_update_ = this->get_clock()->now();
        }
    }

    void app_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
       if (is_valid_msg(*msg)){
        app_msg_ = *msg;
        app_last_update_ = this->get_clock()->now();
       }
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
       if (is_valid_msg(*msg)){
        nav_msg_ = *msg;
        nav_last_update_ = this->get_clock()->now();
       }
    }

    // 定时器回调函数，根据优先级发布消息
    void publish_cmd_vel() {
        auto current_time = this->get_clock()->now();
        rclcpp::Duration timeout(0.2s); // 超时时间为 0.2 秒

        // 根据优先级检查有效消息
        if ((current_time - joy_last_update_) < timeout) {
            cmd_vel_publisher_->publish(joy_msg_);
            RCLCPP_INFO(this->get_logger(), "Published joy_cmd_vel");
        } else if ((current_time - key_last_update_) < timeout) {
            cmd_vel_publisher_->publish(key_msg_);
            RCLCPP_INFO(this->get_logger(), "Published turtle1/cmd_vel");
        } else if ((current_time - app_last_update_) < timeout) {
            cmd_vel_publisher_->publish(app_msg_);
            RCLCPP_INFO(this->get_logger(), "Published app_cmd_vel");
        } else if ((current_time - nav_last_update_) < timeout) {
            cmd_vel_publisher_->publish(nav_msg_);
            RCLCPP_INFO(this->get_logger(), "Published nav_cmd_vel");
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid cmd_vel message to publish.");
        }
    }

    // 各优先级消息及其更新时间
    geometry_msgs::msg::Twist joy_msg_, key_msg_, app_msg_, nav_msg_;
    rclcpp::Time joy_last_update_, key_last_update_, app_last_update_, nav_last_update_;

    // 发布器和订阅器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr app_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PriorityCmdVelPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
