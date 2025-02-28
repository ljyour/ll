#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoyToCmdVelNode : public rclcpp::Node
{
public:
    JoyToCmdVelNode()
        :Node("joy_to_cmd_vel")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",10,std::bind(&JoyToCmdVelNode::joy_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("joy_cmd_vel",10);

    }
private: 
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        float linear_x = msg->axes[1];
        float angular_z = msg->axes[0];
        const float threshold = 0.3;
        if ( linear_x > threshold || linear_x < -threshold)
        {
            angular_z = 0.0;
        }
        if(linear_x > 0.8)
        {
            linear_x = 1.0;
        }
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = linear_x;
        cmd_vel_msg.angular.z = angular_z;
        cmd_vel_pub_->publish(cmd_vel_msg);
        RCLCPP_DEBUG(this->get_logger(), "Publishing cmd_vel: linear_x: %.2f, angular_z: %.2f", linear_x, angular_z);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<JoyToCmdVelNode>());
    rclcpp::shutdown();
    return 0;
}
