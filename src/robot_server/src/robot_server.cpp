#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
class RobotServer : public rclcpp::Node {
public:
    RobotServer() : Node("robot_server") {
        // 创建 ROS2 发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("app_cmd_vel", 10);

        // 创建 ROS2 订阅器，订阅 amcl_pose 话题
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&RobotServer::poseCallback, this, std::placeholders::_1));
        // 创建 socket
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
            std::cerr << "Failed to create socket." << std::endl;
            exit(EXIT_FAILURE);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;  // 监听所有网络接口
        server_addr.sin_port = htons(8888);  // 绑定端口

        // 绑定 socket 到端口
        if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to bind to port." << std::endl;
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        // 开始监听连接请求
        if (listen(server_fd, 5) < 0) {
            std::cerr << "Failed to listen on port." << std::endl;
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        std::cout << "Server listening on port 8888..." << std::endl;
    }

void run() {
    while (rclcpp::ok()) {
        // 接受客户端的连接
        client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
        if (client_fd < 0) {
            std::cerr << "Failed to accept connection." << std::endl;
            continue;
        }

        std::cout << "Client connected." << std::endl;

        // 循环处理客户端的命令
        while (rclcpp::ok()) {
            // 处理接收到的数据
            char buffer[1024];
            int bytes_received = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            if (bytes_received < 0) {
                std::cerr << "Failed to receive data." << std::endl;
                break; // 如果接收数据失败，退出当前循环
            }

            if (bytes_received == 0) {
                std::cout << "Client disconnected." << std::endl;
                break; // 客户端关闭了连接，退出循环
            }

            buffer[bytes_received] = '\0';  // Null terminate the received data
            std::string command(buffer);
            std::cout << "Received command: " << command << std::endl;

            // 根据接收到的指令控制机器人
            handleCommand(command);

            // 发送响应回客户端
            std::string response = "Command received: " + command;
            send(client_fd, response.c_str(), response.length(), 0);
        }

        close(client_fd);  // 关闭与当前客户端的连接
    }
}


    ~RobotServer() {
        close(server_fd);  // 关闭服务器 socket
    }

private:
    void handleCommand(const std::string& command) {
        // 创建 Twist 消息
        auto msg = geometry_msgs::msg::Twist();

        if (command == "move_forward") {
            std::cout << "Robot moving forward." << std::endl;
            msg.linear.x = 1.0;  // 前进
            msg.angular.z = 0.0;  // 旋转角速度为 0
        } else if (command == "turn_left") {
            std::cout << "Robot turning left." << std::endl;
            msg.linear.x = 0.0;  // 停止前进
            msg.angular.z = 2.0;  // 左转
        } else if (command == "move_backward") {
            std::cout << "Robot moving backward." << std::endl;
            msg.linear.x = -1.0;  // 后退
            msg.angular.z = 0.0;  // 旋转角速度为 0
        } else if (command == "turn_right") {
            std::cout << "Robot turning right." << std::endl;
            msg.linear.x = 0.0;  // 停止前进
            msg.angular.z = -2.0;  // 右转
        } else if (command == "stop") {
            std::cout << "Robot stopping." << std::endl;
            msg.linear.x = 0.0;  // 停止前进
            msg.angular.z = 0.0;  // 停止旋转
        } else {
            std::cout << "Unknown command." << std::endl;
        }

        // 发布 Twist 消息到 cmd_vel
        if (msg.linear.x != 0.0 || msg.angular.z != 0.0) {
            cmd_vel_pub_->publish(msg);
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // 从 amcl_pose 消息中获取机器人的位置
        float robot_x = msg->pose.pose.position.x;
        float robot_y = msg->pose.pose.position.y;

        // 将机器人的位置发送给客户端
        if (client_fd >= 0) {
            std::string position_data = std::to_string(robot_x) + "," + std::to_string(robot_y);
            send(client_fd, position_data.c_str(), position_data.length(), 0);
        }
    }
private:
    int server_fd, client_fd;
    sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // 初始化 ROS2 节点

    RobotServer robotServer;
    robotServer.run();

    rclcpp::shutdown();  // 关闭 ROS2 节点
    return 0;
}
