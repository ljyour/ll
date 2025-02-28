#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#define STARTFLAG1 0xAA
#define STARTFLAG2 0x55
#define ADDR 0x01 
#define CMD 0x03 //主机给从机发送时命令的指令
#define QUERY 0x81 //主机给从机发送查询的指令
#define HEARTLENGTH 0x1D //心跳包的数据长度
#define ZERO 0x00 //默认是零
#define DATALENGTH 0x04 //发送数据长度
#define RUN 0x02 //电机运行状态
#define UNLOCK 0x00 //电机松轴
#define BRAKE 0x06  //电机刹车
#define LOCK 0x04   //电机锁轴
#define RESERVERD 0x00 //保留位
#define END 0xCC    //包尾 
#define DATA_SIZE 13 //发送数组的大小
#define HEARTBEAT_SIZE 36 //心跳包数组的大小
#define BUFFER_SIZE 144 //接收缓冲区的数组大小

//创建一个sp对象，用于串口通信
serial::Serial sp;

//定义一个心跳包的数组
uint8_t Recv_Data[BUFFER_SIZE] = {0};
// 定义一个下发指令的数组
uint8_t Spend_data[DATA_SIZE] = {0};

double linear_v = 0.0;
double angular_w = 0.0;

//订阅cmd_vel节点
//打开串口
//控制速度
//对接收的查询帧进行分析
//读取编码器的数据转换成里程计的信息
class Motor_Drive : public rclcpp::Node
{
public:
    Motor_Drive() : Node("Motor_Drive_Controller"), wheel_radius(0.08),wheel_base(0.38)
    {
        this->declare_parameter<std::string>("device_id","/dev/ttyS4");
        this->get_parameter("device_id",device_id_);
        try
    {
        sp.setPort(device_id_); // 或者你具体的串口设备路径
        sp.setBaudrate(115200); // 设置波特率
        serial::Timeout timout = serial::Timeout::simpleTimeout(1000);
        sp.setTimeout(timout); // 设置超时时间
        sp.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
    }
     subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Motor_Drive::cmd_vel_callback, this, std::placeholders::_1));
                motor_contol_timer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&Motor_Drive::Cmd_Speed,this));
    driver_data_timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Motor_Drive::read_driver_data_callback, this));
    }

private:
    //校验函数
    uint16_t crc(uint8_t *data,uint16_t len)
    {
        uint8_t i = 0;
        uint16_t sum = 0 ;
        for(i = 0 ; i < len; i++)
        {
            sum += data[i];
        }
        return sum & 0xff;
    }
    //封装控制帧控制左右两个电机
    void CotrolMotors(double speed1, double speed2){
        if(!sp.isOpen())
        {
            RCLCPP_ERROR(this->get_logger(),"controlMotors serial is not open ");
            return ;
        }
        //单位转换由米转换为厘米
        int16_t speed1_int = static_cast<int16_t>(speed1 * 100);
        int16_t speed2_int = static_cast<int16_t>(speed2 * 100);
        //给数组每一位赋值
        Spend_data[0] = STARTFLAG1;
        Spend_data[1] = STARTFLAG2;
        Spend_data[2] = ADDR;
        Spend_data[3] = CMD;
        Spend_data[4] = DATALENGTH;
        //对速度进行限制
        if(speed1_int > -180 && speed1_int < 180 && speed2_int < 180 && speed2_int > -180)
        {
            // 第一个电机的速度
            Spend_data[5] = speed1_int >> 8;
            Spend_data[6] = speed1_int & 0xFF;
            // 第二个电机的速度
            Spend_data[7] = speed2_int >> 8;
            Spend_data[8] = speed2_int & 0xFF;
            RCLCPP_INFO(this->get_logger(), "Spend_data[5]: {} Spend_data[6]: {} Spend_data[7]: {} Spend_data[8]: {}", Spend_data[5], Spend_data[6], Spend_data[7], Spend_data[8]);
        }
        else
        {
            if(speed1_int > 0)
            {
                speed1_int = 100;
                Spend_data[5] = speed1_int >> 8;
                Spend_data[6] = speed1_int & 0xFF;
                RCLCPP_INFO(this->get_logger(), "speeding: {} Spend_data[5]: {} Spend_data[6]: {}", speed1_int, Spend_data[5], Spend_data[6]);
            }else
            {
                speed1_int = -100;
                Spend_data[5] = speed1_int >> 8;
                Spend_data[6] = speed1_int & 0xFF;
               RCLCPP_INFO(this->get_logger(), "speeding: {} Spend_data[5]: {} Spend_data[6]: {}", speed1_int, Spend_data[5], Spend_data[6]);

            }
            if(speed2_int > 0)
            {
                speed2_int = 100;
                Spend_data[7] = speed2_int >> 8;
                Spend_data[8] = speed2_int & 0xFF;
                RCLCPP_INFO(this->get_logger(), "speeding: {} Spend_data[7]: {} Spend_data[8]: {}", speed2_int, Spend_data[7], Spend_data[8]);
            }
            else
            {
                speed2_int = -100;
                Spend_data[7] = speed2_int >> 8;
                Spend_data[8] = speed2_int & 0xFF;
                RCLCPP_INFO(this->get_logger(), "speeding: {} Spend_data[7]: {} Spend_data[8]: {}", speed2_int, Spend_data[7], Spend_data[8]);
            }
        }
        Spend_data[9] = 0x02;
        Spend_data[10] = RESERVERD;
        Spend_data[11] = crc(Spend_data,DATA_SIZE-2);
        Spend_data[12] = END;
        if (sp.isOpen())
        {
            sp.write(Spend_data, DATA_SIZE);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"Serial port is not open. Unable to send data.");
        }
    }
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_v = 0;
        angular_w = 0;
        linear_v = msg->linear.x;
        angular_w = msg->angular.z;
        RCLCPP_INFO(this->get_logger(), "x: {} z: {}", linear_v, angular_w);
    }
    void Cmd_Speed()
    {
        int left_speed = ( linear_v - (angular_w * (wheel_base / 2) *5));
        int right_speed = ( linear_v + (angular_w * (wheel_base / 2) *5));
        CotrolMotors(right_speed,left_speed);
    }
    void read_driver_data_callback(){
        if(!sp.isOpen()){
            RCLCPP_INFO(this->get_logger(),"read_driver_data_callback: serial is not open");
            return ;
        }
        sp.read(Recv_Data, BUFFER_SIZE);
        //解析数据
        //找到帧头
        size_t index = 0;
        while(index < BUFFER_SIZE)
        {
            if(Recv_Data[index] == STARTFLAG1 && Recv_Data[index + 1] == STARTFLAG2)
            {
                //检查剩下的数据不足构成完整的数据包时，等待更多的数据
                    if (index +36 > BUFFER_SIZE)
                    {
                        
                        break;
                    }
                    //定义一个心跳包大小的数组接收数据
                    uint8_t heartbeat[HEARTBEAT_SIZE] = {0};
                    //将数据拷贝到心跳包数组中
                    memcpy(heartbeat, Recv_Data+index, HEARTBEAT_SIZE);
                    //检查crc校验和
                    if(crc(heartbeat, HEARTBEAT_SIZE-2) != heartbeat[HEARTBEAT_SIZE-2]){
                         RCLCPP_ERROR_STREAM(this->get_logger(), "CRC check failed");
                        index++;
                        continue;
                    }
                    if(heartbeat[HEARTBEAT_SIZE-1] != END){
                         RCLCPP_ERROR_STREAM(this->get_logger(), "End flag error");
                        index++;
                        continue;
                    }
                //对心跳包的数据开始解析
                 //电机1编码器里面的数据
                int16_t encoder1;
                encoder1 = (heartbeat[5] << 8) | heartbeat[6];
                //电机2编码器里面的数据
                int16_t encoder2;
                encoder2 = (heartbeat[7] << 8) | heartbeat[8];
                //电机1里面的速度数据 单位 mm/s
                int16_t motor_speed1;
                motor_speed1 = (heartbeat[9] << 8) | heartbeat[10];
                //电机2里面的速度数据
                int16_t motor_speed2;
                motor_speed2 = (heartbeat[11] << 8) | heartbeat[12];
                //电机1里面的电流数据 单位 0.1A
                int16_t motor_elect1;
                motor_elect1 = (heartbeat[13] << 8) | heartbeat[14];
                //电机2里面的电流数据
                int16_t motor_elect2;
                motor_elect2 = (heartbeat[15] << 8) | heartbeat[16];
                //电机1里面的温度数据 单位摄氏度
                int16_t motor_tem1;
                motor_tem1 = (heartbeat[17] << 8) | heartbeat[18];
                //电机2里面的温度数据
                int16_t motor_tem2;
                motor_tem2 = (heartbeat[19] << 8) | heartbeat[20];
                //cpu温度 单位摄氏度
                int16_t cpu_tem;
                cpu_tem = (heartbeat[21] << 8) | heartbeat[22];
                //电机时间戳
                uint16_t motor_time;
                motor_time = (heartbeat[23] << 8) | heartbeat[24] ;
                //电机电压 单位 0.1v
                uint16_t motor_voltage;
                motor_voltage = (heartbeat[25] << 8) | heartbeat[26] ;
                //电机1的霍尔数据
                uint8_t motor_hall1;
                motor_hall1 = heartbeat[27];
                //电机2的霍尔数据
                uint8_t motor_hall2;
                motor_hall2 = heartbeat[28];
                //开关数据
                uint8_t on_off;
                on_off = heartbeat[29];
                //电机1的状态
                uint8_t motor_state1;
                motor_state1 = heartbeat[30];
                //电机2的状态
                uint8_t motor_state2;
                motor_state2 = heartbeat[31];
                //电机错误类型
                uint8_t motor_error;
                motor_error = heartbeat[32];
                //电机版本号
                uint8_t motor_version;
                motor_version = heartbeat[33];
                //目前先将接收到的数据一次性的打印出来
                     // 输出解析的结果
                    RCLCPP_INFO_STREAM(this->get_logger(), "Motor1 Encoder: " << encoder1 << ", Motor2 Encoder: " << encoder2);
                    RCLCPP_INFO_STREAM(this->get_logger(), "Motor1 Speed: " << motor_speed1 << ", Motor2 Speed: " << motor_speed2);
                    RCLCPP_INFO_STREAM(this->get_logger(), "Motor1 Current: " << motor_elect1 << ", Motor2 Current: " << motor_elect2);
                    RCLCPP_INFO_STREAM(this->get_logger(), "Motor1 Temperature: " << motor_tem1 << ", Motor2 Temperature: " << motor_tem2);
                    RCLCPP_INFO_STREAM(this->get_logger(), "CPU Temperature: " << cpu_tem << ", Motor Timestamp: " << motor_time);
                    RCLCPP_INFO_STREAM(this->get_logger(), "Motor Voltage: " << motor_voltage << ", Hall1: " << static_cast<int>(motor_hall1) << ", Hall2: " << static_cast<int>(motor_hall2));
                    RCLCPP_INFO_STREAM(this->get_logger(), "Switch: " << static_cast<int>(on_off) << ", Motor1 State: " << static_cast<int>(motor_state1) << ", Motor2 State: " << static_cast<int>(motor_state2));
                    RCLCPP_INFO_STREAM(this->get_logger(), "Motor Error: " << static_cast<int>(motor_error) << ", Motor Version: " << static_cast<int>(motor_version));
                 index += 36;
                }
                else
                {
                index ++;  
            }
        }
    }

    std::string device_id_;
    rclcpp::TimerBase::SharedPtr motor_contol_timer;
    rclcpp::TimerBase::SharedPtr driver_data_timer;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    double wheel_radius;
    double wheel_base;
};



int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Motor_Drive>());
    rclcpp::shutdown();
    return 0;
}