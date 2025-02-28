#include "rclcpp/rclcpp.hpp"
#include "base_msg/msg/battery.hpp"
#include  <serial/serial.h>
#include <std_msgs/msg/string.hpp>

#define RESERVED 0x00
#define STARTFLAG 0xA5
#define UPPERADD 0x40
#define BMSADD 0x01
#define ELECTID 0x90
#define STATUSID 0x93
#define NUMBER 0x94
#define TEMPID 0x96
#define ERRORID 0x98
#define DATALENGTH 0x08


using base_msg::msg::Battery;
using namespace std::chrono_literals;

uint8_t rev_electArray[13];//创建接收返回数据的数组
uint8_t rev_statusArray[13];
uint8_t rev_tempArray[13];
uint8_t rev_temp_numArray[13];
uint8_t rev_errorArray[13];

class battery_topic_pub : public rclcpp::Node
{
public:

    //打开串口
    //发送查询帧
    //接收数据
    //对数据解析发布
    //开始发布
  battery_topic_pub() : Node("battery_topic_pub")
    {
        this->declare_parameter<std::string>("battery_device","/dev/ttyUSB0");
        this->get_parameter("battery_device",battery_device_);
        // 打开串口
        try {
            Ser.setPort(battery_device_);
            Ser.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            Ser.setTimeout(to);
            Ser.open();
            if(Ser.isOpen()){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("battery_topic"), "Serial port open.");
            }
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("battery_topic"), "Unable to open port");
        }

        battery_publisher = this->create_publisher<Battery>("battery_topic", 10);
        battery_error_pub = this->create_publisher<std_msgs::msg::String>("YHS_CIR02/Error", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&battery_topic_pub::on_timer, this));
    }

    ~battery_topic_pub()
    {
        // 确保关闭串口
        if (Ser.isOpen()) {
            Ser.close();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("battery_topic"), "Serial port closed.");
        }
    }
private:
    //校验和函数
    uint8_t Check_Sum(uint8_t *data)
    {
        int i;
        uint8_t sum = 0;
        for (i = 0; i < 12; i++)
        {
            sum += data[i];
        }
        return sum & 0xFF;
    }
    //查询函数的封装
    void Query(uint8_t *data,uint8_t id){
        data[0] = STARTFLAG;
        data[1] = UPPERADD;
        data[2] = id;
        data[3] = DATALENGTH;
        for(int i = 4; i < 12; ++i)
        {
            data[i] = RESERVED;
        }
        data[12] = Check_Sum(data);
    }
    //发送函数
    void send_data(serial::Serial &Ser,uint8_t *data)
    {
        Ser.write(data,13);
    }
    //接收函数
    void recv_data(serial::Serial &Ser,uint8_t *data)
    {
        Ser.read(data,13);
    }
    //检测返回数据是否正确
    bool send_queery_and_recv(serial::Serial &Ser,uint8_t *query,uint8_t *recv,int max_retries = 3)
   {
    for(int attempt = 0; attempt <max_retries; ++attempt)
    {
        send_data(Ser,query);
        rclcpp::sleep_for(100ms);
        recv_data(Ser,recv);
        if(Check_Sum(recv) == recv[12])
        {
            return true;
        }
        else{
            RCLCPP_WARN_STREAM(rclcpp::get_logger("battery_topic"),"Check_Sum Error, retrying...");
        }
    }
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("battery_topic"),"Failed to recceive valid response after retries");
    return false;
   }
   //读取电量以及功率的函数
   void read_elect_power(serial::Serial &Ser,Battery &battery){
    //创建查询帧的数组
    uint8_t electArray[13] = {0};
    //对查询帧进行封装
    Query(electArray,ELECTID);
    //发送查询帧
    //接收返回数据
    //解析数据
    //对接收的数据进行校验，只检查校验位
    if(send_queery_and_recv(Ser,electArray,rev_electArray))
    {
        //电池的功率 采集电压（2-3bit）*总电流（4-5bit）注意电流的偏移量为3000 单位w
        battery.power = (rev_electArray[6] << 8 | rev_electArray[7]) * ((rev_electArray[8] << 8 | rev_electArray[9]) - 3000);//计算出来的功率单位为0.01w
        RCLCPP_INFO_STREAM(rclcpp::get_logger("battery_topic"), "battery.power =" << battery.power);
        ////电池的电量（6-7bit）
        battery.electricity = ((rev_electArray[10] << 8) | rev_electArray[11]) /10.0;//计算出来的电量单位为0.1%
        RCLCPP_INFO_STREAM(rclcpp::get_logger("battery_topic"), "battery.electricity =" << battery.electricity);
    }
    else{
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("battery_topic"),"Failed to read battery power and electricity");
    }
   }
    //电池充电放电的判断以及剩余容量的函数
    void read_status(serial::Serial &Ser,Battery &battery){
    //创建发送查询帧的数组
        uint8_t statusArray[13] = {0};
        //对查询帧进行封装
        Query(statusArray,STATUSID);
        //发送查询帧
        //接收返回数据
        //解析数据
        //对接收的数据进行校验，只检查校验位
        if(send_queery_and_recv(Ser,statusArray,rev_statusArray))
        {
            //电池充电放电状态(8bit)
            if(rev_statusArray[4] == 1)
            {
                battery.charging = true;
            }
            else
            {
                battery.charging = false;
            }
            //电池剩余容量（4-7bit）
            battery.capacity = (rev_statusArray[5] << 24 | rev_statusArray[6] << 16 | rev_statusArray[7] << 8 | rev_statusArray[8]) / 1000.0;//计算出来的电量单位为1mAh
            RCLCPP_INFO_STREAM(rclcpp::get_logger("battery_topic"), "battery.capacity =" << battery.capacity);
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("battery_topic"),"Failed to read battery status and capacity");
        }
    }
    //电池温度的函数取所有单体的平均温度
    void  read_temperature(serial::Serial &Ser,Battery &battery){
    //创建发送温度查询帧的数组
    uint8_t tempArray[13] = {0};
    //创建发送有多少个单体查询帧的数组
    uint8_t temp_numArray[13] = {0};
    //对查询帧进行封装
    Query(tempArray,TEMPID);
    Query(temp_numArray,NUMBER);
    if(send_queery_and_recv(Ser,temp_numArray,rev_temp_numArray))
    {
        uint8_t numCells = rev_temp_numArray[4];
        std::vector<_Float64> temp;
        int receivedCells = 0;
        //对接收的数据进行校验，只检测校验位
        if(send_queery_and_recv(Ser,tempArray,rev_tempArray))
        {
            float sum = 0.0;
            sum = (rev_tempArray[5]-40) + (rev_tempArray[6]-40); 
            battery.temperature = sum / 2.0;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("battery_topic"), "battery.temperature =" << battery.temperature);
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("battery_topic"),"Failed to read battery temperature");
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("battery_topic"),"Failed to read battery temperature");
    }
    }
    //判断电池是否异常
    std_msgs::msg::String judg_battery_error(serial::Serial &Ser){
        //创建发送电池异常查询帧的数组
        uint8_t errorArray[13] = {0};
        //对查询帧进行封装
        Query(errorArray,ERRORID);
        //发送查询帧
        //接收返回数据
        //解析数据
        //对接收的数据进行校验，只检查校验位
        if(send_queery_and_recv(Ser,errorArray,rev_errorArray))
        {
            std_msgs::msg::String error_msg;
            if(rev_errorArray[5] || rev_errorArray[6] || rev_errorArray[7] ||
             rev_errorArray[8] || rev_errorArray[9] || rev_errorArray[10] ||
              rev_errorArray[11] )
              {
                 error_msg.data = "battery error";
              }
              else
              {
                 error_msg.data = "battery ok";
              }
              return error_msg;
        }
         else
        {
        RCLCPP_ERROR(rclcpp::get_logger("battery_error"), "Check_Sum Error");
        std_msgs::msg::String msg;
        msg.data = "Check_Sum Error";
        return msg;
        }

    }
    void on_timer()
    {
        auto Battery_msg = Battery();
        //获取电池电量和功率
        read_elect_power(Ser,Battery_msg);
        //获取电池状态
        read_status(Ser,Battery_msg);
        //获取电池温度
        read_temperature(Ser,Battery_msg);
        //判断电池是否异常
        auto msg = judg_battery_error(Ser);
        battery_publisher->publish(Battery_msg);
        battery_error_pub->publish(msg);
    }

    std::string battery_device_;
    //创建一个串口
    serial::Serial Ser;
    //创建一个发布者
    rclcpp::Publisher<Battery>::SharedPtr battery_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_error_pub;

    //创建定时器
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<battery_topic_pub>());
  rclcpp::shutdown();
  return 0;
}