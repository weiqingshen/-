#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include <serial/serial.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <queue> // 新增

class TrajectorySubscriber : public rclcpp::Node
{
public:
    TrajectorySubscriber() : Node("trajectory_subscriber")
    {
        // 串口设置
        std::string port = "/dev/ttyUSB0";
        uint32_t baud_rate = 921600;
        serial_.setPort(port);
        serial_.setBaudrate(baud_rate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);

        try {
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口");
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "串口已初始化");
        } else {
            RCLCPP_ERROR(this->get_logger(), "串口未能初始化");
        }

        // 订阅 /display_planned_path 话题
        subscription_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10,
            std::bind(&TrajectorySubscriber::topic_callback, this, std::placeholders::_1));

        // 创建定时器，每 1 毫秒触发一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectorySubscriber::send_next_data, this));
    }

    ~TrajectorySubscriber() {
        serial_.close();
    }

private:
    void topic_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        // 清空队列
        data_queue_ = std::queue<std::string>();

        // 遍历所有轨迹
        for (const auto& robot_trajectory : msg->trajectory) {
            const auto& joint_trajectory = robot_trajectory.joint_trajectory;

            // 遍历每个轨迹点
            for (const auto& point : joint_trajectory.points) {

                // 创建一个字符串流来存储最终的字符串
                std::ostringstream oss;

                // 逐个将浮点数转换为字符串并连接
                for (size_t i = 0; i < point.positions.size(); ++i) {
                    float position = static_cast<float>(point.positions[i]);

                    // 根据索引生成字母标识
                    char label = 'A' + static_cast<char>(i % 6);

                    // 将数据转换为字符串并追加到字符串流中
                    oss << label << std::fixed << std::setprecision(6) << position;

                    // 每6个数据生成一次字符串并换行
                    if ((i + 1) % 6 == 0 || i == point.positions.size() - 1) {
                        std::string str = oss.str();
                        str += "\n";
                        data_queue_.push(str);  // 将字符串放入队列
                        oss.str("");
                        oss.clear();
                    }
                }
            }
        }
    }

    void send_next_data()
    {
        if (!serial_.isOpen() || data_queue_.empty()) return;

        // 从队列中取出下一组数据并发送
        std::string str = data_queue_.front();
        serial_.write(reinterpret_cast<const uint8_t*>(str.c_str()), str.size());
        std::cout << str << std::endl;
        data_queue_.pop(); // 发送后将其从队列中移除
    }

    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::queue<std::string> data_queue_;  // 用于存储待发送的数据
    mutable serial::Serial serial_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySubscriber>());
    rclcpp::shutdown();
    return 0;
}


