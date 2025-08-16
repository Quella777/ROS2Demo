#include <rclcpp/rclcpp.hpp>
#include "status_interfaces/msg/system_status.hpp"
#include <QApplication>
#include <QLabel>
#include <QString>

using namespace std;
using namespace std::chrono_literals;

class StatusSub : public rclcpp::Node
{
public:
    StatusSub(const string &NodeName) : Node(NodeName)
    {
        Subscription_ = this->create_subscription<status_interfaces::msg::SystemStatus>("/sys_status", 10, bind(&StatusSub::GetStatus, this, placeholders::_1));
        label_ = new QLabel();
        label_->show();
    }

    void GetStatus(const status_interfaces::msg::SystemStatus::SharedPtr msg)
    {
        label_->setText(Get_Status(msg));
    }

    QString Get_Status(const status_interfaces::msg::SystemStatus::SharedPtr msg)
    {
        stringstream show_str;
        show_str
            << "系统时间:\t" << msg->stamp.sec << "\n"
            << "主机名称:\t" << msg->host_name << "\n"
            << "CPU占用率:\t" << msg->cpu_percent << "\n"
            << "内存使用率:\t" << msg->memory_percent << "\n"
            << "内存总大小:\t" << msg->memory_totla << "\n"
            << "剩余内存大小:\t" << msg->memory_available << "\n"
            << "网络发送数据量:\t" << msg->net_send << "\n"
            << "网络接收数据量:\t" << msg->net_recv << "\n";

        return QString::fromStdString(show_str.str());
    }

private:
    // 订阅对象的智能指针
    rclcpp::Subscription<status_interfaces::msg::SystemStatus>::SharedPtr Subscription_;
    QLabel *label_;
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);    
    rclcpp::init(argc, argv);
    auto node = make_shared<StatusSub>("SubNode");
    thread spin_thread([&]()
                       { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}