#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "status_interfaces/msg/system_status.hpp"

using namespace std;
using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher(const string &NodeName) : Node(NodeName)
    {
        publisher_ = this->create_publisher<status_interfaces::msg::SystemStatus>("/sys_status", 10);
        timer_ = this->create_wall_timer(1000ms, bind(&StatusPublisher::GetSysStatus, this));
    }

    // 计算cpu占用率
    float get_cpu_usage()
    {
        static long prev_idle = 0, prev_total = 0;

        std::ifstream file("/proc/stat");
        std::string cpu;
        long user, nice, system, idle, iowait, irq, softirq, steal;
        file >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

        long idle_time = idle + iowait;
        long total_time = user + nice + system + idle + iowait + irq + softirq + steal;

        long diff_idle = idle_time - prev_idle;
        long diff_total = total_time - prev_total;
        prev_idle = idle_time;
        prev_total = total_time;

        if (diff_total == 0)
            return 0.0f;
        return (100.0f * (diff_total - diff_idle) / diff_total);
    }

    // 获取内存信息
    void get_memory_usage(float &percent, float &total, float &available)
    {
        std::ifstream file("/proc/meminfo");
        std::string key;
        long value;
        std::string unit;
        long mem_total_kb = 0, mem_available_kb = 0;

        while (file >> key >> value >> unit)
        {
            if (key == "MemTotal:")
                mem_total_kb = value;
            if (key == "MemAvailable:")
                mem_available_kb = value;
        }

        total = mem_total_kb / 1024.0f;
        available = mem_available_kb / 1024.0f;
        percent = ((mem_total_kb - mem_available_kb) * 100.0f / mem_total_kb);
    }

    // 获取网络收发字节数
    void get_network_usage(double &net_recv, double &net_send)
    {
        std::ifstream file("/proc/net/dev");
        std::string line;
        net_recv = 0;
        net_send = 0;

        std::getline(file, line); // 跳过第一行
        std::getline(file, line); // 跳过第二行
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string iface;
            long recv_bytes, dummy, send_bytes;
            iss >> iface;

            // 去掉 "eth0:" 里的冒号
            if (iface.back() == ':')
                iface.pop_back();

            // 接收字节数在第2列，发送字节数在第10列
            iss >> recv_bytes; // recv bytes
            for (int i = 0; i < 7; ++i)
                iss >> dummy;  // 跳过多列
            iss >> send_bytes; // send bytes

            if (iface != "lo") // 排除本地回环接口
            {
                net_recv += recv_bytes;
                net_send += send_bytes;
            }
        }
    }

    // 获取主机名
    std::string get_hostname()
    {
        char hostname[256];
        gethostname(hostname, sizeof(hostname));
        return std::string(hostname);
    }

    // 获取系统资源信息
    void GetSysStatus()
    {
        auto msg = status_interfaces::msg::SystemStatus();
        msg.stamp = this->get_clock()->now();
        msg.host_name = get_hostname();
        msg.cpu_percent = get_cpu_usage();
        get_memory_usage(msg.memory_percent, msg.memory_totla, msg.memory_available);
        get_network_usage(msg.net_recv, msg.net_send);
        // 发布
        publisher_->publish(msg);
    }

private:
    // 话题发布对象的智能指针
    rclcpp::Publisher<status_interfaces::msg::SystemStatus>::SharedPtr publisher_;
    // 定时器的智能指针
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = make_shared<StatusPublisher>("sys_status");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}