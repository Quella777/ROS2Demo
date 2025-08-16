#include <rclcpp/rclcpp.hpp> // 引入ROS2 C++客户端库头文件

// 定义一个继承自rclcpp::Node的PersonNode类
class PersonNode : public rclcpp::Node
{
    std::string m_name; // 存储人物姓名
    int m_age;          // 存储人物年龄

public:
    // 构造函数，初始化节点名、姓名和年龄
    PersonNode(const std::string &node_name, const std::string &name, int age)
        : Node(node_name), //调用父类构造函数
        m_name(name), m_age(age) {}

    // eat方法，模拟人物吃东西并输出日志
    void eat(const std::string &food_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s 今年 %d 岁，正在吃 %s", m_name.c_str(), m_age, food_name.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS2运行环境

    // 创建一个PersonNode实例，节点名为"cpp_node"
    auto node = std::make_shared<PersonNode>("person_node", "小明", 18);

    node->eat("苹果"); // 调用eat方法，输出吃苹果的日志

    rclcpp::spin(node); // 进入事件循环，保持节点运行

    rclcpp::shutdown(); // 关闭ROS2运行环境
    return 0;           // 程序正常退出
}