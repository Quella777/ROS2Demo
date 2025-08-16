#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;
using namespace std::chrono_literals;

class TurtleCircleNode : public rclcpp::Node
{
public:
    TurtleCircleNode(const string &node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        // timer_ = this->create_wall_timer(1000ms, bind(&TurtleCircleNode::timer_callback, this));

        timer_ = this->create_wall_timer(1000ms, [this]()
                                         {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 1.0;  // 设置线速度
            message.angular.z = 0.5; // 设置角速度
            publisher_->publish(message); });
    }

    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1.0;  // 设置线速度
        message.angular.z = 0.5; // 设置角速度

        publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;                                // 定时器的智能指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 发布者的只能指针
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleCircleNode>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();
}