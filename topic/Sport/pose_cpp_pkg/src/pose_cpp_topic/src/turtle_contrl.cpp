#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;

class TurtleContrller : public rclcpp::Node
{
public:
    TurtleContrller(const string &NodeName) : Node(NodeName)
    {
        // 填话题而不是消息接口
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_Subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, [this](const turtlesim::msg::Pose::SharedPtr pose)
                                                                             {
                                                                                 // 获取当前的位置                                                                                 
                                                                                 auto msg = geometry_msgs::msg::Twist();
                                                                                 auto current_x = pose->x;
                                                                                 auto current_y = pose->y;
                                                                                 // 打印输出
                                                                                 RCLCPP_INFO(get_logger(), "当前位置 x:%f,y:%f", current_x, current_y);

                                                                                 // 计算当前与目标位置之间的距离差和角度差

                                                                                 auto distance = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

                                                                                 // 计算:目标点相对于当前点的角度差
                                                                                 auto angle = atan2(target_y - current_y, target_x - current_x) - pose->theta;

                                                                                 //
                                                                                 if (distance > 0.1)
                                                                                 {
                                                                                     if (fabs(angle) > 0.2)
                                                                                     {
                                                                                         msg.angular.z = fabs(angle);
                                                                                     }
                                                                                     else
                                                                                     {
                                                                                         msg.linear.x = k_ * distance;
                                                                                     }
                                                                                 }

                                                                                 // 限制线速度最大值
                                                                                 if (msg.linear.x > max_speed)
                                                                                     msg.linear.x = max_speed;

                                                                                 velocity_publisher_->publish(msg); });
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_Subscription_;    // 订阅者的智能指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_; // 发布者的智能指针

private:
    double target_x = 1.0;
    double target_y = 1.0;

    double k_ = 1.0;        // 比例系数
    double max_speed = 3.0; // 最大速度
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleContrller>("TurtleConller");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
