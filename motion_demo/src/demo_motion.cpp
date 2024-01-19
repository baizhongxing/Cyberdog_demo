
/*
    需求：订阅发布方发布的消息，并在终端输出
    流程：
        1.包含头文件；
        2.初始化ROS2客户端
        3.自定义节点类
          3-1 创建一个订阅方
          3-2 解析并输出数据
        4.调用spin函数，并传入节点对象指针
        5.资源释放
*/

// 1.包含头文件；
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include <protocol/msg/motion_servo_cmd.hpp>
#include <protocol/msg/motion_servo_response.hpp>
#include "std_msgs/msg/float64.hpp"
#include <chrono>
using std::placeholders::_1;
// 3.自定义节点类
class MotionSubscriberNode: public rclcpp::Node {
public:
    MotionSubscriberNode()
    : Node("motion_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "订阅方创建！");
        
       // 创建速度发布者，连接到 "motion_servo_cmd" topic
       velocity_publisher_ = create_publisher<protocol::msg::MotionServoCmd>("motion_servo_cmd", 10);

        // 创建频率发布者，连接到 "velocity_frequency" topic
        frequency_publisher_ = create_publisher<std_msgs::msg::Float64>("velocity_frequency", 10);

    // 创建定时器，以指定频率发布速度消息
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MotionSubscriberNode::publishVelocity, this));
    }

private:
    void publishVelocity() {
        static auto servo_response_msg = std::make_shared<protocol::msg::MotionServoCmd>();
        servo_response_msg->motion_id = 111;
        // servo_response_msg->code = code;
        velocity_publisher_->publish(*servo_response_msg);

        //         // 发布频率消息（用于演示）
        // auto frequency_msg = std_msgs::msg::Float64();
        // frequency_msg.data = 1.0 / 1.0;  // 设置频率为1 Hz
        // frequency_publisher_->publish(frequency_msg);

        servo_response_msg->motion_id = 303;
        servo_response_msg->vel_des = std::vector<float>{0.2, 0, 0};
        servo_response_msg->step_height =  std::vector<float>{0.05, 0.05};
        // servo_response_msg->code = code;
        velocity_publisher_->publish(*servo_response_msg);

        // //         // 发布频率消息（用于演示）
        // auto frequency_msg_motion = std_msgs::msg::Float64();
        // frequency_msg_motion.data = 1.0 / 1.0;  // 设置频率为1 Hz
        // frequency_publisher_->publish(frequency_msg_motion);
 
    }

    rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr frequency_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionSubscriberNode>();

    // 4.调用spin函数，并传入节点对象指针
    rclcpp::spin(node);
    // 5.资源释放
    rclcpp::shutdown();
    return 0;
}

