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
#include <protocol/msg/head_tof_payload.hpp>
using std::placeholders::_1;
// 3.自定义节点类
class TofSubscriberNode: public rclcpp::Node {
public:
    TofSubscriberNode()
    : Node("tof_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "订阅方创建！");
        // 3-1 创建一个订阅方
        /*
           模板：对象的消息类型
           参数：
            1.话题名称
            2.QOS:服务质量管理，对类长度
            3.回调函数。
           返回值：订阅对象的指针
        */
        subscription_ = this->create_subscription<protocol::msg::HeadTofPayload>(
            "head_tof_payload", 10, std::bind(&TofSubscriberNode::do_callback, this, _1));
    }

private:
    void do_callback(const protocol::msg::HeadTofPayload::SharedPtr msg) const {
        // 3-2 解析并输出数据
        RCLCPP_INFO(this->get_logger(), "订阅到的消息: left_head.data[6] = %f", msg->left_head.data[6]);
        RCLCPP_INFO(this->get_logger(), "订阅到的消息: msg->right_head.data[6] = %f", msg->right_head.data[6]);

    }
    rclcpp::Subscription<protocol::msg::HeadTofPayload>::SharedPtr subscription_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4.调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<TofSubscriberNode>());
    // 5.资源释放
    rclcpp::shutdown();
    return 0;
}
