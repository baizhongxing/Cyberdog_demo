
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/camera_service.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using protocol::srv::CameraService;
using namespace std::chrono_literals;


// 定义节点类
class CameraClient: public rclcpp::Node {
public:
     CameraClient():Node("client_node_cpp") {
        
        client_ = this->create_client<CameraService>("camera_service");
        RCLCPP_INFO(this->get_logger(), "客户端创建成功");
        
        // // 等待服务就绪
        while (!client_->wait_for_service(std::chrono::seconds(5))) {
            if (!rclcpp::ok()) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端！");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接中！");
        }
        // if (!client_->wait_for_service(std::chrono::seconds(5))) {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_service is not activate");
        //     return;
        // }

        auto request = std::make_shared<CameraService::Request>();
        // request->command = CameraService::Request::START_IMAGE_PUBLISH;
        // request->width = 640;
        // request->height = 480;
        // request->fps = 30;

        CameraClient::CallCamService(false, request, 640, 480, 30);
        auto future_result1 = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "请求stop服务");
        std::future_status status1 = future_result1.wait_for(std::chrono::seconds(10));
        printf("status1 = %d\n", status1);
        // printf("result1 = %d\n", future_result1.get()->result);

        CameraClient::CallCamService(true, request, 640, 480, 30);
        auto future_result = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "请求start服务");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_result.get();
            std::future_status status = future_result.wait_for(std::chrono::seconds(10));
            if (result->result == CameraService::Response::RESULT_SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "contrl camera succeed 2！: ");
                // 订阅topic
                color_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "image", rclcpp::SystemDefaultsQoS(),
                    std::bind(&CameraClient::colorCallback, this, std::placeholders::_1));
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "contrl camera failed 2:");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用未能完成。");
        }

        // std::future_status status = future_result.wait_for(std::chrono::seconds(10));
        // printf("status = %d\n", status);
        // printf("result = %d\n", future_result.get()->result);
        // if (status == std::future_status::ready) {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ready success 1！: ");
            
        //     // uint8_t result = future_result.get()->result;
        //     if (future_result.get()->result == CameraService::Response::RESULT_SUCCESS) {
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "contrl camera succeed 2！: ");
        //         // 订阅topic
        //         auto rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        //             "image", rclcpp::SystemDefaultsQoS(),
        //             std::bind(&CameraClient::colorCallback, this, std::placeholders::_1));
        //     } else {
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "contrl camera failed 2:");
        //     }
        // } else {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ready failed 1！:");
        // }

        // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应失败！");
        // } else {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功！");
        //     // auto result = client_->async_send_request(request);
        //     if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
        //         if (result.get()->result == CameraService::Response::RESULT_SUCCESS) {
        //             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功！");
        //             // ros2::this_node::getNamespace();
        //             // 订阅topic
        //             auto rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        //                 "image", rclcpp::SystemDefaultsQoS(),
        //                 std::bind(&CameraClient::colorCallback, this, std::placeholders::_1));


        //         } else {
        //             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应失败！");
        //         }
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功！");
        //     } else {
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应失败！");
        //     }
        // }
     }
    // 3-2.等待服务连接（对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求）；
    /*
        连接服务器实现，如果成功返回ture,如果是失败，返回false;
     */
    bool connect_service() {
        // 设置指定超时时间内连接服务器，如果连接上，返回ture,否则返回false;
        // client_->wait_for_service(1s);
        // 循环以1s为超时时间连接服务器，直到连接到服务器才退出循环
        while(!client_->wait_for_service(5s)) {

            // 按下ctrlC是要结束ros2程序，意味着要释放资源，比如：关闭 context；
            if (!rclcpp::ok()) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端！");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接中！");
        }

        return true;
    }
    void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_image;
        RCLCPP_INFO(this->get_logger(), "input colorCallback");
        try
        {
            cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
        }

        // 获取彩色图像
        cv::Mat color_image = cv_image->image;
        // 检查图像是否成功读取
        if (color_image.empty()) {
            std::cerr << "无法读取图像文件" << std::endl;
            return;
        }
        // 在彩色图像上执行视觉开发操作
        // ...
        
        // 显示彩色图像
        RCLCPP_INFO(this->get_logger(), "save color image");
        for (int i = 0; i<1; i++)
        {
            bool success = cv::imwrite("/home/mi/Camera/Color_Image.png", color_image);
                // 检查图像是否成功保存
            if (success) {
                std::cout << "图像保存成功" << std::endl;
            } else {
                std::cerr << "图像保存失败" << std::endl;
                return;
            }
        }
        while (!client_->wait_for_service(std::chrono::seconds(5))) {
            if (!rclcpp::ok()) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端！");
                auto requestend = std::make_shared<CameraService::Request>();
                CameraClient::CallCamService(false, requestend, 640, 480, 30);
                auto future_result1 = client_->async_send_request(requestend);
                RCLCPP_INFO(this->get_logger(), "请求stop服务");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接中！");
        }
        // cv::waitKey(1);
    }


private:
    rclcpp::Client<CameraService>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscriber_;
    bool CallCamService(bool activate, std::shared_ptr<protocol::srv::CameraService::Request> &request, int height, int width, int fps)
    {
        request->command = activate ?
            protocol::srv::CameraService::Request::START_IMAGE_PUBLISH :
            protocol::srv::CameraService::Request::STOP_IMAGE_PUBLISH;
        if (activate) {
            request->height = height;
            request->width = width;
            request->fps = fps;
        }
        return true;
    }
};

int main(int argc, char ** argv)
{

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "input main");
    // 2. 初始化ROS2客户端；
    rclcpp::init(argc, argv);
    // 4 创建客户端对象(自定义节点类型的)
    auto client_node = std::make_shared<CameraClient>();



    // //  调用客户端对象的连接服务器功能；
    // bool flag = client->connect_service();
    // // 根据连接结果做进一步处理
    // if (!flag) {
    //     /*
    //         rclcpp::get_logger("name")创建不依赖context
    //      */
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败，程序推出");
    //     return 0;
    // }

    rclcpp::spin(client_node);
    // 5. 释放资源
    rclcpp::shutdown();
    return 0;
}