#include "fairino_hardware/command_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "libfairino/include/robot.h"

int main(int argc, char *argv[]){
    //该main函数用于创建简化指令客户端的app
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor mulexecutor;
    // 指令服务与状态话题均由同一 SDK 连接提供。状态使用 20004 实时接口，
    // 不再启动与 V3.8.8-LA 结构不兼容的 8081 原始帧解析节点。
    auto command_server_node = std::make_shared<robot_command_thread>("fr_command_server");
    mulexecutor.add_node(command_server_node);
    mulexecutor.spin();
    rclcpp::shutdown();
    return 0;
}
