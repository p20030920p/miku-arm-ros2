// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp；节点类直接继承 rclcpp::Node，
//            不再使用 ros::NodeHandle，日志宏改为 RCLCPP_*（需传 logger）。
#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件
#include <cmath> // 包含cmath头文件以使用cosf函数

#include <clocale>    // setlocale（ROS 1 由 ros/ros.h 间接引入，ROS 2 需显式包含）
#include <functional> // std::bind
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>   // usleep

// 计算实时扭矩
float calculate_t(float L, float theta, float m, float g) {
    return L * cosf(theta) * m * g;
}

class TestNode : public rclcpp::Node {
private:
    // ROS 2 差异：ros::Publisher / ros::Subscriber → rclcpp 的 SharedPtr 句柄。
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;
    // ROS 2 差异：原工程在循环中用 ros::spinOnce() 泵回调，
    //             ROS 2 用 SingleThreadedExecutor::spin_some() 等价替代。
    rclcpp::executors::SingleThreadedExecutor executor;

    uint8_t Task_status_rx;
    uint8_t mode_rx;
    float pos_1_rx;
    float pos_2_rx;
    float vel_1_rx;
    float vel_2_rx;
    float tor_1_rx;
    float tor_2_rx;
    float kp_rx;
    float kd_rx;

    float L=0.15;
    float m=0.28;
    float g=9.8;

    arm_control::msg::ArmMsg msg;
    
public:
    TestNode() : Node("test_node") { // ROS 2 差异：ros::init(argc, argv, "test_node") 的节点名在这里给出
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
                "Arm_rx", 20, std::bind(&TestNode::msgCallback, this, std::placeholders::_1));
        // ROS 2 差异：把节点加入单线程执行器，供 run() 里的 spin_some() 使用。
        executor.add_node(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Pub_Test 启动成功 ");

        // // 开启电机力控模式
        // // 赋值消息
        // msg.task_status = 1;
        // msg.mode  = 1;
        // msg.pos_1 = 3.14/2;
        // msg.pos_2 = 3.14/2;
        // msg.vel_1 = 0;
        // msg.vel_2 = 0;
        // msg.tor_1 = 0.2;
        // msg.tor_2 = 0.01;
        // msg.kp    = 0;
        // msg.kd    = 0;
        // // 发布消息
        // pub->publish(msg);
    }

    // 订阅回调函数
    void msgCallback(const arm_control::msg::ArmMsg::SharedPtr msg) {
        // 说明：回调体在原工程里整段是注释（只保留订阅通道，不做处理），
        //       这里显式标注参数未使用，避免 -Wunused-parameter 告警。
        (void)msg;
        // RCLCPP_INFO(this->get_logger(), "订阅成功\n ");
        // RCLCPP_INFO(this->get_logger(), "Received msg: ");
        // RCLCPP_INFO(this->get_logger(), "【电机1】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_1, msg->vel_1,msg->tor_1);
        // RCLCPP_INFO(this->get_logger(), "【电机2】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_2, msg->vel_2,msg->tor_2);

        // Task_status_rx = msg->task_status; // ROS 2 差异：Task_status → task_status
        // mode_rx  = msg->mode;
        // pos_1_rx = msg->pos_1;
        // pos_2_rx = msg->pos_2;
        // vel_1_rx = msg->vel_1;
        // vel_2_rx = msg->vel_2;
        // tor_1_rx = msg->tor_1;
        // tor_2_rx = msg->tor_2;
        // kp_rx = msg->kp;
        // kd_rx = msg->kd;
    }

    // 运行输入循环
    void run() {
        
        int flag=0;

        msg.task_status = 1; // ROS 2 差异：Task_status → task_status
        msg.mode  = 1;

        msg.pos_1 = 0;
        msg.pos_2 = 0;
        msg.pos_3 = 0;
        msg.pos_4 = 0;
        msg.pos_5 = 0;
        msg.pos_6 = 0;

        msg.vel_1 = 0;
        msg.vel_2 = 0;
        msg.vel_3 = 0;
        msg.vel_4 = 0;
        msg.vel_5 = 0;
        msg.vel_6 = 0;

        msg.tor_1 = 0;
        msg.tor_2 = 0;
        msg.tor_3 = 0;
        msg.tor_4 = 0;
        msg.tor_5 = 0;
        msg.tor_6 = 0;

        msg.kp = 0;
        msg.kd = 0;

        // 发布消息
        pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()

        while(rclcpp::ok()) { // ROS 2 差异：ros::ok() → rclcpp::ok()
            
            // 获取用户输入
            std::cout << "\n按下【回车键】执行下一步操作）> ";
            std::string input;
            std::getline(std::cin, input);

            // // 提供转矩 补偿重力
            // msg.task_status = 1;
            // msg.mode  = 1;
            // msg.pos_1 = 3.14/2;
            // msg.pos_2 = 3.14/2;
            // msg.vel_1 = 0;
            // msg.vel_2 = 0;
            // msg.tor_1 = calculate_t(L,pos_1_rx,m,g);
            // msg.tor_2 = 0.01;
            // msg.kp    = 0;
            // msg.kd    = 0;

            msg.kp = 20;
            msg.kd = 5;

            // 全部移动至极限位置
            if(flag == 0){
                // 赋值消息
                msg.pos_6 = 0;

                flag++;
            }

            else if(flag == 1){
                // 赋值消息
                msg.pos_6 = 3;

                flag = 0;
            }

            // 发布消息
            pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
            RCLCPP_INFO(this->get_logger(), "消息已发布!");
            // RCLCPP_INFO(this->get_logger(), "flag:%d 状态:%d 模式:%d 位置1:%.2f 速度1:%.2f 转矩1:%.2f 位置2:%.2f 速度2:%.2f 转矩2:%.2f"
            //         ,flag,msg.task_status,msg.mode,msg.pos_1,msg.vel_1,msg.tor_1,msg.pos_2,msg.vel_2,msg.tor_2);
            RCLCPP_INFO(this->get_logger(), "flag:%d 状态:%d 模式:%d 位置6:%.2f"
                    ,flag,msg.task_status,msg.mode,msg.pos_6);

            // 处理ROS回调
            executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
            // 控制循环频率
            usleep(10000);  // 10ms
        }

    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    // ROS 2 差异：ros::init(argc, argv, "test_node") → rclcpp::init(argc, argv)，
    //            节点名在 TestNode 的构造函数中以 Node("test_node") 指定。
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    node->run();
    rclcpp::shutdown(); // ROS 2 差异：显式关闭 rclcpp
    return 0;
}
