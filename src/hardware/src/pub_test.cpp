// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp；节点类直接继承 rclcpp::Node，
//            不再使用 ros::NodeHandle，日志宏改为 RCLCPP_*（需传 logger）。
#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件

#include <clocale>   // setlocale（ROS 1 由 ros/ros.h 间接引入，ROS 2 需显式包含）
#include <functional> // std::bind
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>  // usleep

class TestNode : public rclcpp::Node {
private:
    // ROS 2 差异：ros::Publisher / ros::Subscriber → rclcpp 的 SharedPtr 句柄。
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;
    // ROS 2 差异：原工程在循环中用 ros::spinOnce() 泵回调，
    //             ROS 2 用 SingleThreadedExecutor::spin_some() 等价替代。
    rclcpp::executors::SingleThreadedExecutor executor;

public:
    TestNode() : Node("test_node") { // ROS 2 差异：ros::init(argc, argv, "test_node") 的节点名在这里给出
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
                "Arm_rx", 20, std::bind(&TestNode::msgCallback, this, std::placeholders::_1));
        // ROS 2 差异：把节点加入单线程执行器，供 run() 里的 spin_some() 使用。
        executor.add_node(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Pub_Test 启动成功 ");
        // RCLCPP_INFO(this->get_logger(), "输入格式：状态   模式   速度    转矩  位置1  位置2");
        // RCLCPP_INFO(this->get_logger(), "输入示例: 1     2     3.14   3.14  3.14  3.14");
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
    }

    // 运行输入循环
    void run() {
        
        int flag=0;

        while(rclcpp::ok()) { // ROS 2 差异：ros::ok() → rclcpp::ok()
            arm_control::msg::ArmMsg msg;
            
            // 获取用户输入
            std::cout << "\n按下【回车键】执行下一步操作）> ";
            std::string input;
            std::getline(std::cin, input);

            // // 解析输入数据
            // int task_status, mode;
            // float vel, tor, pos_1, pos_2;

            // std::istringstream iss(input);
            // try {
            //     iss >> task_status;      // 先提取为整数
            //     iss >> mode;             // 再提取为整数
            //     iss >> vel >> tor >> pos_1 >> pos_2;

            //     // 赋值到消息
            //     msg.task_status = task_status;
            //     msg.mode = mode;
            //     msg.vel = vel;
            //     msg.tor = tor;
            //     msg.pos_1 = pos_1;
            //     msg.pos_2 = pos_2;
            // } catch (...) {
            //     RCLCPP_ERROR(this->get_logger(), "输入格式错误！");
            //     continue;
            // }
            float vel = 0.5;
            
            // 全部移动至极限位置
            if(flag == 0){
                // 赋值消息
                msg.task_status = 1; // 机械臂使能（ROS 2 差异：Task_status → task_status）
                msg.mode  = 1;       // 机械臂无力矩模式

                msg.pos_1 = 0;
                msg.pos_2 = -M_PI/2;
                msg.pos_3 = -M_PI/2;
                msg.pos_4 = -M_PI/2;
                msg.pos_5 = -M_PI/2;
                msg.pos_6 = -M_PI/2;

                msg.vel_1 = vel;
                msg.vel_2 = vel;
                msg.vel_3 = vel;
                msg.vel_4 = vel;
                msg.vel_5 = vel;
                msg.vel_6 = vel;

                // msg.tor_1 = 0.01;
                // msg.tor_2 = 0.01;
                // msg.tor_3 = 0.01;
                // msg.tor_4 = 0.01;
                // msg.tor_5 = 0.01;
                // msg.tor_6 = 0.01;

                // 夹爪
                // msg.pos_7 = -M_PI/2;
                // msg.vel_7 = vel;
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0.0;

                // msg.kp = 6;
                // msg.kd = 0.6;

                msg.kp = 0;
                msg.kd = 0;

                flag++;
            }
            // 全部移动到零点
            else if(flag == 1){
                // 赋值消息
                msg.task_status = 1; // 机械臂使能
                msg.mode  = 2;       // 机械臂位置模式

                msg.pos_1 = 0;
                msg.pos_2 = 0;
                msg.pos_3 = 0;
                msg.pos_4 = 0;
                msg.pos_5 = 0;
                msg.pos_6 = 0;

                msg.vel_1 = vel;
                msg.vel_2 = vel;
                msg.vel_3 = vel;
                msg.vel_4 = vel;
                msg.vel_5 = vel;
                msg.vel_6 = vel;

                // msg.tor_1 = 0.01;
                // msg.tor_2 = 0.01;
                // msg.tor_3 = 0.01;
                // msg.tor_4 = 0.01;
                // msg.tor_5 = 0.01;
                // msg.tor_6 = 0.01;
                
                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = vel;
                // msg.tor_7 = 0.01;

                msg.kp = 6;
                msg.kd = 0.6;

                flag=0;
            }
            // 无力矩模式
            else if(flag == 2){
                // 赋值消息
                msg.task_status = 1; // 机械臂使能
                msg.mode  = 1;       // 机械臂无力矩模式

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

                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0;

                msg.kp = 0;
                msg.kd = 0;

                flag ++;
            }
            // 全部再次移动到零点
            else if(flag == 3){
                // 赋值消息
                msg.task_status = 1; // 机械臂使能
                msg.mode  = 2;       // 机械臂位置模式

                msg.pos_1 = 0;
                msg.pos_2 = 0;
                msg.pos_3 = 0;
                msg.pos_4 = 0;
                msg.pos_5 = 0;
                msg.pos_6 = 0;

                msg.vel_1 = vel;
                msg.vel_2 = vel;
                msg.vel_3 = vel;
                msg.vel_4 = vel;
                msg.vel_5 = vel;
                msg.vel_6 = vel;

                // msg.tor_1 = 0.01;
                // msg.tor_2 = 0.01;
                // msg.tor_3 = 0.01;
                // msg.tor_4 = 0.01;
                // msg.tor_5 = 0.01;
                // msg.tor_6 = 0.01;

                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = vel;
                // msg.tor_7 = 0.01;

                msg.kp = 6;
                msg.kd = 0.6;

                flag++;
            }
            // 机械臂失能
            else if(flag == 4){
                // 赋值消息
                msg.task_status = 0; // 机械臂失能
                msg.mode  = 1;       // 机械臂无力矩模式
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

                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0;

                msg.kp = 0;
                msg.kd = 0;

                flag = 0;
            }
            // 发布消息
            pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
            RCLCPP_INFO(this->get_logger(), "消息已发布!,flag=%d",flag);
            RCLCPP_INFO(this->get_logger(), "位置1:%.2f 位置2:%.2f 位置3:%.2f 位置4:%.2f 位置5:%.2f 位置6:%.2f 位置7:%.2f"
                    ,msg.pos_1,msg.pos_2,msg.pos_3,msg.pos_4,msg.pos_5,msg.pos_6,msg.pos_7);
            RCLCPP_INFO(this->get_logger(), "【夹爪】模式:%d 位置:%.2f° 速度:%.4f 转矩:%.4f",
                        msg.mode,
                        msg.pos_7 / M_PI * 180,   // 弧度转角度，更直观
                        msg.vel_7,
                        msg.tor_7);
                        
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
