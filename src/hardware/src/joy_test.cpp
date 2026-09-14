// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp；节点类直接继承 rclcpp::Node，
//            不再使用 ros::NodeHandle，日志宏改为 RCLCPP_*（需传 logger）。
#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件
#include <sensor_msgs/msg/joy.hpp>

#include <clocale>    // setlocale（ROS 1 由 ros/ros.h 间接引入，ROS 2 需显式包含）
#include <cmath>      // M_PI
#include <functional> // std::bind
#include <iostream>
#include <memory>
#include <string>

class TestNode : public rclcpp::Node {
private:
    // ROS 2 差异：ros::Publisher / ros::Subscriber → rclcpp 的 SharedPtr 句柄。
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;  // 手柄订阅者

    arm_control::msg::ArmMsg msg;

public:
    TestNode() : Node("test_node") { // ROS 2 差异：ros::init(argc, argv, "test_node") 的节点名在这里给出
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
                "Arm_rx", 20, std::bind(&TestNode::msgCallback, this, std::placeholders::_1));

        // 新增：订阅手柄话题
        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&TestNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "已启用手柄控制");

        // 初始化默认消息
        msg.task_status = 1; // ROS 2 差异：Task_status → task_status
        msg.mode = 1;

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

    // 新增：手柄回调函数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
        /* 手柄按键映射示例（PS4布局）：
        axes[0]: 左摇杆左右 → 关节1位置
        axes[1]: 左摇杆上下 → 关节2位置
        buttons[0]: X按钮 → 启动/暂停 (0/1)
        buttons[1]: ○按钮 → 急停
        buttons[4]: L1 → 模式1 (MIT)
        buttons[5]: R1 → 模式2 (位置)
        */
        
        RCLCPP_INFO(this->get_logger(), "手柄 订阅成功\n ");

        // 1. 更新任务状态
        if (joy->buttons[2] || joy->buttons[2]) {        // ○按钮：急停
            msg.task_status = 0; // ROS 2 差异：Task_status → task_status
        } else if (joy->buttons[0] || joy->buttons[3]) { // X按钮：运行
            msg.task_status = 1;
        }

        // 2. 更新控制模式
        if (joy->buttons[9]) {        // 左肩键: MIT模式
            msg.mode = 1;
        } else if (joy->buttons[10]) { // 右肩键: 位置模式
            msg.mode = 2;
        }

        // 3. 更新关节位置（摇杆值范围[-1,1] → 转换为[-π,π]）
        if(msg.mode == 1){
            msg.tor_1 = joy->axes[0]; 
            msg.tor_2 = joy->axes[0]; 
            msg.tor_3 = joy->axes[0]; 
            msg.tor_4 = joy->axes[0]; 
            msg.tor_5 = joy->axes[0]; 
            msg.tor_6 = joy->axes[0];  
        }
        else if(msg.mode == 2){
            msg.pos_1 = joy->axes[0]*M_PI; 
            msg.pos_2 = joy->axes[0]*M_PI; 
            msg.pos_3 = joy->axes[0]*M_PI; 
            msg.pos_4 = joy->axes[0]*M_PI; 
            msg.pos_5 = joy->axes[0]*M_PI; 
            msg.pos_6 = joy->axes[0]*M_PI; 
        }

        // 4. 发布更新后的指令
        pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
        RCLCPP_INFO(this->get_logger(), "手柄指令已发布: 状态=%d, 模式=%d, 关节1=%.2f 关节2=%.2f 关节3=%.2f 关节4=%.2f 关节5=%.2f 关节6=%.2f", 
                msg.task_status, msg.mode, msg.tor_1, msg.tor_2, msg.tor_3, msg.tor_4, msg.tor_5, msg.tor_6);
    }

    // 运行输入循环
    void run() {
        
        // int flag=0;

        // while(rclcpp::ok()) {
        //     Simple_Arm::ArmMsg msg;
            
        //     // 获取用户输入
        //     std::cout << "\n按下【回车键】执行下一步操作）> ";
        //     std::string input;
        //     std::getline(std::cin, input);

        //     // // 解析输入数据
        //     // int task_status, mode;
        //     // float vel, tor, pos_1, pos_2;

        //     // std::istringstream iss(input);
        //     // try {
        //     //     iss >> task_status;      // 先提取为整数
        //     //     iss >> mode;             // 再提取为整数
        //     //     iss >> vel >> tor >> pos_1 >> pos_2;

        //     //     // 赋值到消息
        //     //     msg.task_status = task_status;
        //     //     msg.mode = mode;
        //     //     msg.vel = vel;
        //     //     msg.tor = tor;
        //     //     msg.pos_1 = pos_1;
        //     //     msg.pos_2 = pos_2;
        //     // } catch (...) {
        //     //     RCLCPP_ERROR(this->get_logger(), "输入格式错误！");
        //     //     continue;
        //     // }
            
        //     if(flag == 0){
        //         // 赋值消息
        //         msg.task_status = 1;
        //         msg.mode  = 2;
        //         msg.pos_1 = 3.14;
        //         msg.pos_2 = 3.14;
        //         msg.pos_3 = 3.14;
        //         msg.pos_4 = 3.14;
        //         msg.pos_5 = 3.14;
        //         msg.pos_6 = 3.14;
        //         msg.vel_1 = 1;
        //         msg.vel_2 = 1;
        //         msg.vel_3 = 1;
        //         msg.vel_4 = 1;
        //         msg.vel_5 = 1;
        //         msg.vel_6 = 1;
        //         msg.kp = 6;
        //         msg.kd = 0.6;

        //         flag++;
        //     }
        //     // else if(flag == 1){
        //     //     // 赋值消息
        //     //     msg.task_status = 1;
        //     //     msg.mode  = 1;
        //     //     msg.pos_1 = 3.14*2/3;
        //     //     msg.pos_2 = 3.14*2/3;
        //     //     msg.vel_1 = 0;
        //     //     msg.vel_2 = 0;
        //     //     msg.tor_1 = 0.5;
        //     //     msg.tor_2 = 0.5;

        //     //     flag++;
        //     // }
        //     // else if(flag == 2){
        //     //     // 赋值消息
        //     //     msg.task_status = 1;
        //     //     msg.mode  = 1;
        //     //     msg.pos_1 = 3.14/3;
        //     //     msg.pos_2 = 3.14/3;
        //     //     msg.vel_1 = 0;
        //     //     msg.vel_2 = 0;
        //     //     msg.tor_1 = 0.5;
        //     //     msg.tor_2 = 0.5;

        //     //     flag++;
        //     // }
        //     else if(flag == 1){
        //         // 赋值消息
        //         msg.task_status = 1;
        //         msg.mode  = 2;
        //         msg.pos_1 = 0;
        //         msg.pos_2 = 0;
        //         msg.pos_3 = 0;
        //         msg.pos_4 = 0;
        //         msg.pos_5 = 0;
        //         msg.pos_6 = 0;
        //         msg.vel_1 = 1;
        //         msg.vel_2 = 1;
        //         msg.vel_3 = 1;
        //         msg.vel_4 = 1;
        //         msg.vel_5 = 1;
        //         msg.vel_6 = 1;
        //         msg.kp = 6;
        //         msg.kd = 0.6;

        //         flag = 0;
        //     }
            
        //     // 发布消息
        //     pub->publish(msg);
        //     RCLCPP_INFO(this->get_logger(), "消息已发布!");
        //     RCLCPP_INFO(this->get_logger(), "flag:%d 状态:%d 模式:%d 位置1:%.2f 速度1:%.2f 转矩1:%.2f 位置2:%.2f 速度2:%.2f 转矩2:%.2f"
        //             ,flag,msg.task_status,msg.mode,msg.pos_1,msg.vel_1,msg.tor_1,msg.pos_2,msg.vel_2,msg.tor_2);

            // 处理ROS回调
            // executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
            rclcpp::spin(this->shared_from_this()); // ROS 2 差异：ros::spin() → rclcpp::spin(node)
            // 控制循环频率
            // usleep(10000);  // 10ms
        // }

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
