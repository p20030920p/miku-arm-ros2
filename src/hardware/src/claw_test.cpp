// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp；节点类直接继承 rclcpp::Node，
//            不再使用 ros::NodeHandle，日志宏改为 RCLCPP_*（需传 logger）。
#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件

#include <clocale>    // setlocale（ROS 1 由 ros/ros.h 间接引入，ROS 2 需显式包含）
#include <cmath>      // fabs / M_PI
#include <functional> // std::bind
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>   // usleep

class TestNode : public rclcpp::Node {
private:
    // ROS 2 差异：ros::Publisher / ros::Subscriber → rclcpp 的 SharedPtr 句柄。
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;
    // ROS 2 差异：原工程在循环中用 ros::spinOnce() 泵回调，
    //             ROS 2 用 SingleThreadedExecutor::spin_some() 等价替代。
    rclcpp::executors::SingleThreadedExecutor executor;

    float rx_pos_7 = 0.0f;  // 回读的电机7位置
    float rx_vel_7 = 0.0f;  // 回读的电机7速度
    float rx_tor_7 = 0.0f;  // 回读的电机7力矩

    float claw_max_pos = 0.0 * M_PI / 180.0;   // 夹爪张开上限
    float claw_min_pos = -165 * M_PI / 180.0; // 夹爪闭合下限 

public:
    TestNode() : Node("test_node") { // ROS 2 差异：ros::init(argc, argv, "test_node") 的节点名在这里给出
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
                "Arm_rx", 20, std::bind(&TestNode::msgCallback, this, std::placeholders::_1));
        // ROS 2 差异：把节点加入单线程执行器，供 run() 里的 spin_some() 使用。
        executor.add_node(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Claw_Test 启动成功 ");
        // RCLCPP_INFO(this->get_logger(), "输入格式：状态   模式   速度    转矩  位置1  位置2");
        // RCLCPP_INFO(this->get_logger(), "输入示例: 1     2     3.14   3.14  3.14  3.14");
    }

    // 订阅回调函数
    void msgCallback(const arm_control::msg::ArmMsg::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "订阅成功\n ");
        // RCLCPP_INFO(this->get_logger(), "Received msg: ");
        // RCLCPP_INFO(this->get_logger(), "【电机1】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_1, msg->vel_1,msg->tor_1);
        // RCLCPP_INFO(this->get_logger(), "【电机2】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_2, msg->vel_2,msg->tor_2);
        rx_tor_7= msg->tor_7;
        rx_pos_7= msg->pos_7;
        rx_vel_7= msg->vel_7;
    }

    // 运行输入循环
    void run() {
        
        int flag=0;
        const char* state; // 状态字符

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
            float vel = 1.0;
            
            // 夹爪失去力矩
            if(flag == 0){
                state = "夹爪失去力矩"; // 夹爪失去力矩

                executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
                // 赋值消息
                msg.task_status = 1; // 机械臂使能（ROS 2 差异：Task_status → task_status）
                msg.mode  = 1;       // 机械臂无力矩模式
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0.0;
                msg.kp = 0;
                msg.kd = 0;
                pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
                usleep(10000);  // 10ms

                while (rclcpp::ok() && fabs(rx_tor_7) > 0.01) { // 0.01Nm
                    executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
                    // 赋值消息
                    msg.task_status = 1; // 机械臂使能
                    msg.mode  = 1;       // 机械臂无力矩模式
                    msg.pos_7 = 0;
                    msg.vel_7 = 0;
                    msg.tor_7 = 0.0;
                    msg.kp = 0;
                    msg.kd = 0;
                    pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
                    usleep(10000);  // 10ms
                }

                flag++;
            }
            
            // 夹爪移动到零点
            else if(flag == 1){
                state = "夹爪移动到零点"; // 夹爪移动到零点

                executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
                msg.task_status = 1;
                msg.mode  = 1;
                msg.pos_7 = 0;
                msg.vel_7 = vel;
                msg.tor_7 = 0.0;
                msg.kp = 6;
                msg.kd = 0.6;
                pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
                usleep(10000);  // 10ms
                
                while (rclcpp::ok() && fabs(rx_pos_7) > 0.0087) { // 0.5°=0.0087rad
                    executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
                    msg.task_status = 1;
                    msg.mode  = 2;
                    msg.pos_7 = 0;
                    msg.vel_7 = vel;
                    msg.tor_7 = 0.0;
                    msg.kp = 6;
                    msg.kd = 0.6;
                    pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
                    usleep(10000);  // 10ms
                }
                flag++;
            }

            // 夹爪开始夹取
            else if(flag == 2){
                // 定义状态变量
                int is_catch = 0; // 1表示夹取，0表示释放
                float catch_pos = claw_min_pos; // 夹取位置，单位为弧度

                // 夹取过程 参数赋值
                msg.task_status = 1; // 机械臂使能
                msg.mode  = 2;       // 机械臂位置模式
                msg.pos_7 = claw_min_pos;  // 向最小距离夹取
                msg.vel_7 = vel;
                msg.tor_7 = 0;
                msg.kp = 6;
                msg.kd = 0.6;
                // 发布消息
                pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
                RCLCPP_INFO(this->get_logger(), "正在夹取...");
                RCLCPP_INFO(this->get_logger(), "【夹爪】模式:%d 位置:%.2f° 速度:%.4f 转矩:%.4f",
                            msg.mode,
                            msg.pos_7 / M_PI * 180,   // 弧度转角度，更直观
                            msg.vel_7,
                            msg.tor_7);
                

                // 处理ROS回调
                executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
                // 控制循环频率
                usleep(10000);  // 10ms

                while(!is_catch && rx_pos_7 > claw_min_pos+0.0087){ // 0.5°=0.0087rad
                    if(fabs(rx_tor_7) > 0.3){ // 0.3Nm
                        RCLCPP_INFO(this->get_logger(), "夹取成功！力矩值:%.4f 位置:%.2f°",rx_tor_7, rx_pos_7 / M_PI * 180);
                         // 更新夹爪位置
                        catch_pos = rx_pos_7-0.6;  // 比接触位置再闭合几度，产生夹紧预压
                        if (catch_pos < claw_min_pos) {
                            catch_pos = claw_min_pos;           // 不超出下限
                        }
                        is_catch = 1;

                        // 夹住过程 参数赋值
                        msg.task_status = 1; // 机械臂使能
                        msg.mode  = 1;       // 机械臂MIT模式
                        msg.pos_7 = catch_pos;  // 锁定接触位置
                        msg.vel_7 = 0;
                        msg.tor_7 = 0.25;        // 恒定夹持力矩（适中，不夹坏物体）
                        msg.kp = 6.0;            // 强弹簧：物体试图撑开时产生大恢复力
                        msg.kd = 0.6;            // 适度阻尼，防止振荡

                        state = "夹取成功"; // 夹取成功

                    }else{
                        // 持续发送闭合指令（有了不亏，没它也能跑）
                        msg.task_status = 1;
                        msg.mode  = 2;
                        msg.pos_7 = claw_min_pos;
                        msg.vel_7 = 0.3;
                        msg.tor_7 = 0;
                        msg.kp = 6;
                        msg.kd = 0.6;
                        // 发布消息
                        pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
                        RCLCPP_INFO(this->get_logger(), "夹取中...力矩:%.4f 位置:%.2f°",rx_tor_7, rx_pos_7 / M_PI * 180);
                        // 处理ROS回调
                        executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
                        // 控制循环频率
                        usleep(10000);  // 10ms
                    }
                }
                if(!is_catch){
                    RCLCPP_INFO(this->get_logger(), "夹取失败！力矩值:%.4f 位置:%.2f°",rx_tor_7, rx_pos_7 / M_PI * 180);
                    state = "夹取失败"; // 夹取失败
                }

                flag = 0;
            }
            
            // 发布消息
            pub->publish(msg); // ROS 2 差异：pub.publish() → pub->publish()
            RCLCPP_INFO(this->get_logger(), "消息已发布! 状态：%s", state);
            RCLCPP_INFO(this->get_logger(), "【夹爪】模式:%d 目标位置:%.2f° 速度:%.4f 转矩:%.4f",
                     msg.mode, msg.pos_7 / M_PI * 180, msg.vel_7, msg.tor_7);

            // 处理ROS回调
            executor.spin_some(); // ROS 2 差异：ros::spinOnce() → executor.spin_some()
            usleep(10000);
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
