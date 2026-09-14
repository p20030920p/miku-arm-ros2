#include <ros/ros.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    float rx_pos_7 = 0.0f;  // 回读的电机7位置
    float rx_vel_7 = 0.0f;  // 回读的电机7速度
    float rx_tor_7 = 0.0f;  // 回读的电机7力矩

    float claw_max_pos = 0.0 * M_PI / 180.0;   // 夹爪张开上限
    float claw_min_pos = -165 * M_PI / 180.0; // 夹爪闭合下限 

public:
    TestNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TestNode::msgCallback, this);

        ROS_INFO("Claw_Test 启动成功 ");
        // ROS_INFO("输入格式：状态   模式   速度    转矩  位置1  位置2");
        // ROS_INFO("输入示例: 1     2     3.14   3.14  3.14  3.14");
    }

    // 订阅回调函数
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg) {
        // ROS_INFO("订阅成功\n ");
        // ROS_INFO("Received msg: ");
        // ROS_INFO("【电机1】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_1, msg->vel_1,msg->tor_1);
        // ROS_INFO("【电机2】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_2, msg->vel_2,msg->tor_2);
        rx_tor_7= msg->tor_7;
        rx_pos_7= msg->pos_7;
        rx_vel_7= msg->vel_7;
    }

    // 运行输入循环
    void run() {
        
        int flag=0;
        const char* state; // 状态字符

        while(ros::ok()) {
            arm_control::ArmMsg msg;
            
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
            //     msg.Task_status = task_status;
            //     msg.mode = mode;
            //     msg.vel = vel;
            //     msg.tor = tor;
            //     msg.pos_1 = pos_1;
            //     msg.pos_2 = pos_2;
            // } catch (...) {
            //     ROS_ERROR("输入格式错误！");
            //     continue;
            // }
            float vel = 1.0;
            
            // 夹爪失去力矩
            if(flag == 0){
                state = "夹爪失去力矩"; // 夹爪失去力矩

                ros::spinOnce();
                // 赋值消息
                msg.Task_status = 1; // 机械臂使能
                msg.mode  = 1;       // 机械臂无力矩模式
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0.0;
                msg.kp = 0;
                msg.kd = 0;
                pub.publish(msg);
                usleep(10000);  // 10ms

                while (ros::ok() && fabs(rx_tor_7) > 0.01) { // 0.01Nm
                    ros::spinOnce();
                    // 赋值消息
                    msg.Task_status = 1; // 机械臂使能
                    msg.mode  = 1;       // 机械臂无力矩模式
                    msg.pos_7 = 0;
                    msg.vel_7 = 0;
                    msg.tor_7 = 0.0;
                    msg.kp = 0;
                    msg.kd = 0;
                    pub.publish(msg);
                    usleep(10000);  // 10ms
                }

                flag++;
            }
            
            // 夹爪移动到零点
            else if(flag == 1){
                state = "夹爪移动到零点"; // 夹爪移动到零点

                ros::spinOnce();
                msg.Task_status = 1;
                msg.mode  = 1;
                msg.pos_7 = 0;
                msg.vel_7 = vel;
                msg.tor_7 = 0.0;
                msg.kp = 6;
                msg.kd = 0.6;
                pub.publish(msg);
                usleep(10000);  // 10ms
                
                while (ros::ok() && fabs(rx_pos_7) > 0.0087) { // 0.5°=0.0087rad
                    ros::spinOnce();
                    msg.Task_status = 1;
                    msg.mode  = 2;
                    msg.pos_7 = 0;
                    msg.vel_7 = vel;
                    msg.tor_7 = 0.0;
                    msg.kp = 6;
                    msg.kd = 0.6;
                    pub.publish(msg);
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
                msg.Task_status = 1; // 机械臂使能
                msg.mode  = 2;       // 机械臂位置模式
                msg.pos_7 = claw_min_pos;  // 向最小距离夹取
                msg.vel_7 = vel;
                msg.tor_7 = 0;
                msg.kp = 6;
                msg.kd = 0.6;
                // 发布消息
                pub.publish(msg);
                ROS_INFO("正在夹取...");
                ROS_INFO("【夹爪】模式:%d 位置:%.2f° 速度:%.4f 转矩:%.4f",
                            msg.mode,
                            msg.pos_7 / M_PI * 180,   // 弧度转角度，更直观
                            msg.vel_7,
                            msg.tor_7);
                

                // 处理ROS回调
                ros::spinOnce();
                // 控制循环频率
                usleep(10000);  // 10ms

                while(!is_catch && rx_pos_7 > claw_min_pos+0.0087){ // 0.5°=0.0087rad
                    if(fabs(rx_tor_7) > 0.3){ // 0.3Nm
                        ROS_INFO("夹取成功！力矩值:%.4f 位置:%.2f°",rx_tor_7, rx_pos_7 / M_PI * 180);
                         // 更新夹爪位置
                        catch_pos = rx_pos_7-0.6;  // 比接触位置再闭合几度，产生夹紧预压
                        if (catch_pos < claw_min_pos) {
                            catch_pos = claw_min_pos;           // 不超出下限
                        }
                        is_catch = 1;

                        // 夹住过程 参数赋值
                        msg.Task_status = 1; // 机械臂使能
                        msg.mode  = 1;       // 机械臂MIT模式
                        msg.pos_7 = catch_pos;  // 锁定接触位置
                        msg.vel_7 = 0;
                        msg.tor_7 = 0.25;        // 恒定夹持力矩（适中，不夹坏物体）
                        msg.kp = 6.0;            // 强弹簧：物体试图撑开时产生大恢复力
                        msg.kd = 0.6;            // 适度阻尼，防止振荡

                        state = "夹取成功"; // 夹取成功

                    }else{
                        // 持续发送闭合指令（有了不亏，没它也能跑）
                        msg.Task_status = 1;
                        msg.mode  = 2;
                        msg.pos_7 = claw_min_pos;
                        msg.vel_7 = 0.3;
                        msg.tor_7 = 0;
                        msg.kp = 6;
                        msg.kd = 0.6;
                        // 发布消息
                        pub.publish(msg);
                        ROS_INFO("夹取中...力矩:%.4f 位置:%.2f°",rx_tor_7, rx_pos_7 / M_PI * 180);
                        // 处理ROS回调
                        ros::spinOnce();
                        // 控制循环频率
                        usleep(10000);  // 10ms
                    }
                }
                if(!is_catch){
                    ROS_INFO("夹取失败！力矩值:%.4f 位置:%.2f°",rx_tor_7, rx_pos_7 / M_PI * 180);
                    state = "夹取失败"; // 夹取失败
                }

                flag = 0;
            }
            
            // 发布消息
            pub.publish(msg);
            ROS_INFO("消息已发布! 状态：%s", state);
            ROS_INFO("【夹爪】模式:%d 目标位置:%.2f° 速度:%.4f 转矩:%.4f",
                     msg.mode, msg.pos_7 / M_PI * 180, msg.vel_7, msg.tor_7);

            // 处理ROS回调
            ros::spinOnce();
            usleep(10000);
        }

    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "test_node");
    TestNode node;
    node.run();
    return 0;
}