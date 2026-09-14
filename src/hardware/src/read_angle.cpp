// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp；节点类直接继承 rclcpp::Node，
//            不再使用 ros::NodeHandle，日志宏改为 RCLCPP_*（需传 logger）。
#include <rclcpp/rclcpp.hpp>
// ROS 2 差异：ros/package.h（ros::package::getPath）在 ROS 2 中没有对应物，
//            改用 ament_index_cpp::get_package_share_directory 获取包目录。
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <thread> // <-- 引入线程库
#include <clocale>    // setlocale（ROS 1 由 ros/ros.h 间接引入，ROS 2 需显式包含）
#include <functional> // std::bind
#include <memory>

class TestNode : public rclcpp::Node {
private:
    // ROS 2 差异：ros::Publisher / ros::Subscriber → rclcpp 的 SharedPtr 句柄。
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;

    arm_control::msg::ArmMsg pub_msg;

    // --- 用于示教录制的参数 ---
    std::ofstream data_file;
    bool is_recording = false;
    arm_control::msg::ArmMsg current_arm_state;
    rclcpp::Time record_start_time; // ROS 2 差异：ros::Time → rclcpp::Time
    // ROS 2 差异：ros::Timer → rclcpp 的 WallTimer（回调不带参数）。
    rclcpp::TimerBase::SharedPtr record_timer;
    rclcpp::Time last_msg_time; // 记录上一条消息的时间
    bool initial_data_error_logged = false; // 控制初始错误只报一次
    bool data_update_warning_logged = false; // 控制更新警告只报一次
    // --------------------------

public:
    TestNode() : Node("teach_node") { // ROS 2 差异：ros::init(argc, argv, "teach_node") 的节点名在这里给出
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
                "Arm_rx", 20, std::bind(&TestNode::msgCallback, this, std::placeholders::_1));

        // 创建一个定时器用于以固定频率记录数据，但先不启动
        // ROS 2 差异：ros::Duration(0.001)（1000Hz，原注释写 100Hz）→ 1ms wall timer；
        //            ROS 1 的 createTimer(..., false) 表示 autostart=false，
        //            ROS 2 同样用 autostart=false 创建，start() → reset()、stop() → cancel()。
        //            ROS 2 的 wall timer 精度受执行器调度限制，无法严格保证 1000Hz。
        record_timer = this->create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(&TestNode::recordCallback, this),
                nullptr,   // 使用默认回调组
                false);    // autostart=false，等价于 ROS 1 的 createTimer(..., false)

        RCLCPP_INFO(this->get_logger(), "示教节点启动成功。");
    }

    // 订阅回调函数，用于实时更新机械臂状态
    void msgCallback(const arm_control::msg::ArmMsg::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "订阅成功\n ");
        
        // 将收到的最新消息存储在成员变量中
        current_arm_state = *msg;
        last_msg_time = this->now(); // 更新收到消息的时间（ROS 2 差异：ros::Time::now() → this->now()）
        data_update_warning_logged = false; // 收到数据，重置警告状态
    }

    // 定时器回调函数，用于将数据写入文件
    // ROS 2 差异：ROS 2 的 timer 回调不接收 ros::TimerEvent 参数。
    void recordCallback() {
        if (!is_recording) return;

        rclcpp::Time now = this->now(); // ROS 2 差异：ros::Time::now() → this->now()
        double time_since_last_msg = (now - last_msg_time).seconds(); // ROS 2 差异：toSec() → seconds()

        // 检查1：开始录制后，如果长时间没收到数据，则报错
        // ROS 2 差异：rclcpp::Time 没有 is_zero()，默认构造（零值）用 nanoseconds()==0 判断
        if (last_msg_time.nanoseconds() == 0) { // is_zero() 表示从未收到过消息
            if (time_since_last_msg > 2.0 && !initial_data_error_logged) {
                RCLCPP_ERROR(this->get_logger(), "开始录制后超过2秒未收到任何机械臂数据，请检查 'Arm_rx' 话题！");
                initial_data_error_logged = true; // 确保只报错一次
            }
            return; // 没有数据，不执行写入
        }

        // 检查2：录制过程中，如果数据长时间不更新，则告警
        if (time_since_last_msg > 2.0 && !data_update_warning_logged) {
            RCLCPP_WARN(this->get_logger(), "机械臂数据超过2秒未更新！");
            data_update_warning_logged = true; // 确保只告警一次，直到下次数据恢复
        }

        // 写入文件
        if (!data_file.is_open()) return;
        rclcpp::Duration time_since_start = now - record_start_time; // ROS 2 差异：ros::Duration → rclcpp::Duration
        data_file << std::fixed << std::setprecision(4) << time_since_start.seconds() << " "
                  << current_arm_state.pos_1 << " " << current_arm_state.pos_2 << " "
                  << current_arm_state.pos_3 << " " << current_arm_state.pos_4 << " "
                  << current_arm_state.pos_5 << " " << current_arm_state.pos_6 << " "
                  << current_arm_state.vel_1 << " " << current_arm_state.vel_2 << " "
                  << current_arm_state.vel_3 << " " << current_arm_state.vel_4 << " "
                  << current_arm_state.vel_5 << " " << current_arm_state.vel_6 << std::endl;

        // 打印已写入数据，便于调试
        RCLCPP_INFO(this->get_logger(), "\nRecorded data at %.4f s: \n  pos: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n  vel: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  time_since_start.seconds(),
                  current_arm_state.pos_1, current_arm_state.pos_2,
                  current_arm_state.pos_3, current_arm_state.pos_4,
                  current_arm_state.pos_5, current_arm_state.pos_6,
                  current_arm_state.vel_1, current_arm_state.vel_2,
                  current_arm_state.vel_3, current_arm_state.vel_4,
                  current_arm_state.vel_5, current_arm_state.vel_6);
    }

    // 运行主循环，控制录制的开始和停止
    void run() {
        // 发布机械臂使能消息，机械臂恢复到初始位置
        pub_msg.task_status = 1; // ROS 2 差异：Task_status → task_status
        pub_msg.mode  = 2;
        pub_msg.pos_1 = 0;
        pub_msg.pos_2 = 0;
        pub_msg.pos_3 = 0;
        pub_msg.pos_4 = 0;
        pub_msg.pos_5 = 0;
        pub_msg.pos_6 = 0;
        pub_msg.vel_1 = 0.5;
        pub_msg.vel_2 = 0.5;
        pub_msg.vel_3 = 0.5;
        pub_msg.vel_4 = 0.5;
        pub_msg.vel_5 = 0.5;
        pub_msg.vel_6 = 0.5;

        while(rclcpp::ok()) { // ROS 2 差异：ros::ok() → rclcpp::ok()
            if (!is_recording) {
                std::cout << "\n按下【回车键】开始录制...> ";
            } else {
                std::cout << "\n正在录制... 按下【回车键】停止录制...> ";
            }
            
            std::string input;
            std::getline(std::cin, input);

            // 切换录制状态
            is_recording = !is_recording;

            if (is_recording) {
                // --- 开始录制 ---

                // 发布机械臂失能消息
                pub_msg.task_status = 1;
                pub_msg.mode  = 1;
                pub_msg.pos_1 = 0; pub_msg.pos_2 = 0; pub_msg.pos_3 = 0; pub_msg.pos_4 = 0; pub_msg.pos_5 = 0; pub_msg.pos_6 = 0;
                pub_msg.vel_1 = 0; pub_msg.vel_2 = 0; pub_msg.vel_3 = 0; pub_msg.vel_4 = 0; pub_msg.vel_5 = 0; pub_msg.vel_6 = 0;
                pub_msg.tor_1 = 0; pub_msg.tor_2 = 0; pub_msg.tor_3 = 0; pub_msg.tor_4 = 0; pub_msg.tor_5 = 0; pub_msg.tor_6 = 0;
                pub_msg.kp = 0; pub_msg.kd = 0;

                pub->publish(pub_msg); // ROS 2 差异：pub.publish() → pub->publish()
                RCLCPP_INFO(this->get_logger(), "\n机械臂已失能，可以手动拖动机械臂进行示教。");

                // 获取包路径并拼接文件夹路径
                // ROS 2 差异：ros::package::getPath("hardware") 返回源码包目录，ROS 2 用
                //            ament_index_cpp::get_package_share_directory("hardware") 返回安装后的
                //            share 目录（<install>/hardware/share/hardware），
                //            teach_path 由 CMakeLists.txt 安装到该目录下。
                std::string package_path = ament_index_cpp::get_package_share_directory("hardware");
                std::string folder_path = package_path + "/teach_path/";

                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
                std::string filename = ss.str() + ".txt";
                std::string full_path = folder_path + filename;

                data_file.open(full_path);
                if (!data_file.is_open()) {
                    RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", full_path.c_str());
                    is_recording = false; // 无法打开文件，重置状态
                    continue;
                }
                
                data_file << "# " << ss.str() << std::endl;
                data_file << "# timestamp pos1 pos2 pos3 pos4 pos5 pos6 vel1 vel2 vel3 vel4 vel5 vel6" << std::endl;
                
                // 重置状态
                // ROS 2 差异：ros::Time(0) → rclcpp::Time(0, 0, 节点时钟类型)，
                //            显式指定时钟类型以避免与 this->now() 相减时时钟不一致。
                last_msg_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type()); // 重置上次消息时间
                initial_data_error_logged = false;
                data_update_warning_logged = false;
                record_start_time = this->now();
                record_timer->reset(); // 启动定时器（ROS 2 差异：ros::Timer::start() → WallTimer::reset()）
                RCLCPP_INFO(this->get_logger(), "开始录制数据到文件: %s", full_path.c_str());

            } else {
                // --- 停止录制 ---
                record_timer->cancel(); // 停止定时器（ROS 2 差异：ros::Timer::stop() → WallTimer::cancel()）
                if (data_file.is_open()) {
                    data_file.close();
                    RCLCPP_INFO(this->get_logger(), "录制结束。");

                    // 发布机械臂使能消息，机械臂恢复到初始位置
                    pub_msg.task_status = 1;
                    pub_msg.mode  = 2;
                    pub_msg.pos_1 = 0  ; pub_msg.pos_2 = 0  ; pub_msg.pos_3 = 0  ; pub_msg.pos_4 = 0  ; pub_msg.pos_5 = 0  ; pub_msg.pos_6 = 0;
                    pub_msg.vel_1 = 0.5; pub_msg.vel_2 = 0.5; pub_msg.vel_3 = 0.5; pub_msg.vel_4 = 0.5; pub_msg.vel_5 = 0.5; pub_msg.vel_6 = 0.5;

                    pub->publish(pub_msg); // ROS 2 差异：pub.publish() → pub->publish()
                }

            }
        }
        // 确保程序退出时文件已关闭
        if (data_file.is_open()) {
            data_file.close();
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    // ROS 2 差异：ros::init(argc, argv, "teach_node") → rclcpp::init(argc, argv)，
    //            节点名在 TestNode 的构造函数中以 Node("teach_node") 指定。
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();

    // ROS 2 差异：原工程用 std::thread(ros::spin()) 提供并发回调；ROS 2 用
    //            MultiThreadedExecutor + std::thread（等价于 ros::AsyncSpinner 模式）。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);

    // 创建一个新线程专门用于处理ROS消息回调
    std::thread ros_thread([&executor](){
        executor.spin();
    });

    node->run();

    rclcpp::shutdown(); // ROS 2 差异：关闭 rclcpp，使上面的 executor.spin() 退出

    // 等待ROS线程结束
    if (ros_thread.joinable()) ros_thread.join();

    return 0;
}
