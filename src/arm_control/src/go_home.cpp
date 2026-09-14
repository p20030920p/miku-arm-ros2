/* ====================================================================
 * go_home.cpp —— ROS 2 Jazzy 移植版
 * 安全归零程序：所有启动脚本在 Ctrl+C 退出时调用
 *
 * 作用：程序完全退出前，以"速度位置模式"(mode=2) 把所有电机 1~7
 *       低速复位到 0 位置，方便下次调试，实现安全退出。
 *
 * 调用前提（由启动脚本的 cleanup 保证）：
 *   1. 已杀掉所有其他算法节点（避免抢 Arm_tx 发布权）
 *   2. hardware 节点仍在运行（它是 Arm_tx → 串口的唯一通道）
 *
 * 退出条件：
 *   - 所有关节回读位置都接近 0（< 0.01 rad）
 *   - 或超时（默认 20 秒，防止 hardware 断连时卡死）
 *
 * ROS 2 差异：原 ros::spinOnce() 改为 rclcpp 单线程 executor 的
 *             spin_some()，在 50Hz 循环里主动泵一次回调。
 * ==================================================================== */

#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp>
#include <cmath>    // fabs
#include <memory>

// 回读缓存（全局，供回调写、主循环读；spin_some 同一线程，无竞争）
static double pos_now[7] = {0, 0, 0, 0, 0, 0, 0};
static bool   got_feedback = false;

// 订阅回调：把回读的 7 个电机位置存下来
void feedbackCallback(const arm_control::msg::ArmMsg::SharedPtr m)
{
    pos_now[0] = m->pos_1;  pos_now[1] = m->pos_2;
    pos_now[2] = m->pos_3;  pos_now[3] = m->pos_4;
    pos_now[4] = m->pos_5;  pos_now[5] = m->pos_6;
    pos_now[6] = m->pos_7;
    got_feedback = true;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("go_home");

    /* ===== 参数（要改速度/超时改这里）===== */
    const double home_speed   = 0.3;    // 归零速度 rad/s（低速）
    const double kp           = 6.0;    // 位置刚度
    const double kd           = 0.6;    // 阻尼
    const double pos_tol      = 0.01;   // 到位判定：|位置| < 0.01 rad
    const double max_duration = 20.0;   // 超时（秒）

    // 发布 Arm_tx（下发给 hardware），订阅 Arm_rx（回读判断到位）
    auto pub = node->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 10);
    auto sub = node->create_subscription<arm_control::msg::ArmMsg>(
        "Arm_rx", 10, feedbackCallback);

    // 单线程执行器：在循环里手动 spin_some()，等价于原来的 ros::spinOnce()
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Time start = node->now();
    rclcpp::WallRate rate(50);    // 50Hz 持续发布

    RCLCPP_INFO(node->get_logger(),
                "[go_home] 开始低速归零，速度 %.2f rad/s，超时 %.0fs",
                home_speed, max_duration);

    while (rclcpp::ok())
    {
        // 1. 组装"速度位置模式"回零指令（所有电机目标 = 0）
        arm_control::msg::ArmMsg msg;
        msg.task_status = 1;
        msg.mode        = 2;        // 2 = 速度位置模式
        msg.kp          = kp;
        msg.kd          = kd;

        msg.pos_1 = 0;  msg.pos_2 = 0;  msg.pos_3 = 0;
        msg.pos_4 = 0;  msg.pos_5 = 0;  msg.pos_6 = 0;
        msg.pos_7 = 0;                  // 夹爪也回 0（张开位，安全）

        msg.vel_1 = home_speed;  msg.vel_2 = home_speed;  msg.vel_3 = home_speed;
        msg.vel_4 = home_speed;  msg.vel_5 = home_speed;  msg.vel_6 = home_speed;
        msg.vel_7 = home_speed;

        msg.tor_1 = 0;  msg.tor_2 = 0;  msg.tor_3 = 0;
        msg.tor_4 = 0;  msg.tor_5 = 0;  msg.tor_6 = 0;
        msg.tor_7 = 0;

        pub->publish(msg);

        // 2. 判断是否全部到位
        bool all_home = true;
        for (int i = 0; i < 7; ++i)
        {
            if (fabs(pos_now[i]) > pos_tol)
            {
                all_home = false;
                break;
            }
        }
        if (got_feedback && all_home)
        {
            RCLCPP_INFO(node->get_logger(), "[go_home] 全部关节已归零，退出");
            break;
        }

        // 3. 超时保护（hardware 不在 / 串口断连时不卡死）
        if ((node->now() - start).seconds() > max_duration)
        {
            RCLCPP_WARN(node->get_logger(), "[go_home] 超时(%.0fs)未完全归零，强制退出", max_duration);
            break;
        }

        executor.spin_some();   // 等价于 ros::spinOnce()
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "[go_home] 归零程序结束");
    rclcpp::shutdown();
    return 0;
}
