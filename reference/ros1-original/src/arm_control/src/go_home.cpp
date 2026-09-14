/* ====================================================================
 * go_home.cpp
 * 安全归零程序：所有 bash 脚本在 Ctrl+C 退出时调用
 *
 * 作用：程序完全退出前，以"速度位置模式"(mode=2) 把所有电机 1~7
 *       低速复位到 0 位置，方便下次调试，实现安全退出。
 *
 * 调用前提（由 bash 的 cleanup 保证）：
 *   1. 已杀掉所有其他算法节点（避免抢 Arm_tx 发布权）
 *   2. hardware 节点仍在运行（它是 Arm_tx → 串口的唯一通道）
 *   3. roscore 仍在运行
 *
 * 退出条件：
 *   - 所有关节回读位置都接近 0（< 0.01 rad）
 *   - 或超时（默认 20 秒，防止 hardware 断连时卡死）
 * ==================================================================== */

#include <ros/ros.h>
#include <arm_control/ArmMsg.h>
#include <cmath>    // fabs

// 回读缓存（全局，供回调写、主循环读；spinOnce 同一线程，无竞争）
static double pos_now[7] = {0, 0, 0, 0, 0, 0, 0};
static bool   got_feedback = false;

// 订阅回调：把回读的 7 个电机位置存下来
void feedbackCallback(const arm_control::ArmMsg::ConstPtr& m)
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
    ros::init(argc, argv, "go_home");
    ros::NodeHandle nh;

    /* ===== 参数（要改速度/超时改这里）===== */
    const double home_speed   = 0.3;    // 归零速度 rad/s（低速）
    const double kp           = 6.0;    // 位置刚度
    const double kd           = 0.6;    // 阻尼
    const double pos_tol      = 0.01;   // 到位判定：|位置| < 0.01 rad
    const double max_duration = 20.0;   // 超时（秒）

    // 发布 Arm_tx（下发给 hardware），订阅 Arm_rx（回读判断到位）
    ros::Publisher  pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 10);
    ros::Subscriber sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 10,
                                                            feedbackCallback);

    ros::Time start = ros::Time::now();
    ros::Rate rate(50);    // 50Hz 持续发布

    ROS_INFO("[go_home] 开始低速归零，速度 %.2f rad/s，超时 %.0fs",
             home_speed, max_duration);

    while (ros::ok())
    {
        // 1. 组装"速度位置模式"回零指令（所有电机目标 = 0）
        arm_control::ArmMsg msg;
        msg.Task_status = 1;
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

        pub.publish(msg);

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
            ROS_INFO("[go_home] 全部关节已归零，退出");
            break;
        }

        // 3. 超时保护（hardware 不在 / 串口断连时不卡死）
        if ((ros::Time::now() - start).toSec() > max_duration)
        {
            ROS_WARN("[go_home] 超时(%.0fs)未完全归零，强制退出", max_duration);
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[go_home] 归零程序结束");
    return 0;
}