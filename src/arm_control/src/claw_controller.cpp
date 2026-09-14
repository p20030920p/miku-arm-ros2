/* ====================================================================
 * claw_controller.cpp
 * 夹爪控制封装：命令式接口 + 4 态状态机实现
 * 参数来自 setParams()（yaml），不设则用 ClawParams 默认值
 * 方向约定：pos_7 正值=张开，负值=闭合；力矩 正=张开，负=闭合
 * ==================================================================== */

#include "claw_controller.h"

#include <cmath>    // fabs
#include <cstdio>   // printf

namespace {
// ROS 2 差异：本机 GCC 13 的 libstdc++ 只在 C++20 下提供 <numbers>::pi，
// 而本工作空间统一用 C++17，因此这里直接定义一个 π 常量。
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;   // 度 → 弧度
}   // namespace

/* ------------------------------------------------------------------
 * setParams：从外部（yaml）设置夹爪参数
 * ------------------------------------------------------------------ */
void ClawController::setParams(const ClawParams& p)
{
    params_ = p;
}

/* ------------------------------------------------------------------
 * open：张开（松开物体）。任何状态调用都回到张开
 * ------------------------------------------------------------------ */
void ClawController::open()
{
    claw_state_    = CLAW_OPEN;
    catch_counter_ = 0;
    catch_pos_     = 0.0;
}

/* ------------------------------------------------------------------
 * close：开始夹取。已在夹紧/保持中 → 忽略（要先 open 松开再夹）
 * ------------------------------------------------------------------ */
void ClawController::close()
{
    if (claw_state_ == CLAW_CLOSING || claw_state_ == CLAW_HOLD)
        return;

    claw_state_    = CLAW_CLOSING;
    catch_counter_ = 0;
    catch_pos_     = 0.0;
}

/* ------------------------------------------------------------------
 * isHolding / isFailed：查询夹取结果
 * ------------------------------------------------------------------ */
bool ClawController::isHolding() const
{
    return claw_state_ == CLAW_HOLD;
}

bool ClawController::isFailed() const
{
    return claw_state_ == CLAW_FAILED;
}

/* ------------------------------------------------------------------
 * stateName：当前状态的中文名（供打印）
 * ------------------------------------------------------------------ */
const char* ClawController::stateName() const
{
    switch (claw_state_) {
        case CLAW_OPEN:    return "张开";
        case CLAW_CLOSING: return "夹紧中";
        case CLAW_HOLD:    return "已夹到";
        case CLAW_FAILED:  return "未夹到";
    }
    return "未知";
}

/* ------------------------------------------------------------------
 * update：每帧推进状态机，算出该下发的 pos/vel/tor
 * ------------------------------------------------------------------ */
void ClawController::update(double pos_now, double tor_now,
                            double& cmd_pos, double& cmd_vel, double& cmd_tor)
{
    cmd_vel = 0.0;   // 夹爪不做速度控制

    // 取参数（度数 → 弧度）
    const double open_pos_rad   = params_.open_pos_deg   * kDeg2Rad;
    const double open_step_rad  = params_.open_step_deg  * kDeg2Rad;
    const double close_pos_rad  = params_.close_pos_deg  * kDeg2Rad;
    const double close_step_rad = params_.close_step_deg * kDeg2Rad;
    const double hold_step_rad  = params_.hold_step_deg  * kDeg2Rad;

    switch (claw_state_) {
        case CLAW_OPEN:
        {
            // 张开：步进到张开位，到位后停力矩（防顶限位）
            cmd_pos = pos_now + open_step_rad;
            if (cmd_pos > open_pos_rad) cmd_pos = open_pos_rad;

            if (pos_now > open_pos_rad - 0.02)
                cmd_tor = 0.0;              // 已到位 → 停止出力
            else
                cmd_tor = params_.open_torque;   // 未到位 → 张开驱动力矩
            break;
        }

        case CLAW_CLOSING:
        {
            // 夹紧：步进到闭合位
            cmd_pos = pos_now - close_step_rad;
            if (cmd_pos < close_pos_rad) cmd_pos = close_pos_rad;
            cmd_tor = params_.close_torque;

            // 判定 0（优先）：已到下限 → 空载失败
            if (pos_now <= close_pos_rad + 0.0087)
            {
                claw_state_ = CLAW_FAILED;
                printf("[夹爪] 未夹取到物体（已到下限）\n");
            }
            else
            {
                // 判定 1：力矩连续超阈值 5 帧 → 才算真夹到（防卡顿误判）
                if (fabs(tor_now) > params_.catch_torque)
                    catch_counter_++;
                else
                    catch_counter_ = 0;

                if (catch_counter_ >= 5)
                {
                    claw_state_ = CLAW_HOLD;
                    catch_pos_  = pos_now;
                    printf("[夹爪] 夹取成功 @ %.2f°\n", catch_pos_ / kDeg2Rad);
                }
            }
            break;
        }

        case CLAW_HOLD:
        {
            // 预压夹紧：目标 = 接触位置再闭合 hold_step，kp 提供持续夹持力
            double hold_pos = catch_pos_ - hold_step_rad;
            if (hold_pos < close_pos_rad) hold_pos = close_pos_rad;

            cmd_pos = hold_pos;             // 固定目标（不跟随回读）
            cmd_tor = params_.hold_torque;  // 保持力矩辅助
            break;
        }

        case CLAW_FAILED:
        {
            // 位置截止：锁在当前回读位置，不施加力矩
            cmd_pos = pos_now;
            cmd_tor = 0.0;
            break;
        }
    }
}