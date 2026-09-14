/* ====================================================================
 * linear_planner.cpp
 * 直线轨迹规划封装：从起点匀速推进到终点，同时按比例插值姿态
 * ==================================================================== */

#include "linear_planner.h"

#include <cmath>        // sqrt
#include <algorithm>    // std::min

/* ------------------------------------------------------------------
 * setTarget：开始一段新的直线运动
 * 入参：
 *   start : 起点位姿（R,P,Y,x,y,z）
 *   goal  : 终点位姿
 *   speed : 运动速度（单位 m/s）
 * ------------------------------------------------------------------ */
void LinearPlanner::setTarget(const Pose6& start,
                              const Pose6& goal,
                              double speed)
{
    // 记录起点和终点
    start_ = start;
    goal_  = goal;

    // 当前位置从起点开始
    current_ = start;

    // 保存速度
    target_speed_ = speed;

    // 标记"正在执行"
    executing_ = true;
}

/* ------------------------------------------------------------------
 * update：每帧调用一次，把当前位置往前推进一小步
 * 入参：
 *   dt  : 两帧之间的时间（秒），把速度换算成单帧步长
 *   out : 输出，这一帧应该到达的位姿
 * 返回：
 *   true  = 已经到达终点（这段运动结束）
 *   false = 还在运动中
 * ------------------------------------------------------------------ */
bool LinearPlanner::update(double dt, Pose6& out)
{
    // 情况一：没有在执行，直接返回当前位姿
    if (!executing_)
    {
        out = current_;
        return true;
    }

    // 第 1 步：这一帧最多能走多远 = 速度 × 时间
    double step = target_speed_ * dt;

    // 第 2 步：当前位置到终点的向量，以及距离
    double dx = goal_.x - current_.x;
    double dy = goal_.y - current_.y;
    double dz = goal_.z - current_.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 情况二：已经非常接近终点（小于一步的 60%），直接吸附到终点
    if (dist < step * 0.6)
    {
        // 跳到终点
        current_ = goal_;

        // 运动结束
        executing_ = false;

        // 输出终点位姿
        out = current_;

        // 告诉调用者：完成了
        return true;
    }
    else
    {
        // 情况三：还没到终点，沿直线方向走一步
        current_.x += dx / dist * step;
        current_.y += dy / dist * step;
        current_.z += dz / dist * step;

        // 姿态插值：按"走过的距离 / 总距离"的比例来线性插值 R/P/Y
        // 这样位置和姿态同步推进，不会出现"先到位、后转姿态"
        double total_dx = goal_.x - start_.x;
        double total_dy = goal_.y - start_.y;
        double total_dz = goal_.z - start_.z;
        double total_dist = std::sqrt(total_dx * total_dx +
                                      total_dy * total_dy +
                                      total_dz * total_dz);

        double moved_dx = current_.x - start_.x;
        double moved_dy = current_.y - start_.y;
        double moved_dz = current_.z - start_.z;
        double moved = std::sqrt(moved_dx * moved_dx +
                                 moved_dy * moved_dy +
                                 moved_dz * moved_dz);

        // 总距离为 0（起点=终点）时比例取 1；否则取"走过的/总的"，且不超过 1
        double ratio = 1.0;
        if (total_dist > 1e-9)
        {
            ratio = std::min(1.0, moved / total_dist);
        }

        // 线性插值三个姿态角
        current_.R = start_.R + (goal_.R - start_.R) * ratio;
        current_.P = start_.P + (goal_.P - start_.P) * ratio;
        current_.Y = start_.Y + (goal_.Y - start_.Y) * ratio;

        // 输出这一帧的位姿
        out = current_;

        // 还在运动中
        return false;
    }
}

/* ------------------------------------------------------------------
 * isExecuting：是否正在执行一段直线运动
 * ------------------------------------------------------------------ */
bool LinearPlanner::isExecuting() const
{
    return executing_;
}