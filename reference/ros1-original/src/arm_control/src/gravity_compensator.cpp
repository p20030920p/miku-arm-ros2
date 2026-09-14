/* ====================================================================
 * gravity_compensator.cpp
 * 重力补偿封装：根据当前关节角算重力矩，再乘上标定系数
 * ==================================================================== */

#include "gravity_compensator.h"

#include <ros/ros.h>   // 用于 ROS_WARN 打印

/* ------------------------------------------------------------------
 * init：用 KDL 链创建重力（动力学）求解器
 * 入参：
 *   chain : KDL 链（从 KinematicsSolver::chain() 拿到）
 * 返回：
 *   true  = 成功；false = 失败
 * ------------------------------------------------------------------ */
bool GravityCompensator::init(const KDL::Chain& chain)
{
    if (dyn_solver != nullptr)   // 已建过 → 先释放，防止重复 init 泄漏
    {
        delete dyn_solver;
        dyn_solver = nullptr;
    }
    
    // 创建动力学求解器，第二个参数是重力加速度方向（Z 轴向下）
    dyn_solver = new KDL::ChainDynParam(chain, KDL::Vector(0.0, 0.0, -9.81));

    // 检查是否创建成功
    if (dyn_solver == nullptr)
    {
        ROS_ERROR("重力求解器创建失败");
        return false;
    }

    // 为 tau_g 分配和关节数一样大的空间
    tau_g = KDL::JntArray(chain.getNrOfJoints());

    return true;
}

/* ------------------------------------------------------------------
 * setGains：设置每个关节的补偿系数（之前手工标定的 0.5/0.55/...）
 * 入参：
 *   g : 长度 6 的系数数组
 * ------------------------------------------------------------------ */
void GravityCompensator::setGains(const std::vector<double>& g)
{
    // 校验长度：必须和关节数一致（之前缺的检查）
    if (g.size() != (size_t)tau_g.rows())
    {
        ROS_WARN("系数个数(%zu)与关节数(%d)不一致，忽略本次设置",
                 g.size(), tau_g.rows());
        return;
    }

    // 长度正确，拷贝进来
    gains = g;
}

/* ------------------------------------------------------------------
 * update：根据当前关节角 q 计算重力矩，保存到成员 tau_g
 * 入参：
 *   q : 当前回读的关节角
 * ------------------------------------------------------------------ */
void GravityCompensator::update(const KDL::JntArray& q)
{
    // 求解器没建好，直接跳过
    if (dyn_solver == nullptr)
    {
        return;
    }

    // 调用 KDL 计算重力矩
    int ret = dyn_solver->JntToGravity(q, tau_g);
    if (ret < 0)
    {
        ROS_WARN("重力矩计算失败（可能 URDF 惯量不完整）");
    }
}

/* ------------------------------------------------------------------
 * compensatedTorques：返回"补偿后的力矩" = tau_g × gains
 * 返回的就是要写进 msg_out.tor_1..tor_6 的那 6 个数
 * ------------------------------------------------------------------ */
std::vector<double> GravityCompensator::compensatedTorques() const
{
    // 建一个和关节数一样长的输出数组
    std::vector<double> out(tau_g.rows());

    // 逐个关节：补偿力矩 = 重力矩 × 系数
    for (unsigned int i = 0; i < out.size(); ++i)
    {
        out[i] = tau_g(i) * gains[i];
    }

    return out;
}

/* ------------------------------------------------------------------
 * 析构函数：释放动力学求解器（之前缺的部分）
 * ------------------------------------------------------------------ */
GravityCompensator::~GravityCompensator()
{
    delete dyn_solver;
}