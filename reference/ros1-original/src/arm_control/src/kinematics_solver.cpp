/* ====================================================================
 * kinematics_solver.cpp
 * 运动学封装：加载URDF、正运动学(FK)、逆运动学(IK)
 * ==================================================================== */

#include "kinematics_solver.h"

#include <ros/ros.h>          // 用于 ROS_ERROR 打印
#include <urdf/model.h>       // 读取 urdf 文件
#include <kdl_parser/kdl_parser.hpp>  // urdf -> KDL 树

/* ------------------------------------------------------------------
 * loadURDF：加载机器人模型，构建 KDL 链和两个求解器
 * 入参：
 *   urdf_path : URDF 文件完整路径
 *   base_link : 起始连杆名（一般 "base_link"）
 *   tip_link  : 末端连杆名（你的模型 "link_6"）
 * 返回：
 *   true  = 成功；false = 失败
 * ------------------------------------------------------------------ */
bool KinematicsSolver::loadURDF(const std::string& urdf_path,
                                const std::string& base_link,
                                const std::string& tip_link)
{
    // 第 1 步：用 urdf 库读取模型文件
    urdf::Model model;
    if (!model.initFile(urdf_path))
    {
        ROS_ERROR("无法读取 URDF 文件: %s", urdf_path.c_str());
        return false;
    }

    // 第 2 步：把 urdf 模型转成 KDL 的树结构
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("URDF 转 KDL 树失败");
        return false;
    }

    // 第 3 步：从树中取出 base_link → tip_link 这一段链
    //         （getChain 返回 0 表示成功，负数表示失败）
    if (kdl_tree.getChain(base_link, tip_link, kdl_chain) < 0)
    {
        ROS_ERROR("找不到链: %s → %s", base_link.c_str(), tip_link.c_str());
        return false;
    }

    // 第 4 步：创建逆运动学求解器（LMA 迭代法）
    ik_solver = new KDL::ChainIkSolverPos_LMA(kdl_chain);

    // 第 5 步：创建正运动学求解器（递归法）
    fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);

    // 全部成功
    return true;
}

/* ------------------------------------------------------------------
 * solveIK：逆运动学。给一个种子角度 + 目标末端位姿，求关节角
 * 入参：
 *   seed   : 起始猜测（一般用上一帧的解，收敛更快更平滑）
 *   target : 目标末端位姿（KDL::Frame）
 *   sol    : 输出，求出的关节角
 * 返回：
 *   true  = 求到解；false = 无解（目标超出可达范围等）
 * ------------------------------------------------------------------ */
bool KinematicsSolver::solveIK(const KDL::JntArray& seed,
                               const KDL::Frame& target,
                               KDL::JntArray& sol)
{
    // 求解器没建好，直接失败
    if (ik_solver == nullptr)
    {
        ROS_ERROR("IK 求解器未初始化");
        return false;
    }

    // 调用 KDL 逆解；返回 >=0 表示成功
    int result = ik_solver->CartToJnt(seed, target, sol);
    if (result < 0)
    {
        return false;
    }

    return true;
}

/* ------------------------------------------------------------------
 * solveFK：正运动学。给关节角，求末端位姿
 * 入参：
 *   q : 关节角
 * 返回：
 *   末端位姿 Frame（失败时返回单位矩阵兜底）
 * ------------------------------------------------------------------ */
KDL::Frame KinematicsSolver::solveFK(const KDL::JntArray& q)
{
    // 默认返回单位矩阵（原点、无旋转），失败时兜底用
    KDL::Frame result;

    // 求解器没建好，直接返回单位阵
    if (fk_solver == nullptr)
    {
        ROS_ERROR("FK 求解器未初始化");
        return result;
    }

    // 调用 KDL 正解；返回 >=0 表示成功
    int ret = fk_solver->JntToCart(q, result);
    if (ret < 0)
    {
        ROS_ERROR("正运动学计算失败");
    }

    return result;
}

/* ------------------------------------------------------------------
 * jointCount：返回链的关节数量（一般是 6）
 * ------------------------------------------------------------------ */
unsigned int KinematicsSolver::jointCount() const
{
    return kdl_chain.getNrOfJoints();
}

/* ------------------------------------------------------------------
 * 析构函数：释放两个求解器，防止内存泄漏（之前缺的部分）
 * ------------------------------------------------------------------ */
KinematicsSolver::~KinematicsSolver()
{
    delete ik_solver;
    delete fk_solver;
}