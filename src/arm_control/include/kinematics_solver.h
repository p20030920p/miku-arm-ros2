// kinematics_solver.h
#pragma once
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

class KinematicsSolver {
public:
    bool loadURDF(const std::string& urdf_path,
                  const std::string& base_link,
                  const std::string& tip_link);
    // 逆解：输入种子+目标位姿，输出解。返回 true/false
    bool solveIK(const KDL::JntArray& seed, const KDL::Frame& target, KDL::JntArray& sol);
    // 正解：关节角→末端位姿
    KDL::Frame solveFK(const KDL::JntArray& q);
    unsigned int jointCount() const;
    const KDL::Chain& chain() const { return kdl_chain; }
    ~KinematicsSolver();     // 释放 ik_solver / fk_solver
private:
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    KDL::ChainIkSolverPos_LMA* ik_solver = nullptr;
    KDL::ChainFkSolverPos_recursive* fk_solver = nullptr;
};