/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;

struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks;//内容是优化变量的地址
    std::vector<int> drop_set;

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};

class MarginalizationInfo
{
  public:
    MarginalizationInfo(){valid = true;};
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    std::vector<ResidualBlockInfo *> factors;//边缘化的所有残差函数
    int m, n;//n= 新添加的状态维度，应该是没有被边缘化的状态维度。m=当前帧边缘化参与优化的参数的维度，不计算当前帧被边缘化的变量的维度，应该是要被边缘化的状态!!!!
    std::unordered_map<long, int> parameter_block_size; //global size 序号是参数地址形成的唯一标识符，内容是这个参数的维度
    int sum_block_size;//=边缘之后留下的所有状态的维度总和
	//local size 序号是参数的地址形成的唯一标识符，如果这个参数是要被边缘化的则内容等于0，内容应该是当前帧新建空间的参数开始的序号
	//需要注意了如果优化的是位姿则认为维度是6，详见marginalize函数
    std::unordered_map<long, int> parameter_block_idx; 
    std::unordered_map<long, double *> parameter_block_data;//序号是新开辟的参数空间地址的唯一表示符，内容是参数地址

    std::vector<int> keep_block_size; //global size 边缘化之后保留下的各个状态的维度
    std::vector<int> keep_block_idx;  //local size  边缘化之后留下的各个状态在新建的空间中的序号
    std::vector<double *> keep_block_data; //边缘化之后留下的各个状态的地址

    Eigen::MatrixXd linearized_jacobians;//
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
    bool valid;

};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};
