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
    std::vector<double *> parameter_blocks;//�������Ż������ĵ�ַ
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

    std::vector<ResidualBlockInfo *> factors;//��Ե�������вв��
    int m, n;//n= ����ӵ�״̬ά�ȣ�Ӧ����û�б���Ե����״̬ά�ȡ�m=��ǰ֡��Ե�������Ż��Ĳ�����ά�ȣ������㵱ǰ֡����Ե���ı�����ά�ȣ�Ӧ����Ҫ����Ե����״̬!!!!
    std::unordered_map<long, int> parameter_block_size; //global size ����ǲ�����ַ�γɵ�Ψһ��ʶ�������������������ά��
    int sum_block_size;//=��Ե֮�����µ�����״̬��ά���ܺ�
	//local size ����ǲ����ĵ�ַ�γɵ�Ψһ��ʶ����������������Ҫ����Ե���������ݵ���0������Ӧ���ǵ�ǰ֡�½��ռ�Ĳ�����ʼ�����
	//��Ҫע��������Ż�����λ������Ϊά����6�����marginalize����
    std::unordered_map<long, int> parameter_block_idx; 
    std::unordered_map<long, double *> parameter_block_data;//������¿��ٵĲ����ռ��ַ��Ψһ��ʾ���������ǲ�����ַ

    std::vector<int> keep_block_size; //global size ��Ե��֮�����µĸ���״̬��ά��
    std::vector<int> keep_block_idx;  //local size  ��Ե��֮�����µĸ���״̬���½��Ŀռ��е����
    std::vector<double *> keep_block_data; //��Ե��֮�����µĸ���״̬�ĵ�ַ

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
