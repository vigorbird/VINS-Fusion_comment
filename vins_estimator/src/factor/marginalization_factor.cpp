/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "marginalization_factor.h"

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

    if (loss_function)
    {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    //ROS_WARN("release marginlizationinfo");
    
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete it->second;

    for (int i = 0; i < (int)factors.size(); i++)
    {

        delete[] factors[i]->raw_jacobians;
        
        delete factors[i]->cost_function;

        delete factors[i];
    }
}

//主要是更新了三个变量factors，parameter_block_size和parameter_block_idx
//更新了优化变量的维度，和 当前帧要被边缘化的参数
void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);

    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;//参数快
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();//应该是参数块对应的大小

    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)//遍历参数块
    {
        double *addr = parameter_blocks[i];//参数的地址
        int size = parameter_block_sizes[i];//这个参数的大小
	    //reinterpret_cast可用作所有指针(引用)之间的转换
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)//遍历要被边缘化的参数
    {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];//得到要被边缘化的参数在参数块中的地址
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}
/*
actors是在addResidualBlockInfo()函数中添加的残差的容器factors.emplace_back(residual_block_info)
此时取出保存的残差Evaluate()计算所有状态变量构成的残差和雅克比矩阵
addr拿到优化变量的地址 开辟新的一块内存空间进行数据关联，主要是对parameter_block_data参数进行了更新并计算了每个cost funciton的雅克比和残差
*/
void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors)//遍历所有的cost funciton
    {
        it->Evaluate();//计算所有状态变量构成的残差和雅克比矩阵

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();//这个cost function 每个变量的维度
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)//遍历这个cost fucntion 的所有优化状态
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);//得到这个优化变量的地址
            int size = block_sizes[i];//得到这个优化变量的维度
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double *data = new double[size];
		  		//将 it->parameter_blocks[i]中的内容复制到data中
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
				//parameter_block_data = 序号是新开辟的参数空间地址的唯一表示符，内容是参数地址
				//整个代码中就这里对parameter_block_data进行了更新
                parameter_block_data[addr] = data;
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

//主要是进行了对A矩阵和b向量的赋值
//详见算法实现文档
void* ThreadsConstructA(void* threadsstruct)
{
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    for (auto it : p->sub_factors)//遍历这个线程的cost function
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)//遍历这个cost function 的参数块所有参数
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];//内容应该是当前帧新建空间的参数开始的序号
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];//这个参数的维度
            if (size_i == 7)
                size_i = 6;
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);//得到这个costfunction 对这个变量的雅科比
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)//再遍历这个cost function从第i个开始的其他参数
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];//内容应该是当前帧新建空间的参数开始的序号
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];//这个参数的维度
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);//得到这个costfunction 对这个变量的雅科比
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

//边缘化操作函数,舒尔布边缘化
//主要是更新linearized_jacobians和linearized_residuals
void MarginalizationInfo::marginalize()
{
    int pos = 0;
    for (auto &it : parameter_block_idx)//parameter_block_idx = 参数的地址形成的唯一标识符，如果这个参数是要被边缘化的则内容等于0
    {
        it.second = pos;
        pos += localSize(parameter_block_size[it.first]);//如果维度为7 则赋值为6
    }

    m = pos;//m = 当前帧边缘化参与优化的参数的维度，不计算当前帧被边缘化的变量的维度

    for (const auto &it : parameter_block_size)//parameter_block_size = 序号是参数地址形成的唯一标识符，内容是这个参数的维度
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())//表示新产生的优化变量
        {
            parameter_block_idx[it.first] = pos;
            pos += localSize(it.second);
        }
    }

    n = pos - m;
    //ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());
    if(m == 0)
    {
        valid = false;//整个代码就这里赋值为false
        printf("unstable tracking...\n");
        return;
    }

    TicToc t_summing;
    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();
    /*
    for (auto it : factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    ROS_INFO("summing up costs %f ms", t_summing.toc());
    */
    //multi thread

    //这里作者使用多线程进行赋值--非常值得借鉴!!!!!!
    TicToc t_thread_summing;
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];//NUM_THREADS = 4
    int i = 0;
    for (auto it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }
    for (int i = 0; i < NUM_THREADS; i++)
    {
        TicToc zero_matrix;
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = parameter_block_size;//parameter_block_size = 序号是参数地址形成的唯一标识符，内容是这个参数的维度
        threadsstruct[i].parameter_block_idx = parameter_block_idx;//parameter_block_idx = 序号是参数的地址形成的唯一标识符，如果这个参数是要被边缘化的则内容等于0，内容应该是当前帧新建空间的参数开始的序号
        //&tids[i] = 指向线程标识符的指针
        //ThreadsConstructA=线程主要的执行程序
        //(void*)&(threadsstruct[i]) = 运行函数的参数
        //主要是进行了对A矩阵和b向量的赋值
        int ret = pthread_create( &tids[i], NULL, ThreadsConstructA ,(void*)&(threadsstruct[i]));//非常重要的函数!!!!!!!!!!
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();
        }
    }
    for( int i = NUM_THREADS - 1; i >= 0; i--)  
    {
        pthread_join( tids[i], NULL ); //用于等待线程结束
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }
    //ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
    //ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());


    //TODO
    //赋值结束之后开始进行计算
    //Amm对应要被边缘化的状态
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());//m=当前帧边缘化参与优化的参数的维度，不计算当前帧被边缘化的变量的维度
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);//用于求特征向量

    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());

    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd( (saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0) ).asDiagonal() * saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr;//边缘化之后的A矩阵
    b = brr - Arm * Amm_inv * bmm;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));//如果这个特征值不大于eps则赋值为0
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();//sqrt(S_sqrt)
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();//asDiagonal = view a vector as a diagonal matrix 
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    //std::cout << A << std::endl
    //          << std::endl;
    //std::cout << linearized_jacobians << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

//更新边缘化之后保留的状态
//输入的变量 序号是位姿和外参地址的唯一id，内容是位姿地址和外参地址，这个结构中不包括要被边缘化的帧的位姿
std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx)//parameter_block_idx 序号是参数的地址形成的唯一标识符，如果这个参数是要被边缘化的则内容等于0，内容应该是当前帧新建空间的参数开始的序号
    {
        if (it.second >= m)//如果不是要被边缘化的状态
        {
            keep_block_size.push_back(parameter_block_size[it.first]);
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
	//accumulate = 用来计算特定范围内（包括连续的部分和初始值）所有元素的和
	//得到的是边缘之后留下的所有状态的维度总和
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}

//设置ceres中的被边缘化的参数块维度和残差的总体维度
MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info):marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)
    {
        mutable_parameter_block_sizes()->push_back(it);//mutable_parameter_block_sizes 是ceres中定义的变量 即参数块维度
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n);//set_num_residuals 是ceres中定义的变量 即被边缘化的残差总体维度
};

//边缘化的cost function 计算残差和雅克比
bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    //for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
    //{
    //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    //    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    //printf("residual %x\n", reinterpret_cast<long>(residuals));
    //}
    int n = marginalization_info->n;//没有被边缘化的状态总维度
    int m = marginalization_info->m;//被边缘化的状态总维度
    Eigen::VectorXd dx(n);//没有被边缘化的当前帧相对于这一帧的状态变化
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)//遍历上一帧的边缘化之后保留的参数块
    {
        int size = marginalization_info->keep_block_size[i];//参数块的维度
        int idx = marginalization_info->keep_block_idx[i] - m;//在上一帧中的状态序号
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);//这一帧优化之后的状态
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);//上一帧的状态
        if (size != 7)
            dx.segment(idx, size) = x - x0;//当前帧相对于这一帧的状态变化
        else//这个参数是位姿
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();//位置的变化量
		//作者这里定义的positify就是什么都没做，直接返回的四元数
		//vec 函数 = expression of the imaginary part (x,y,z)
		//2*(q0.inverse*q)_xyz
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify( Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5)) ).vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }
	
    //详见okvis文档说明，新的残差
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;

   //主要是更新雅克比
	if (jacobians)
    {

        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)//遍历上一帧的边缘化之后保留的参数块
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;//在上一帧中的状态序号
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    return true;
}
