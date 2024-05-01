/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

//在滑动窗中最后一个观测到这个点的序号
int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}

/*VINS里为了控制优化计算量，在实时情况下，只对当前帧之前某一部分帧进行优化，而不是全部历史帧。局部优化帧的数量就是窗口大小。
为了维持窗口大小，需要去除旧的帧添加新的帧，也就是边缘化 Marginalization。到底是删去最旧的帧（MARGIN_OLD）还是删去刚刚进来窗口倒数第二帧(MARGIN_SECOND_NEW)，
就需要对当前帧与之前帧进行视差比较，如果是当前帧变化很小，就会删去倒数第二帧，如果变化很大，就删去最旧的帧。
VINS 里把特征点管理和检查视差放在了同一个函数里，先添加特征点，再进行视差检查。
*/
//frame_count = 初始化使用的帧数，最大值就是9
//输入的image中的第一个int表示特征点的ID，第二个int表示是左相机还是右相机，Eigen::Matrix<double, 7, 1>表示当前帧的归一化平面坐标，图像坐标，和相对于上帧图像的像素速度
//返回true表示删除老的帧
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)//遍历当前帧的所有特征点
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);//id_pts.second[0].second=左图像的特征点
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)//表示有右相机
        {
            f_per_fra.rightObservation(id_pts.second[1].second);//id_pts.second[1].second=右图像的特征点
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;//特征点的id
        //从之前的feauter中寻找当前帧的特征点，
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
        {
            return it.feature_id == feature_id;
        });

        if (it == feature.end())//如果找不到表示这是新的特征点，则更新feature特征点的信息
        {   //整个代码中就这里更新了feature的信息
      	    //frame_count = 在滑动窗口中第一个观测到这个特征点的帧的序号
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)//如果找到了
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;//增加当前帧和地图点匹配的个数
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    
	//滑动窗口中帧的数量小于2 跟踪到角点数目小于20 连续被跟踪小于40帧 当前帧加入角点记录列表的数目大于跟踪到角点数目的一半(当前帧出现新的视野较多)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)//计算滑动窗口中中各个特征点的事差
    {
        //这个特征点在滑动窗口中第一次被看到一定要小于最新帧-2 且 从被开始看到滑动窗结束要一直被观测到
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);//视差
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;//这个变量没用
        return parallax_sum / parallax_num >= MIN_PARALLAX;//默认设置是10 如果平均视差大于10则认为我们要删除最老的帧
    }
}

//输入的是滑动窗中的帧的范围
//遍历滑动窗所有特征点，获得起始和结束帧上的像素坐标
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

//更新特征点中的深度
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

//删除那些优化失败的特征点
void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

//返回被观测到超过四次的特征点的逆深度
VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());//返回的是观测次数超过4次的特征点
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

//推导详见 https://blog.csdn.net/u012101603/article/details/79714332，或者算法实现文档
//输出的参数是point_3d
void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

//frameCnt = 滑动窗口中关键帧的数量
//Rs ts 是滑动窗口中 世界坐标系移动到imu坐标系的位姿变化
void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;//在这个滑动窗口中这个特征点从被观测到最后一个关键帧的 帧数
                //这个特征点在滑动窗口中被观测到的次数要大于等于上面计算到的帧数+1
                //即表示这个特征点在滑动窗中被连续的观测到，直到当前帧
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];//将图像坐标系下的点变换到imu做坐标系下
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];//将imu坐标系下的点变换到世界坐标系下

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//这个特征点在当前帧的观测值
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d);
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam 
        //世界坐标系移动到滑动窗中倒数第二帧相机坐标系的位姿变化
        //这里非常好理解，我们要求滑动窗最后一个也就是当前帧的位姿变化，那么最好的初值肯定使用倒数第二个帧在世界坐标系下的位姿初始化了
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))//计算的结果是RCam和Pcam
        {
            // trans to w_T_imu
            //根据pnp的结果反推得到世界坐标系变换到当前帧的imu坐标系的位姿变化
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}


//frameCnt = 滑动窗口内的第几帧 好像没有用到!!!!!!
//这个函数目的是对于那些还没有深度但是已经在滑动窗被观测到4次以上的特征点，更新其在起始帧下的深度
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;

        if(STEREO && it_per_id.feature_per_frame[0].is_stereo)//在滑动窗口中第一个看到这个特征点的时刻 左右相机都看到了
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;//第i帧左相机坐标系移动到世界坐标系的位姿变换
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];//世界坐标系移动到第i帧左相机坐标系的位姿变换
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;//第i帧右相机坐标系移动到世界坐标系的位姿变换
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);//左相机观测到的像素坐标
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);//右相机观测到的像素坐标
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

			//详见算法实现文档
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);//point3d是结果，世界坐标系下的点
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();//将世界坐标下的点变换到i帧左相机坐标系下
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1)//这个特征点在当前帧只被一个相机看到了
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;//第i帧左相机坐标系移动到世界坐标系的位姿变换
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;//这里是惟一的区别
            Eigen::Matrix<double, 3, 4> rightPose;//第j帧左相机坐标系移动到世界坐标系的位姿变换
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();//如果这特征点少于被4个帧看到则不进行后续的更新
        if (it_per_id.used_num < 4)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];//世界坐标系到第i帧的imu坐标系的变化
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)//遍历看到这个特征点的所有帧
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;//第i帧坐标系移动到第j帧坐标系的
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);//详见算法实现文档triangulatePoint函数
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;//计算得到的是在第i帧坐标系下的深度
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }//遍历所有特征点结束
}

//删除特征点中重投影误差比较大的点
void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

//输入的参数 = 删除之前滑动窗的第一个帧的位姿，删除之前第二个帧的位姿
//如果这个特征点的滑动起始帧不是第一个帧，更新滑动起始帧的序号，
//如果这个特征点的滑动起始帧是第一个帧，则需要让这个特征点不被第一个帧观测到，并且将这个特征点的深度变换到第二个特征帧坐标系下
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    //遍历滑动窗所有的特征点
    for (auto it = feature.begin(), it_next = feature.begin();it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;//更新这个特征点的滑动起始帧的序号
        else//这个特征点的滑动起始帧是第一个帧，因为第一个帧要删除了 所以要更新这个特征点的信息
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());//删除看到这个特征点的第一个帧
            if (it->feature_per_frame.size() < 2)//看到这个特征点的征小于2 则删除
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;//根据第一帧的位姿得到的这个特征点在世界坐标系下的坐标
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);//变换到第二个帧坐标系下
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

//如果这个特征点的滑动起始帧不是第一个帧，更新滑动起始帧的序号，
//如果这个特征点的滑动起始帧是第一个帧，则需要让这个特征点不被第一个帧观测到
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;//滑动起始帧等于第10帧
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;//
            if (it->endFrame() < frame_count - 1)//如果最后一个观测到这个特征点的序号等于第10个帧，直接跳过，不受到影响
                continue;
			//如果最后一个观测到这个特征点的序号等于第10个帧
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);//特征点不让第10帧观测到
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

//详见算法实现文档
//it_per_id = 滑动窗口中的特征点
//frame_count = 滑动窗口中所有帧的个数
//du=xi/zi-xj, dv = xi/zi-yj,视差 = sqrt(du*du+dv*dv)
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];//倒数第二个看到这个特征点的帧
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];//倒数第一个看到这个特征点的帧

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}
