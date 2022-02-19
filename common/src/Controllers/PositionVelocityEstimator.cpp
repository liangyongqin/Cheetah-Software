/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "Controllers/PositionVelocityEstimator.h"

/*!
 * Initialize the state estimator
 * 初始化状态估计器
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {
	
/*Eigen::Matrix<T, 18, 1> _xhat;//状态估计值 [p v p1 p2 p3 p4] 世界坐标下
  Eigen::Matrix<T, 12, 1> _ps;//储存状态p
  Eigen::Matrix<T, 12, 1> _vs;//储存状态v
  Eigen::Matrix<T, 18, 18> _A;//状态转移阵
  Eigen::Matrix<T, 18, 18> _Q0;//初始状态估计噪声
  Eigen::Matrix<T, 18, 18> _P;//状态不确定性
  Eigen::Matrix<T, 28, 28> _R0;//初始观测噪声
  Eigen::Matrix<T, 18, 3> _B;//输入阵
  Eigen::Matrix<T, 28, 18> _C;//观测阵*/
  T dt = this->_stateEstimatorData.parameters->controller_dt;//从参数配置文件加载dt
  _xhat.setZero();//状态估计值
  _ps.setZero();
  _vs.setZero();
  //状态转移矩阵，计算K+1时刻状态值X[k+1] 自己写出来算下就知道
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
  //输入矩阵
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  //观测矩阵
  //观测量[p1 p2 p3 p4 v1 v2 v3 v4 z1 z2 z3 z4]（v z都在世界坐标下p在机身坐标系） p v 是向量 z是标量 pib=pb-piw vi=vb
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = T(1);
  _C(26, 14) = T(1);
  _C(25, 11) = T(1);
  _C(24, 8) = T(1);
  
  //初始不确定性
  _P.setIdentity();
  _P = T(100) * _P;
  //初始状态估计噪声
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
	  
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  
  _R0.setIdentity();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run() {
  T process_noise_pimu =
      this->_stateEstimatorData.parameters->imu_process_noise_position;
  T process_noise_vimu =
      this->_stateEstimatorData.parameters->imu_process_noise_velocity;
  T process_noise_pfoot =
      this->_stateEstimatorData.parameters->foot_process_noise_position;
  T sensor_noise_pimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_position;
  T sensor_noise_vimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
  T sensor_noise_zfoot =
      this->_stateEstimatorData.parameters->foot_height_sensor_noise;
//状态估计噪声
  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;
//观测噪声矩阵
  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;
//重力向量
  Vec3<T> g(0, 0, T(-9.81));
  Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();//机身到世界的变换矩阵
  // in old code, Rbod * se_acc + g
  //输入量a 世界下
  Vec3<T> a = this->_stateEstimatorData.result->aWorld + g; 
  
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();
  Vec3<T> p0, v0;
  //初始位置 速度
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];
  
//构成状态变量等
  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
    Quadruped<T>& quadruped =
        *(this->_stateEstimatorData.legControllerData->quadruped);
    Vec3<T> ph = quadruped.getHipLocation(i);  // hip positions relative to CoM 相对于CoM的髋位置
    // hw_i->leg_controller->leg_datas[i].p; 
    Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;//足端位置在机身坐标系
    // hw_i->leg_controller->leg_datas[i].v;
    Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v;  //足端速度在机身坐标系
    Vec3<T> p_f = Rbod * p_rel;//足端位置在世界坐标系描述 即方向 大小 没有位置
	
	//足端速度在世界坐标系描述 机身转动导致足端速度+足端本身速度
    Vec3<T> dp_f =
        Rbod *(this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);
		
//更新四条腿用索引
    qindex = 6 + i1;
    rindex1 = i1;//p
    rindex2 = 12 + i1;//V
    rindex3 = 24 + i;//Z

    T trust = T(1);
    T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));//获得接触状态估计 
	//获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
	//脚i的测量协方差在摆动过程中被提高到一个很高的值，因此在这个融合过程中，摆动腿的测量值被有效地忽略了
    //T trust_window = T(0.25);
    T trust_window = T(0.2);

//在开始和结束支撑相窗口范围 当前相位在窗口范围的百分比
//trust 一般为1
    if (phase < trust_window) {
      trust = phase / trust_window;
    } 
	else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }
    //T high_suspect_number(1000);
    T high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
	//摆动腿和支撑腿刚触地，即将离地时状态，测量噪声协方差增大 
    Q.block(qindex, qindex, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);//p
		
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);//p
    R.block(rindex2, rindex2, 3, 3) =(T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);//v
    R(rindex3, rindex3) =(T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);//z

    trusts(i) = trust;
//处理后的
    _ps.segment(i1, 3) = -p_f;//足端位置在世界坐标系描述
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);//足端速度在世界坐标系描述
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<T, 28, 1> y;
  
  y << _ps, _vs, pzs;
  
  //卡尔曼滤波
  _xhat = _A * _xhat + _B * a;//状态预测方程
  
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;//不确定性预测方程
  
  //卡尔曼增益准备
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;//预测的观测值
  
  Eigen::Matrix<T, 28, 1> ey = y - yModel;//误差 卡尔曼增益计算准备
  
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;//卡尔曼增益计算准备

  // todo compute LU only once
  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);//求逆？？
  
  _xhat += Pm * Ct * S_ey;//？？

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);//？?
  
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;//最佳估计不确定性协方差??

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();//??
  _P = (_P + Pt) / T(2);//??

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {//??
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }
//输出状态量
  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody =
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run() {
  this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
  this->_stateEstimatorData.result->vWorld =
      this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
