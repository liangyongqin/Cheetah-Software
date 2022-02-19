#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utilities/Utilities_print.h>
#include <Utilities/pseudoInverse.h>
#include <cppTypes.h>
#include <vector>
#include "ContactSpec.hpp"
#include "Task.hpp"

// Assume first 6 (or 3 in 2D case) joints are for the representation of
// a floating base. 假设前6个(或2D情况下的3个)关节是表示浮动基座的。

#define WB WBC<T>
//num_qdot=18
template <typename T>
class WBC
{
public:
  WBC(size_t num_qdot) : num_act_joint_(num_qdot - 6), num_qdot_(num_qdot)
  {
    Sa_ = DMat<T>::Zero(num_act_joint_, num_qdot_); //12*18
    Sv_ = DMat<T>::Zero(6, num_qdot_);              //6*18

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity(); //[0,0,0;0,1,0;0,0,1] 0是6*6
    Sv_.block(0, 0, 6, 6).setIdentity();                           //[1,0,0]
  }
  virtual ~WBC() {}

  virtual void UpdateSetting(const DMat<T> &A, const DMat<T> &Ainv,
                             const DVec<T> &cori, const DVec<T> &grav,
                             void *extra_setting = NULL) = 0;

  virtual void MakeTorque(DVec<T> &cmd, void *extra_input = NULL) = 0;

protected:
  // full rank fat matrix only 求动态一致伪逆
  void _WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv,
                        double threshold = 0.0001)
  {
    DMat<T> lambda(J * Winv * J.transpose());
    DMat<T> lambda_inv;
    pseudoInverse(lambda, threshold, lambda_inv);
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  size_t num_act_joint_; //活动腿数
  size_t num_qdot_;      //qdot数 即全部状态数[qf,qj] 18*1

  DMat<T> Sa_; // Actuated joint 12*18   即乘状态向量即将浮动基座变量部分置为0 [0,0,0;0,1,0;0,0,1] 0是6*6
  DMat<T> Sv_; // Virtual joint 6*18  即乘状态向量即将关节变量部分置为0 [1，0，0] 0是6*6
  //动力学矩阵
  DMat<T> A_;    //质量矩阵
  DMat<T> Ainv_; //质量矩阵 逆
  DVec<T> cori_; //科氏力
  DVec<T> grav_; //重力

  bool b_updatesetting_;
  bool b_internal_constraint_;
};

#endif
