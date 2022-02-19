#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FSM_States/ControlFSMData.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include "cppTypes.h"
#include <WBC/WBIC/WBIC.hpp>
#include <WBC/WBIC/KinWBC.hpp>

#include <lcm-cpp.hpp>
#include "wbc_test_data_t.hpp"

#define WBCtrl WBC_Ctrl<T>

class MIT_UserParameters;
//WBC_CTRL 是一个父类 主要使用是LocomotionCtrl
template <typename T>
class WBC_Ctrl
{
public:
  WBC_Ctrl(FloatingBaseModel<T> model);
  virtual ~WBC_Ctrl();

  void run(void *input, ControlFSMData<T> &data);
  void setFloatingBaseWeight(const T &weight)
  {
    _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
  }

protected:
  virtual void _ContactTaskUpdate(void *input, ControlFSMData<T> &data) = 0;
  virtual void _ContactTaskUpdateTEST(void *input, ControlFSMData<T> &data)
  {
    (void)input;
    (void)data;
  }
  virtual void _LCM_PublishData() {}
  void _UpdateModel(const StateEstimate<T> &state_est, const LegControllerData<T> *leg_data);
  void _UpdateLegCMD(ControlFSMData<T> &data);
  void _ComputeWBC();

  KinWBC<T> *_kin_wbc;
  WBIC<T> *_wbic;
  WBIC_ExtraData<T> *_wbic_data;

  FloatingBaseModel<T> _model; //浮动刚体模型
  std::vector<ContactSpec<T> *> _contact_list;//接触列表用于存放接触腿任务 维度为6+? ?=0~4
  std::vector<Task<T> *> _task_list;//任务列表负责存放摆动腿任务

  //动力学矩阵
  DMat<T> _A;        //质量矩阵
  DMat<T> _Ainv;     //质量矩阵 逆
  DVec<T> _grav;     //重力
  DVec<T> _coriolis; //科氏力

  FBModelState<T> _state;

  DVec<T> _full_config;////动力学模型全部状态变量[qf,qj](WBIC论文中 multi-body dynamics)
  DVec<T> _tau_ff;   //前馈力矩
  DVec<T> _des_jpos; //期望
  DVec<T> _des_jvel; //期望关节速度

  std::vector<T> _Kp_joint, _Kd_joint; //关节pd参数
  //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

  unsigned long long _iter;

  lcm::LCM _wbcLCM;
  wbc_test_data_t _wbc_data_lcm;
};
#endif
