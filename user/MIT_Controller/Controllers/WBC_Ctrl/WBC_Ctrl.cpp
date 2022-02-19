#include "WBC_Ctrl.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/Timer.h>




/*
constexpr size_t num_act_joint = 12;
constexpr size_t num_q = 19;
constexpr size_t dim_config = 18;
constexpr size_t num_leg = 4;
constexpr size_t num_leg_joint = 3;
*/
template <typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model) : _full_config(cheetah::num_act_joint + 7),
                                                    _tau_ff(cheetah::num_act_joint),
                                                    _des_jpos(cheetah::num_act_joint),
                                                    _des_jvel(cheetah::num_act_joint),
                                                    _wbcLCM(getLcmUrl(255))
{
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[5] = 0.1;
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

//析构函数
template <typename T>
WBC_Ctrl<T>::~WBC_Ctrl()
{
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end())
  {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end())
  {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

//计算WBIC
template <typename T>
void WBC_Ctrl<T>::_ComputeWBC()
{
  // TEST 运动学WBC
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  // WBIC 力WBC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
}

//运行
template <typename T>
void WBC_Ctrl<T>::run(void *input, ControlFSMData<T> &data)
{
  ++_iter;

  // Update Model 更新模型
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // Task & Contact Update 更新任务和接触
  _ContactTaskUpdate(input, data);

  // WBC Computation
  _ComputeWBC();

  // TEST
  //T dt(0.002);
  //for(size_t i(0); i<12; ++i){
  //_des_jpos[i] = _state.q[i] + _state.qd[i] * dt + 0.5 * _wbic_data->_qddot[i+6] * dt * dt;
  //_des_jvel[i] = _state.qd[i] + _wbic_data->_qddot[i+6]*dt;
  //}

  //_ContactTaskUpdateTEST(input, data);
  //_ComputeWBC();
  // END of TEST

  // Update Leg Command 更新腿部控制命令
  _UpdateLegCMD(data);

  // LCM publish lcm发送
  _LCM_PublishData();
}

//更新腿部控制命令 ControlFSMData._legController的值
template <typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> &data)
{
  LegControllerCommand<T> *cmd = data._legController->commands;
  //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;

  //赋值
  for (size_t leg(0); leg < cheetah::num_leg; ++leg)
  {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx)
    {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];

      cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];

      //if(contact[leg] > 0.){ // Contact
      //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
      //}else{
      //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint_swing[jidx];
      //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint_swing[jidx];
      //}
    }
  }

  // Knee joint non flip barrier 膝关节无翻转障碍
  for (size_t leg(0); leg < 4; ++leg)
  {
    if (cmd[leg].qDes[2] < 0.3)
    {
      cmd[leg].qDes[2] = 0.3;
    }
    if (data._legController->datas[leg].q[2] < 0.3)
    {
      T knee_pos = data._legController->datas[leg].q[2];
      cmd[leg].tauFeedForward[2] = 1. / (knee_pos * knee_pos + 0.02);
    }
  }
}


//根据状态估计更新动力学模型
template <typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> &state_est,
                               const LegControllerData<T> *leg_data)
{

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for (size_t i(0); i < 3; ++i)
  {
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i + 3] = state_est.vBody[i];

    for (size_t leg(0); leg < 4; ++leg)
    {
      _state.q[3 * leg + i] = leg_data[leg].q[i];//关节角度
      _state.qd[3 * leg + i] = leg_data[leg].qd[i];//关节速度

      _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];//动力学模型全部状态变量
    }
  }
  _model.setState(_state);           //给动力学模型传入状态

  _model.contactJacobians();         //计算每个接触点的雅可比矩阵
  _model.massMatrix();               //质量矩阵
  _model.generalizedGravityForce();  //生成重力矩阵
  _model.generalizedCoriolisForce(); //生成科氏力矩阵

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();
}

template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
