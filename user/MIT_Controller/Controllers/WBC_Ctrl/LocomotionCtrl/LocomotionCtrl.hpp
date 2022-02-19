#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER

#include <WBC_Ctrl/WBC_Ctrl.hpp>

template <typename T>
class LocomotionCtrlData
{
public:
  Vec3<T> pBody_des;     //期望坐标
  Vec3<T> vBody_des;     //速度
  Vec3<T> aBody_des;     //加速度
  Vec3<T> pBody_RPY_des; //欧拉角
  Vec3<T> vBody_Ori_des; //欧拉角速度

  Vec3<T> pFoot_des[4]; //期望足端坐标 四足
  Vec3<T> vFoot_des[4]; //速度 四足
  Vec3<T> aFoot_des[4]; //加速度 四足
  Vec3<T> Fr_des[4];    //足端力 四足

  Vec4<T> contact_state; //接触状态
};

template <typename T>
class LocomotionCtrl : public WBC_Ctrl<T>
{
public:
  LocomotionCtrl(FloatingBaseModel<T> model);
  virtual ~LocomotionCtrl();

protected:
  virtual void _ContactTaskUpdate(
      void *input, ControlFSMData<T> &data);
  virtual void _ContactTaskUpdateTEST(void *input, ControlFSMData<T> &data);
  void _ParameterSetup(const MIT_UserParameters *param);
  void _CleanUp();
  virtual void _LCM_PublishData();

  LocomotionCtrlData<T> *_input_data; //输入数据
                                      
  //任务即论文式16之后的公式                                   
  //机身相关任务
  Task<T> *_body_pos_task;            //机身位置任务
  Task<T> *_body_ori_task;            //机身角度任务
  //脚的相关任务
  Task<T> *_foot_task[4]; //四腿任务
  ContactSpec<T> *_foot_contact[4]; //四脚接触

  Vec3<T> pre_foot_vel[4]; //上次足端速度

  Vec3<T> _Fr_result[4]; //四脚反力输出
  Quat<T> _quat_des;     //期望四元数
};

#endif
