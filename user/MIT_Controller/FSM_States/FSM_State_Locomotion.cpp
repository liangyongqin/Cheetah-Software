/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 * 用于机器人移动的FSM状态。管理特定于接触的逻辑并处理对控制器的接口调用。
 * 这种状态应该独立于控制器、步态和期望的轨迹。
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 * 将状态特定信息传递给通用FSM状态构造函数的FSM状态的构造函数。
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  //根据机型实例化MPC控制器
  if (_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH)
  {
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
                                      27 / (1000. * _controlFSMData->controlParameters->controller_dt),
                                      _controlFSMData->userParameters);
  }
  else if (_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3)
  {
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
                                      33 / (1000. * _controlFSMData->controlParameters->controller_dt),
                                      _controlFSMData->userParameters);
  }
  else
  {
    assert(false);
  }
  //安全检查turnOn
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  //关闭足部pos命令，因为它在WBC中被设置为操作任务
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  //将GRF和footstep初始化为零
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  // Default is to not transition 默认是不转换
  this->nextStateName = this->stateName;

  // Reset the transition data 重置转换数据
  this->transitionData.zero();
  cMPCOld->initialize();
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;//切换状态时默认trot步态
  printf("[FSM LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 * 调用要在每个控制循环迭代中执行的函数
 */
template <typename T>
void FSM_State_Locomotion<T>::run()
{
  // Call the locomotion control logic for this iteration
  //为这个迭代调用移动控制逻辑
  LocomotionControlStep();
}

extern rc_control_settings rc_control;

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 * 管理哪些状态可以由用户命令或状态事件触发器转换为。
 * 状态转移
 *
 * @return the enumerated FSM state name to transition into
 * 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition()
{
  // Get the next state
  iter++;

  // Switch FSM control mode
  //切换FSM控制模式
  if (locomotionSafe())//当前姿态安全才可以切换状态
  {
    switch ((int)this->_data->controlParameters->control_mode)
    {
    case K_LOCOMOTION:
      break;

    case K_BALANCE_STAND:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::BALANCE_STAND;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_PASSIVE://啥都不干 
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::PASSIVE;

      // Transition time is immediate 过渡时间是直接的
      this->transitionDuration = 0.0;

      break;

    case K_STAND_UP://站起
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = 0.;
      break;

    case K_RECOVERY_STAND://摔倒翻身
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      this->transitionDuration = 0.;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_LOCOMOTION << " to "
                << this->_data->controlParameters->control_mode << std::endl;
    }
  }
  else
  {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
  }

  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 * 处理机器人在状态之间的实际转换。在转换完成时返回true。
 * 状态转移
 *
 * @return true if transition is complete 在转换完成时返回true。
 */
template <typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition()
{
  // Switch FSM control mode
  switch (this->nextStateName)
  {
  case FSM_StateName::BALANCE_STAND:
    LocomotionControlStep();

    //从移动转换到站立时缓慢过度状态  状态转换完成
    iter++;
    if (iter >= this->transitionDuration * 1000)
    {
      this->transitionData.done = true;
    }
    else
    {
      this->transitionData.done = false;
    }

    break;

  case FSM_StateName::PASSIVE:
    this->turnOffAllSafetyChecks();

    this->transitionData.done = true;

    break;

  case FSM_StateName::STAND_UP:
    this->transitionData.done = true;
    break;

  case FSM_StateName::RECOVERY_STAND:
    this->transitionData.done = true;
    break;

  case FSM_StateName::VISION:
    this->transitionData.done = true;
    break;

  default:
    std::cout << "[CONTROL FSM] Something went wrong in transition"
              << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

//移动是否安全可用
template <typename T>
bool FSM_State_Locomotion<T>::locomotionSafe()
{
  auto &seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll))
  {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch))
  {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for (int leg = 0; leg < 4; leg++)
  {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if (p_leg[2] > 0)
    {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if (std::fabs(p_leg[1] > 0.18))
    {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if (std::fabs(v_leg) > 9.)
    {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit()
{
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 * 通过调用适当的平衡控制器并解析每个姿态或摆动腿的结果，计算每个脚的腿部控制器的命令。
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep()
{
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  // estimateContact();

  cMPCOld->run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];


//下面操作很迷 可能是将WBC控制后的输出更新腿部控制命令
  for (int leg(0); leg < 4; ++leg)
  {
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

  if (this->_data->userParameters->use_wbc > 0.9)
  {
    _wbc_data->pBody_des = cMPCOld->pBody_des;
    _wbc_data->vBody_des = cMPCOld->vBody_des;
    _wbc_data->aBody_des = cMPCOld->aBody_des;

    _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;

    for (size_t i(0); i < 4; ++i)
    {
      _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i];
    }
    _wbc_data->contact_state = cMPCOld->contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }
  for (int leg(0); leg < 4; ++leg)
  {
    //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
    //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }
}

/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 * 用于支撑腿独立阻抗控制。防止腿部打滑和弹跳，以及在高速时跟踪脚部速度。未看到引用
 */
template <typename T>
void FSM_State_Locomotion<T>::StanceLegImpedanceControl(int leg)
{
  // Impedance control for the stance leg 支撑腿的阻抗控制
  this->cartesianImpedanceControl(
      leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),
      this->_data->controlParameters->stand_kp_cartesian,
      this->_data->controlParameters->stand_kd_cartesian);
}

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
