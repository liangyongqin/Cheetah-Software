/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 * FSM声明不调用任何控件。意味着一个安全的状态
 * 机器人不应该做任何事情，因为所有的命令将被设置为0。
 */

#include "FSM_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE")
{
  // Do nothing 啥也不做
  // Set the pre controls safety checks 设置预控制安全检查
  this->checkSafeOrientation = false;

  // Post control safety checks  控制后安全检查
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Passive<T>::onEnter()
{
  // Default is to not transition 默认是不过渡
  this->nextStateName = this->stateName;

  // Reset the transition data 重置转换数据
  this->transitionData.zero();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 * 调用要在每个控制循环迭代中执行的函数
 */
template <typename T>
void FSM_State_Passive<T>::run()
{
  // Do nothing, all commands should begin as zeros 什么都不做，所有的命令都应该以0开头
  testTransition();
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 * 处理机器人在状态之间的实际转换。在转换完成时返回true。
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Passive<T>::testTransition()
{
  this->transitionData.done = true;
  return this->transitionData;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 * 管理哪些状态可以由用户命令或状态事件触发器转换为。
 * 状态转移
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Passive<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode 切换FSM控制模式
  switch ((int)this->_data->controlParameters->control_mode)
  {
  case K_PASSIVE: // normal c (0)
    // Normal operation for state based transitions
    break;

  case K_JOINT_PD:
    // Requested switch to joint PD control
    this->nextStateName = FSM_StateName::JOINT_PD;
    break;

  case K_STAND_UP:
    // Requested switch to joint PD control
    this->nextStateName = FSM_StateName::STAND_UP;
    break;

  case K_RECOVERY_STAND:
    // Requested switch to joint PD control
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
              << K_PASSIVE << " to "
              << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state 得到下一个状态
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed. 
 * 处理机器人在状态之间的实际转换。在转换完成时返回true。
 * 状态转移
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Passive<T>::transition()
{
  // Finish Transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Passive<T>::onExit()
{
  // Nothing to clean up when exiting
}

// template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;
