#ifndef CONTROLFSM_H
#define CONTROLFSM_H
/*MIT-Cheetah Software
**             Email:@qq.com   QQ:1370780559
**---------------------------------------------------------
**  Description: 此文件注释由本人完成，仅为个人理解,本人水平有限，还请见谅
**  interpreter     : NaCl
*/
#include <iostream>

// Contains all of the control related data 包含所有与控制相关的数据
#include "ControlFSMData.h"

// Checks the robot state and commands for safety 检查机器人状态和安全命令
#include "SafetyChecker.h"

// FSM States
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_BalanceStand.h"
#include "../FSM_States/FSM_State_ImpedanceControl.h"
#include "../FSM_States/FSM_State_JointPD.h"
#include "../FSM_States/FSM_State_Locomotion.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_StandUp.h"
#include "../FSM_States/FSM_State_RecoveryStand.h"
#include "../FSM_States/FSM_State_Vision.h"
#include "../FSM_States/FSM_State_BackFlip.h"
#include "../FSM_States/FSM_State_FrontJump.h"

/**
 * Enumerate all of the operating modes 枚举所有操作模式
 */
enum class FSM_OperatingMode { 
  NORMAL, TRANSITIONING, ESTOP, EDAMP };

/**
 *
 */
template <typename T>
struct FSM_StatesList {
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_JointPD<T>* jointPD;
  FSM_State_ImpedanceControl<T>* impedanceControl;
  FSM_State_StandUp<T>* standUp;
  FSM_State_BalanceStand<T>* balanceStand;
  FSM_State_Locomotion<T>* locomotion;
  FSM_State_RecoveryStand<T>* recoveryStand;
  FSM_State_Vision<T>* vision;
  FSM_State_BackFlip<T>* backflip;
  FSM_State_FrontJump<T>* frontJump;
};


/**
 *
 */
template <typename T>
struct FSM_ControllerList {
};


/**
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM {
 public:
  ControlFSM(Quadruped<T>* _quadruped,
             StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController, GaitScheduler<T>* _gaitScheduler,
             DesiredStateCommand<T>* _desiredStateCommand,
             RobotControlParameters* controlParameters,
             VisualizationData* visualizationData,
             MIT_UserParameters* userParameters);

  // Initializes the Control FSM instance  初始化有限状态机
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs  运行FSM逻辑，处理状态转换和正常运行
  void runFSM();

  // This will be removed and put into the SafetyChecker class 这将被删除并放入SafetyChecker类
  FSM_OperatingMode safetyPreCheck(); 

  //
  FSM_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested 从请求时创建的状态列表中获取下一个FSM_State
  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Prints the current FSM status 打印当前状态
  void printInfo(int opt);

  // Contains all of the control related data 包含所有控制相关数据
  ControlFSMData<T> data;

  // FSM state information  状态信息
  FSM_StatesList<T> statesList;  // holds all of the FSM States 保存所有状态
  FSM_State<T>* currentState;    // current FSM state 当前状态
  FSM_State<T>* nextState;       // next FSM state 下一个状态
  FSM_StateName nextStateName;   // next FSM state name 下一个状态名

  // Checks all of the inputs and commands for safety  检查所有输入和命令是否安全
  SafetyChecker<T>* safetyChecker;

  TransitionData<T> transitionData;

 private:
  // Operating mode of the FSM  FSM的工作模式
  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations 控制打印频率
  int printNum = 10000;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;  // make larger than printNum to not print

  int iter = 0;

  //lcm::LCM state_estimator_lcm;
  //state_estimator_lcmt _state_estimator;
};

#endif  // CONTROLFSM_H
