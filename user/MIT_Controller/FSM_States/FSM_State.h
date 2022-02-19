#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"
#include "TransitionData.h"
#include "Controllers/GaitScheduler.h"

#include <Controllers/BalanceController/BalanceController.hpp>

// Normal robot states
#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_BALANCE_STAND 3
#define K_LOCOMOTION 4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6
#define K_VISION 8
#define K_BACKFLIP 9
#define K_FRONTJUMP 11

// Specific control states
#define K_JOINT_PD 51
#define K_IMPEDANCE_CONTROL 52

#define K_INVALID 100

/**
 * Enumerate all of the FSM states so we can keep track of them.
 * 枚举所有FSM状态，以便我们可以跟踪它们。
 */
enum class FSM_StateName
{
  INVALID,
  PASSIVE,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  STAND_UP,
  BALANCE_STAND,
  LOCOMOTION,
  RECOVERY_STAND,
  VISION,
  BACKFLIP,
  FRONTJUMP
};

/**
 *状态机状态父类 
 *目前看到的状态，大部分参数没有用到
 */
template <typename T>
class FSM_State
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  FSM_State(ControlFSMData<T> *_controlFSMData, FSM_StateName stateNameIn,
            std::string stateStringIn);

  // Behavior to be carried out when entering a state 进入一种状态时的行为
  virtual void onEnter() = 0; // {}

  // Run the normal behavior for the state 运行状态的正常行为
  virtual void run() = 0; //{}

  // Manages state specific transitions 管理特定于状态的转换
  virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

  // Runs the transition behaviors and returns true when done transitioning 运行转换行为，并在完成转换时返回true
  virtual TransitionData<T> transition() { return transitionData; }

  // Behavior to be carried out when exiting a state 退出状态时要执行的行为
  virtual void onExit() = 0; // {}

  //关节PD控制
  void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);
  //笛卡尔阻抗控制
  void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,
                                 Vec3<double> kp_cartesian,
                                 Vec3<double> kd_cartesian);
  //脚踏启发式布置  未启用                   
  void footstepHeuristicPlacement(int leg);

  //运行控制器
  void runControls();
  void runBalanceController();
  void runWholeBodyController();
  void runConvexModelPredictiveController();
  void runRegularizedPredictiveController();

  //参数安全检测 置真/假
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();

  // Holds all of the relevant control data 保存所有相关的控制数据
  ControlFSMData<T> *_data;

  // FSM State info FSM状态信息
  FSM_StateName stateName;     // enumerated name of the current state 当前状态名枚举类 每种状固定不变 
  FSM_StateName nextStateName; // enumerated name of the next state 下一个状态名枚举类
  std::string stateString;     // state name string 状态名

  // Transition parameters 状态转移参数
  T transitionDuration;             // transition duration time 转移时间间隔
  T tStartTransition;               // time transition starts 转移开始时间
  TransitionData<T> transitionData; //相关数据的结构，可在转换期间用于在状态间传递数据。

  // Pre controls safety checks 预先控制安全检查
  bool checkSafeOrientation = false; // check roll and pitch

  // Post control safety checks 控制后安全检查
  bool checkPDesFoot = false;         // do not command footsetps too far 不要把脚弄得太远
  bool checkForceFeedForward = false; // do not command huge forces 不使用较大力
  bool checkLegSingularity = false;   // do not let leg  不用腿

  // Leg controller command placeholders for the whole robot (3x4 matrices) 整个机器人的腿部控制器命令占位符(3x4矩阵)
  Mat34<T> jointFeedForwardTorques; // feed forward joint torques 前馈关节力矩
  Mat34<T> jointPositions;          // joint angle positions 关节角度
  Mat34<T> jointVelocities;         // joint angular velocities 关节角速度
  Mat34<T> footFeedForwardForces;   // feedforward forces at the feet 前馈力
  Mat34<T> footPositions;           // cartesian foot positions 足端位置
  Mat34<T> footVelocities;          // cartesian foot velocities 足端速度

  // Footstep locations for next step 下一步的脚步位置
  Mat34<T> footstepLocations;

  // Higher level Robot body controllers 高级机器人身体控制器
  BalanceController balanceController;
  // ModelPredictiveController cMPC
  // RegularizedPredictiveController RPC

private:
  // Create the cartesian P gain matrix 增益 直角坐标
  Mat3<float> kpMat;

  // Create the cartesian D gain matrix 增益 直角坐标
  Mat3<float> kdMat;
};

#endif // FSM_State_H
