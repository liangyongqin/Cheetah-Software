/*MIT-Cheetah Software
**             Email:@qq.com   QQ:1370780559
**---------------------------------------------------------
**  Description: 此文件注释由本人完成，仅为个人理解,本人水平有限，还请见谅
**  interpreter     : NaCl
*/



/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"
#include <rt/rt_rc_interface.h>

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 * 控件FSM的构造函数。
 * 传入所有必要的数据并将其存储在结构中。使用启动状态和操作模式初始化FSM。
 * 传入参数从robotrunner的给RobotController* _robot_ctrl赋值来
 *
 * @param _quadruped the quadruped information 四足的信息
 * @param _stateEstimator contains the estimated states 包含估计状态
 * @param _legController interface to the leg controllers 与腿部控制器的接口
 * @param _gaitScheduler controls scheduled foot contact modes  控制预定的足部接触模式
 * @param _desiredStateCommand gets the desired COM state trajectories 获取所需的COM状态轨迹
 * @param controlParameters passes in the control parameters from the GUI 从GUI中传入控制参数
 */
template <typename T>
ControlFSM<T>::ControlFSM(Quadruped<T> *_quadruped,
                          StateEstimatorContainer<T> *_stateEstimator,
                          LegController<T> *_legController,
                          GaitScheduler<T> *_gaitScheduler,
                          DesiredStateCommand<T> *_desiredStateCommand,
                          RobotControlParameters *controlParameters,
                          VisualizationData *visualizationData,
                          MIT_UserParameters *userParameters)
{
  // Add the pointers to the ControlFSMData struct 将指针添加到ControlFSMData结构
  data._quadruped = _quadruped;
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data._gaitScheduler = _gaitScheduler;
  data._desiredStateCommand = _desiredStateCommand;
  data.controlParameters = controlParameters;
  data.visualizationData = visualizationData;
  data.userParameters = userParameters;

  // Initialize and add all of the FSM States to the state list
  //初始化并将所有FSM状态添加到状态列表中
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.jointPD = new FSM_State_JointPD<T>(&data);
  statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  statesList.standUp = new FSM_State_StandUp<T>(&data);
  statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);
  statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
  statesList.vision = new FSM_State_Vision<T>(&data);
  statesList.backflip = new FSM_State_BackFlip<T>(&data);
  statesList.frontJump = new FSM_State_FrontJump<T>(&data);

  safetyChecker = new SafetyChecker<T>(&data); //进行安全检查并限制操作

  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 * 使用默认设置初始化控件FSM。应设置为被动状态和正常工作模式。
 */
template <typename T>
void ControlFSM<T>::initialize()
{
  // Initialize a new FSM State with the control data
  //用控制数据初始化一个新的FSM状态
  currentState = statesList.passive;

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition 初始化为不处于过渡状态
  nextState = currentState;

  // Initialize FSM mode to normal operation 初始化FSM模式为正常操作
  operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 * 调用每个控制循环迭代。确定机器人是否可以安全地运行控件，
 * 并检查当前状态是否有任何转换。如果一切正常，则运行常规状态行为。
 */
template <typename T>
void ControlFSM<T>::runFSM()
{
  // Publish state estimator data to other computer
  //for(size_t i(0); i<3; ++i){
  //_state_estimator.p[i] = data._stateEstimator->getResult().position[i];
  //_state_estimator.quat[i] = data._stateEstimator->getResult().orientation[i];
  //}
  //_state_estimator.quat[3] = data._stateEstimator->getResult().orientation[3];
  //state_estimator_lcm.publish("state_estimator_ctrl_pc", &_state_estimator);

  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

  if (data.controlParameters->use_rc)
  {                                                           //是否使用遥控器
    int rc_mode = data._desiredStateCommand->rcCommand->mode; //控制模式设定
    if (rc_mode == RC_mode::RECOVERY_STAND)
    {
      data.controlParameters->control_mode = K_RECOVERY_STAND;
    }
    else if (rc_mode == RC_mode::LOCOMOTION)
    {
      data.controlParameters->control_mode = K_LOCOMOTION;
    }
    else if (rc_mode == RC_mode::QP_STAND)
    {
      data.controlParameters->control_mode = K_BALANCE_STAND;
    }
    else if (rc_mode == RC_mode::VISION)
    {
      data.controlParameters->control_mode = K_VISION;
    }
    else if (rc_mode == RC_mode::BACKFLIP || rc_mode == RC_mode::BACKFLIP_PRE)
    {
      data.controlParameters->control_mode = K_BACKFLIP;
    }
    //data.controlParameters->control_mode = K_FRONTJUMP;
    //std::cout<< "control mode: "<<data.controlParameters->control_mode<<std::endl;
  }

  // Run the robot control code if operating mode is not unsafe
  //运行机器人控制代码，如果工作模式不安全
  if (operatingMode != FSM_OperatingMode::ESTOP)
  {

    //下面为状态机
    // Run normal controls if no transition is detected
    //如果没有检测到过渡，则运行正常控件
    if (operatingMode == FSM_OperatingMode::NORMAL)
    {
      // Check the current state for any transition 检查任何转换的当前状态
      nextStateName = currentState->checkTransition();

      // Detect a commanded transition 探测指令转换
      if (nextStateName != currentState->stateName)
      {
        // Set the FSM operating mode to transitioning 将FSM工作模式设置为transitioning
        operatingMode = FSM_OperatingMode::TRANSITIONING;

        // Get the next FSM State by name 按名称获取下一个FSM状态
        nextState = getNextState(nextStateName);

        // Print transition initialized info
        //printInfo(1);
      }
      else
      { //没有状态转换则运行当前状态控制器
        // Run the iteration for the current state normally
        currentState->run();
      }
    }

    // Run the transition code while transition is occuring
    //在发生转换时运行转换代码
    if (operatingMode == FSM_OperatingMode::TRANSITIONING)
    {
      //获得转换数据，进行状态转换操作 有时候是延时切换状态 比如MPClocomotion
      transitionData = currentState->transition();

      // Check the robot state for safe operation
      safetyPostCheck();

      // Run the state transition
      if (transitionData.done) //状态转换完成 延时到
      {
        // Exit the current state cleanly 干净地退出当前状态
        currentState->onExit();

        // Print finalizing transition info
        //printInfo(2);

        // Complete the transition  完成状态转换
        currentState = nextState;

        // Enter the new current state cleanly 进入新状态
        currentState->onEnter();

        // Return the FSM to normal operation mode 操作模式设置为一般
        operatingMode = FSM_OperatingMode::NORMAL;
      }
    }
    else
    { //非转换状态 会进行检查 即任何状态下进行检查并限制
      // Check the robot state for safe operation 检查机器人状态，确保操作安全，并限制
      safetyPostCheck();
    }
  }
  else
  { // if ESTOP 停止的默认状态
    currentState = statesList.passive;
    currentState->onEnter();
    nextStateName = currentState->stateName;
  }

  // Print the current state of the FSM
  printInfo(0);

  // Increase the iteration counter
  iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 * 检查机器人状态是否符合安全操作条件。如果处于不安全状态，
 * 则在安全之前不会运行一般控制
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
  // Check for safe orientation if the current state requires it 检查安全方向 如果当前状态需要
  if (currentState->checkSafeOrientation && data.controlParameters->control_mode != K_RECOVERY_STAND)
  {
    if (!safetyChecker->checkSafeOrientation()) //姿态角不安全
    {
      operatingMode = FSM_OperatingMode::ESTOP;
      std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
    }
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 * 计算控制后，检查机器人状态是否有安全的操作指令。
 * 打印出哪个命令是不安全的。每个状态都有一个选项来开关控制它所关心的命令的检查。
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 * 这是EDamp / EStop还是继续?
 * 为清晰起见，应将每个单独的检查拆分成各自的功能
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
  // Check for safe desired foot positions 检查脚的安全位置
  if (currentState->checkPDesFoot) //是否检查
  {
    safetyChecker->checkPDesFoot();
  }

  // Check for safe desired feedforward forces 检查期望的安全前馈力
  if (currentState->checkForceFeedForward) //是否检查
  {
    safetyChecker->checkForceFeedForward();
  }

  // Default is to return the current operating mode
  //默认是返回当前的操作模式
  return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 * 命令时返回下一个FSM的批准状态。
 *
 * @param  next commanded enumerated state name 下一个命令枚举的状态名
 * @return next FSM state 下一个状态
 */
template <typename T>
FSM_State<T> *ControlFSM<T>::getNextState(FSM_StateName stateName)
{
  // Choose the correct FSM State by enumerated state name
  switch (stateName)
  {
  case FSM_StateName::INVALID:
    return statesList.invalid;

  case FSM_StateName::PASSIVE:
    return statesList.passive;

  case FSM_StateName::JOINT_PD:
    return statesList.jointPD;

  case FSM_StateName::IMPEDANCE_CONTROL:
    return statesList.impedanceControl;

  case FSM_StateName::STAND_UP:
    return statesList.standUp;

  case FSM_StateName::BALANCE_STAND:
    return statesList.balanceStand;

  case FSM_StateName::LOCOMOTION:
    return statesList.locomotion;

  case FSM_StateName::RECOVERY_STAND:
    return statesList.recoveryStand;

  case FSM_StateName::VISION:
    return statesList.vision;

  case FSM_StateName::BACKFLIP:
    return statesList.backflip;

  case FSM_StateName::FRONTJUMP:
    return statesList.frontJump;

  default:
    return statesList.invalid;
  }
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template <typename T>
void ControlFSM<T>::printInfo(int opt)
{
  switch (opt)
  {
  case 0: // Normal printing case at regular intervals
    // Increment printing iteration
    printIter++;

    // Print at commanded frequency
    if (printIter == printNum)
    {
      std::cout << "[CONTROL FSM] Printing FSM Info...\n";
      std::cout
          << "---------------------------------------------------------\n";
      std::cout << "Iteration: " << iter << "\n";
      if (operatingMode == FSM_OperatingMode::NORMAL)
      {
        std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                  << "\n";
      }
      else if (operatingMode == FSM_OperatingMode::TRANSITIONING)
      {
        std::cout << "Operating Mode: TRANSITIONING from "
                  << currentState->stateString << " to "
                  << nextState->stateString << "\n";
      }
      else if (operatingMode == FSM_OperatingMode::ESTOP)
      {
        std::cout << "Operating Mode: ESTOP\n";
      }
      std::cout << "Gait Type: " << data._gaitScheduler->gaitData.gaitName
                << "\n";
      std::cout << std::endl;

      // Reset iteration counter
      printIter = 0;
    }

    // Print robot info about the robot's status
    // data._gaitScheduler->printGaitInfo();
    // data._desiredStateCommand->printStateCommandInfo();

    break;

  case 1: // Initializing FSM State transition
    std::cout << "[CONTROL FSM] Transition initialized from "
              << currentState->stateString << " to " << nextState->stateString
              << "\n"
              << std::endl;

    break;

  case 2: // Finalizing FSM State transition
    std::cout << "[CONTROL FSM] Transition finalizing from "
              << currentState->stateString << " to " << nextState->stateString
              << "\n"
              << std::endl;

    break;
  }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;
