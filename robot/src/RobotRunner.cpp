/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers. 运行机器人控制器的通用框架。
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

//构造函数 将运行框架加入任务管理器
/**
在hardwareBridge使用
实例化运行器 传入控制器，任务管理器 参数 名称 
  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");
**/
RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name),//添加任务
  _lcm(getLcmUrl(255)) {

    _robot_ctrl = robot_ctrl;//目前为空 之后应该会定义
  }

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 初始化机器人模型，状态估计器，腿部控制器，
*机器人数据，以及任何控制逻辑特定的数据
 */
 
 
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object 选择机器人类型
  if (robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  // Initialize the model and robot data 初始化模型和机器人数据
  _model = _quadruped.buildModel();//机器人模型
  _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);//初始化开机时腿的位置

  // Always initialize the leg controller and state estimator 始终初始化腿控制器和状态估计器
  _legController = new LegController<float>(_quadruped);//初始化腿控制器
  
  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);//构造状态估计器 传入参数指针或值
	  
  initializeStateEstimator(false);//重置状态估计 此文件最后定义

  memset(&rc_control, 0, sizeof(rc_control_settings));
  // Initialize the DesiredStateCommand object 初始化所需的状态命令对象
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,
        &rc_control,
        controlParameters,
        &_stateEstimate,
        controlParameters->controller_dt);

  // Controller initializations 控制器初始化
  _robot_ctrl->_model = &_model; //模型
  _robot_ctrl->_quadruped = &_quadruped;//四足机器人物理特性的表征
  _robot_ctrl->_legController = _legController;//腿部控制器，对象
  _robot_ctrl->_stateEstimator = _stateEstimator;//状态估计器
  _robot_ctrl->_stateEstimate = &_stateEstimate;//状态估计值
  _robot_ctrl->_visualizationData= visualizationData;//可视化数据
  _robot_ctrl->_robotType = robotType;//机器人类型
  _robot_ctrl->_driverCommand = driverCommand;//驱动命令
  _robot_ctrl->_controlParameters = controlParameters;//控制参数
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;//期望状态命令

  _robot_ctrl->initializeController();//初始化控制器

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 通过调用每个主要组件来运行整个机器人控制系统运行它们各自的步骤。
 运行周期从参数文件来
 */
void RobotRunner::run() {
  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  //cheetahMainVisualization->p = _stateEstimate.position;
  visualizationData->clear();

  // Update the data from the robot 更新来自机器人的数据
  setupStep();//在运行用户代码之前，设置腿控件和估计器

  static int count_ini(0);
  ++count_ini;
  //之前是开启这里关闭计数50次后开启腿控制
  if (count_ini < 10) {
    _legController->setEnabled(false);
  } else if (20 < count_ini && count_ini < 30) {
    _legController->setEnabled(false);
  } else if (40 < count_ini && count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);

    if( (rc_control.mode == 0) && controlParameters->use_rc ) {
      if(count_ini%1000 ==0)   printf("ESTOP!\n");
      for (int leg = 0; leg < 4; leg++) {
        _legController->commands[leg].zero();
      }
      _robot_ctrl->Estop();//停止？初始化 代码这么写的void Estop(){ _controlFSM->initialize(); }
    }else {
      // Controller 控制器
      if (!_jpos_initializer->IsInitialized(_legController)) {//初始位置
        Mat3<float> kpMat;
        Mat3<float> kdMat;
        // Update the jpos feedback gains 更新jpos反馈增益
        if (robotType == RobotType::MINI_CHEETAH) {
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
        } else if (robotType == RobotType::CHEETAH_3) {
          kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
          kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        } else {
          assert(false);
        } 

        for (int leg = 0; leg < 4; leg++) {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
      } else {//初始完成
        // Run Control  运行控制
        _robot_ctrl->runController();
        cheetahMainVisualization->p = _stateEstimate.position;

        // Update Visualization
        _robot_ctrl->updateVisualization(); 更新可视化
        cheetahMainVisualization->p = _stateEstimate.position;
      }
    }

  }



  // Visualization (will make this into a separate function later)
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      cheetahMainVisualization->q[leg * 3 + joint] =
        _legController->datas[leg].q[joint];
    }
  }
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->p = _stateEstimate.position;
  cheetahMainVisualization->quat = _stateEstimate.orientation;

  // Sets the leg controller commands for the robot appropriate commands
  //为机器人设置适当的腿部控制器命令
  finalizeStep();//更新command
}

/*!
 * Before running user code, setup the leg control and estimators 在运行用户代码之前，设置腿控件和估计器
 */
void RobotRunner::setupStep() {
  // Update the leg data
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
	
  } else if (robotType == RobotType::CHEETAH_3) {//选择机型更新腿部控制参数
    _legController->updateData(tiBoardData);//更新 从tiBoardData来的值 到datas
  } else {
    assert(false);
  }

  // Setup the leg controller for a new iteration 为新的迭代设置腿控制器
  _legController->zeroCommand();//腿部控制命令清零
  _legController->setEnabled(true);//开启腿部控制
  _legController->setMaxTorqueCheetah3(208.5);//最大力矩

  // state estimator 状态估计器
  // check transition to cheater mode:  检查转换到作弊模式
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode: 检查从作弊模式转换
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }
//获得控制设置
  get_rc_control_settings(&rc_control);

  // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 在用户代码之后，发送腿命令，更新状态估计，并发布调试数据
 */
void RobotRunner::finalizeStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
	//选择机型
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else {
    assert(false);
  }
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
  _iterations++;
}

/*!
 * Reset the state estimator in the given mode. 重置给定模式下的状态估计器。
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();//删除所有估计器
  _stateEstimator->addEstimator<ContactEstimator<float>>();//添加接触状态估计器
  Vec4<float> contactDefault;//
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  
  if (cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
	
  } else {//使用这个 添加位置角度状态估计器
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  }
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
