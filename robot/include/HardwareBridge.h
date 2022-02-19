/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#ifdef linux 

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"



/*!
 * Interface between robot and hardware
 */
class HardwareBridge {
 public:
  HardwareBridge(RobotController* robot_ctrl)
      : statusTask(&taskManager, 0.5f),
        _interfaceLCM(getLcmUrl(255)),
        _visualizationLCM(getLcmUrl(255)) {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();
        }
  void prefaultStack();//错误？？
  void setupScheduler();//设置任务
  void initError(const char* reason, bool printErrno = false);//打印初始化时错误 
  void initCommon();//初始化命令
  
  ~HardwareBridge() { delete _robotRunner; }
  //句柄们
  void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const gamepad_lcmt* msg);

  void handleInterfaceLCM();
  void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const control_parameter_request_lcmt* msg);

  void publishVisualizationLCM();
  //运行手柄总线
  void run_sbus();

 protected:
  PeriodicTaskManager taskManager; //任务管理器
  PrintTaskStatus statusTask; // 用于打印任务管理器中所有任务的状态的定期任务 打印被注释了
  GamepadCommand _gamepadCommand;//手柄命令
  VisualizationData _visualizationData;//可视化数据
  CheetahVisualization _mainCheetahVisualization;//可视化
  lcm::LCM _interfaceLCM;//接口LCM
  lcm::LCM _visualizationLCM;//可视化LCM
  
  control_parameter_respones_lcmt _parameter_response_lcmt;//控制参数回应
  
  SpiData _spiData; //控制器数据
  SpiCommand _spiCommand;//控制器命令
  
//四腿控制器 数据 命令
  TiBoardCommand _tiBoardCommand[4];
  TiBoardData _tiBoardData[4];

  bool _firstRun = true;
  
  RobotRunner* _robotRunner = nullptr;
  RobotControlParameters _robotParams; //机器人控制参数
  
  u64 _iterations = 0;
  std::thread _interfaceLcmThread;//接口LCM线程
  volatile bool _interfaceLcmQuit = false; //接口LCM退出
  
  RobotController* _controller = nullptr;//机器人控制器
  ControlParameters* _userControlParameters = nullptr;//控制参数

  int _port;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 */
class MiniCheetahHardwareBridge : public HardwareBridge {
 public:
  MiniCheetahHardwareBridge(RobotController* rc, bool load_parameters_from_file);
  void runSpi();
  void initHardware();
  void run();
  void runMicrostrain();
  void logMicrostrain();
  void abort(const std::string& reason);
  void abort(const char* reason);

 private:
  VectorNavData _vectorNavData;
  lcm::LCM _spiLcm;
  lcm::LCM _microstrainLcm;
  std::thread _microstrainThread;
  LordImu _microstrainImu;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;
};

class Cheetah3HardwareBridge : public HardwareBridge {
public:
  Cheetah3HardwareBridge(RobotController* rc);
  void runEcat();
  void initHardware();
  void run();
  void publishEcatLCM();
  // todo imu?

private:
  VectorNavData _vectorNavData;//imu数据
  lcm::LCM _ecatLCM;//自定义消息用lcm
  ecat_command_t ecatCmdLcm;//两种消息类型
  ecat_data_t ecatDataLcm;
  // nothing?
};
#endif // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
