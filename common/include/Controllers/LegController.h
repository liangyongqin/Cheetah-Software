/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms 常用的腿部控制接口和腿部控制算法
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots 为小型猎豹和猎豹3型机器人实现低水平的腿部控制
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * 	body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * 所有的量都在“腿坐标系”中，与身体坐标系有相同的方向，但是移动了，使得0,0,0在ab/ad轴上(“髋坐标系”)。
 * frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppTypes.h"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"

/*!
 * Data sent from the control algorithm to the legs.
 * 数据从控制算法发送到腿部。
 */
template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 * 从腿返回到控制代码的数据
 */
template <typename T>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

  void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero();

  Vec3<T> q, qd, p, v;//关节角度 关节角速度 足端位置 足端速度 
  Mat3<T> J;//雅可比
  Vec3<T> tauEstimate;//估计力矩反馈
  Quadruped<T>* quadruped;// 机器人类型 cheetah3 or mini
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 *四足4条腿的控制器。适用于小型猎豹和猎豹3
 */
template <typename T>
class LegController {
 public:
  LegController(Quadruped<T>& quad) : _quadruped(quad) {
    for (auto& data : datas) data.setQuadruped(_quadruped);
  }

  void zeroCommand();//腿部控制命令清零
  void edampCommand(RobotType robot, T gain);
  
  void updateData(const SpiData* spiData);
  void updateData(const TiBoardData* tiBoardData);
  
  void updateCommand(SpiCommand* spiCommand);
  void updateCommand(TiBoardCommand* tiBoardCommand);
  void setEnabled(bool enabled) { _legsEnabled = enabled; };
  void setLcm(leg_control_data_lcmt* data, leg_control_command_lcmt* command);

  /*!
   * Set the maximum torque.  This only works on cheetah 3!
   */
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
