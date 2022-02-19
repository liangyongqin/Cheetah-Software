/*============================= FSM State =============================*/
#ifndef TRANSITIONDATA_H
#define TRANSITIONDATA_H

/**
 * Struct of relevant data that can be used during transition to pass
 * data between states.
 * 相关数据的结构，可在转换期间用于在状态间传递数据。
 */
template <typename T>
struct TransitionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TransitionData() { zero(); }

  // Zero out all of the data
  void zero() {
    // Flag to mark when transition is done 在转换完成时标记
    done = false;

    // Timing parameters 时间参数
    t0 = 0.0;         // time that transition started 转变开始的时间
    tCurrent = 0.0;   // current time since transition started 当前时间自过渡开始
    tDuration = 0.0;  // overall transition duration 整体的过渡时间

    // Robot state at the beginning of transition 机器人在过渡开始时的状态
    comState0 = Vec12<T>::Zero();  // center of mass state 质心状态
    qJoints0 = Vec12<T>::Zero();   // joint positions 关节位置
    pFoot0 = Mat34<T>::Zero();     // foot positions 足端位置

    // Current robot state 当前机器人状态
    comState = Vec12<T>::Zero();  // center of mass state 质心状态
    qJoints = Vec12<T>::Zero();   // joint positions 关节位置
    pFoot = Mat34<T>::Zero();     // foot positions 足端位置
  }

  // Flag to mark when transition is done
  bool done = false; 在转换完成时标记

  // Timing parameters
  T t0;         // time that transition started 转变开始的时间
  T tCurrent;   // current time since transition started 当前时间自过渡开始
  T tDuration;  // overall transition duration 整体的过渡时间

  // Robot state at the beginning of transition 机器人在过渡开始时的状态
  Vec12<T> comState0;  // center of mass state 质心状态
  Vec12<T> qJoints0;   // joint positions 关节位置
  Mat34<T> pFoot0;     // foot positions 足端位置

  // Current robot state
  Vec12<T> comState;  // center of mass state 质心状态
  Vec12<T> qJoints;   // joint positions 关节位置
  Mat34<T> pFoot;     // foot positions 足端位置
};

template struct TransitionData<double>;
template struct TransitionData<float>;

#endif  // CONTROLFSM_H