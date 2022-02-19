/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 * 每个状态估计器对象包含许多估计器
 * 当状态估计器运行时，它运行所有的估计器。
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/VisualizationData.h"
#include "state_estimator_lcmt.hpp"

/*!
 * Result of state estimation
 *状态估计结果
 */
template <typename T>
struct StateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;//接触估计
  Vec3<T> position;//位置
  Vec3<T> vBody;//速度
  Quat<T> orientation;//四元数
  Vec3<T> omegaBody;//角速度
  RotMat<T> rBody;//旋转矩阵
  Vec3<T> rpy;//欧拉角

  Vec3<T> omegaWorld;//世界坐标角速度
  Vec3<T> vWorld;//速度
  Vec3<T> aBody, aWorld;//加速度，世界坐标下加速度
  //通过lcm发送
  void setLcm(state_estimator_lcmt& lcm_data) {
    for(int i = 0; i < 3; i++) {
      lcm_data.p[i] = position[i];
      lcm_data.vWorld[i] = vWorld[i];
      lcm_data.vBody[i] = vBody[i];
      lcm_data.rpy[i] = rpy[i];
      lcm_data.omegaBody[i] = omegaBody[i];
      lcm_data.omegaWorld[i] = omegaWorld[i];
    }

    for(int i = 0; i < 4; i++) {
      lcm_data.quat[i] = orientation[i];
    }
  }
};

/*!
 * Inputs for state estimation.
 *状态估计的输入
 * If robot code needs to inform the state estimator of something,
 * it should be added here. 
 *如果机器人代码需要通知状态估计器一些东西，应该在这里加上
 *(You should also a setter method to StateEstimatorContainer)
 *您还应该使用setter方法来设置StateEstimatorContainer
 */
template <typename T>
struct StateEstimatorData {
  StateEstimate<T>* result;  // where to write the output to 这里是输出
  VectorNavData* vectorNavData; //imu数据
  CheaterState<double>* cheaterState;
  LegControllerData<T>* legControllerData;//腿部数据
  Vec4<T>* contactPhase;//接触状态 mpc控制器来
  RobotControlParameters* parameters;//参数文件内
};

/*!
 * All Estimators should inherit from this class
 *所有的估计器都应该继承这个类
 */
template <typename T>
class GenericEstimator {
 public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
};
//stateEstimatorData.legControllerData[i]


/*!
 * Main State Estimator Class 主状态估计器类
 * Contains all GenericEstimators, and can run them 包含所有通用估计器，并可以运行它们
 *也更新可视化
 将估计器添加进去统一管理
 * Also updates visualizations
 */
template <typename T>
class StateEstimatorContainer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container
   构造一个新的状态估计器容器 通过容器向每个估计器传递参数地址，之后直接更新参数
   */
  StateEstimatorContainer(CheaterState<double>* cheaterState,
                          VectorNavData* vectorNavData,
                          LegControllerData<T>* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          RobotControlParameters* parameters) {
    _data.cheaterState = cheaterState;//根据实际情况未使用 robotrunner
    _data.vectorNavData = vectorNavData;//imu数据
    _data.legControllerData = legControllerData;//腿部数据
    _data.result = stateEstimate;//估计结果
    _phase = Vec4<T>::Zero();
    _data.contactPhase = &_phase;//接触状态
    _data.parameters = parameters;
  }

  /*!
   * Run all estimators 运行所有估计
   */
  void run(CheetahVisualization* visualization = nullptr) {
    for (auto estimator : _estimators) {//遍历所有估计器
      estimator->run();//运行
    }
    if (visualization) {//可视化否
      visualization->quat = _data.result->orientation.template cast<float>();
      visualization->p = _data.result->position.template cast<float>();
      // todo contact!
    }
  }

  /*!
   * Get the result
   */
  const StateEstimate<T>& getResult() { return *_data.result; }
  StateEstimate<T> * getResultHandle() { return _data.result; }

  /*!
   * Set the contact phase//从mpc控制器来 获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
   */
  void setContactPhase(Vec4<T>& phase) { 
    *_data.contactPhase = phase; 
  }

  /*!
   * Add an estimator of the given type 添加给定类型的估计器
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() {
    auto* estimator = new EstimatorToAdd();// 实例化一个传入类型的估计器
    estimator->setData(_data);//给估计器传入数据
    estimator->setup();//估计器初始化设置
    _estimators.push_back(estimator);//推入估计器数组
  }

  /*!
   * Remove all estimators of a given type 删除给定类型的估计器
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() {
    int nRemoved = 0;
    _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator<T>* e) {
                         if (dynamic_cast<EstimatorToRemove*>(e)) {
                           delete e;
                           nRemoved++;
                           return true;
                         } else {
                           return false;
                         }
                       }),
        _estimators.end());
  }

  /*!
   * Remove all estimators 删除所有估计
   */
  void removeAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

//析构函数
  ~StateEstimatorContainer() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

 private:
  StateEstimatorData<T> _data;//状态估计数据 传入估计器用
  std::vector<GenericEstimator<T>*> _estimators;//估计器数组
  Vec4<T> _phase;
};

#endif  // PROJECT_STATEESTIMATOR_H
