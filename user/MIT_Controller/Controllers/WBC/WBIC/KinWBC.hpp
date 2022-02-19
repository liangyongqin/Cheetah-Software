#ifndef KINEMATICS_WHOLE_BODY_CONTROL
#define KINEMATICS_WHOLE_BODY_CONTROL

#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <vector>

//应该是运动学WBC
template <typename T>
class KinWBC {
 public:
  KinWBC(size_t num_qdot);
  ~KinWBC() {}

  bool FindConfiguration(const DVec<T>& curr_config,
                         const std::vector<Task<T>*>& task_list,
                         const std::vector<ContactSpec<T>*>& contact_list,
                         DVec<T>& jpos_cmd, DVec<T>& jvel_cmd);

  DMat<T> Ainv_;

 private:
  void _PseudoInverse(const DMat<T> J, DMat<T>& Jinv);
  void _BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N);

  double threshold_;//伪逆阈值
  size_t num_qdot_;//状态变量数 18 
  size_t num_act_joint_;//活动关节数 12
  DMat<T> I_mtx;//单位阵
};
#endif
