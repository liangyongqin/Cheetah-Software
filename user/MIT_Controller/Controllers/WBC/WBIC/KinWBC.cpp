#include "KinWBC.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/pseudoInverse.h>

//num_qdot=18  _kin_wbc = new KinWBC<T>(cheetah::dim_config);
template <typename T>
KinWBC<T>::KinWBC(size_t num_qdot)
    : threshold_(0.001), num_qdot_(num_qdot), num_act_joint_(num_qdot - 6) {
  I_mtx = DMat<T>::Identity(num_qdot_, num_qdot_);
}

//找到参数
template <typename T>
bool KinWBC<T>::FindConfiguration(
    const DVec<T>& curr_config, const std::vector<Task<T>*>& task_list,
    const std::vector<ContactSpec<T>*>& contact_list, DVec<T>& jpos_cmd,
    DVec<T>& jvel_cmd) {

  // Contact Jacobian Setup
  DMat<T> Nc(num_qdot_, num_qdot_); //18*18
  Nc.setIdentity();

//接触队列非空有接触的脚
  if(contact_list.size() > 0){
    DMat<T> Jc, Jc_i;//Jc 3n*18 Jc_i 3*18
    //接触雅可比矩阵第一个接触腿
    contact_list[0]->getContactJacobian(Jc);
    //获取维度
    size_t num_rows = Jc.rows();//3
    //遍历除第一个之外的接触腿
    for (size_t i(1); i < contact_list.size(); ++i) {
      //接触雅可比矩阵
      contact_list[i]->getContactJacobian(Jc_i);
      //获取维度
      size_t num_new_rows = Jc_i.rows();
      //调整矩阵尺寸为 所有接触腿行数*18 并填充数据为每个接触腿的矩阵
      Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
      Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
      num_rows += num_new_rows;
      //JC填充后[jc0.jc1,jc2...]^t (3*n)*18 n为接触腿数 大概？
    }

    // Projection Matrix 投影矩阵 式20-1 投影到最高优先级需要
    //个人理解 最高优先级任务为接触腿的控制任务，其他摆动腿的控制任务依次为次要任务，投影到零空间进行
    _BuildProjectionMatrix(Jc, Nc);
  }

  // First Task 
  DVec<T> delta_q, qdot;//delta_q 18*1  qdot 18*1
  DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;//Jt 3*18 , JtPre 3*18,  JtPre_pinv 18*3, N_nx, N_pre

  //第一个摆动腿
  Task<T>* task = task_list[0];
  //获得其雅可比矩阵
  task->getTaskJacobian(Jt);
  //式19-1
  JtPre = Jt * Nc;
  //式16~17中的逆
  _PseudoInverse(JtPre, JtPre_pinv);
  //式16，17下i=1任务
  //delta_q 1
  delta_q = JtPre_pinv * (task->getPosError());
  qdot = JtPre_pinv * (task->getDesVel());
  //delta_q 1
  DVec<T> prev_delta_q = delta_q;
  DVec<T> prev_qdot = qdot;
  //式20-2 i=2
  //N1|0
  _BuildProjectionMatrix(JtPre, N_nx);
  //式19-2
  //N(2-1)=N0*N1|0
  N_pre = Nc * N_nx;
  //其他摆动腿控制任务
  for (size_t i(1); i < task_list.size(); ++i) {

    task = task_list[i];

    task->getTaskJacobian(Jt);
    //式19-1
    //J2|pre=J2*N(2-1)
    JtPre = Jt * N_pre;
    //式16~17中的逆
    _PseudoInverse(JtPre, JtPre_pinv);
    //式16，17下i=2...任务
    delta_q =
        prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
    qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

    // For the next task
    //为次一级任务准备
    //N2|1  
    _BuildProjectionMatrix(JtPre, N_nx);
    //N(3-1)=N0*N1|0*N2|1  
    N_pre *= N_nx;

    prev_delta_q = delta_q;
    prev_qdot = qdot;
  }

  //关节输出角度和速度
  for (size_t i(0); i < num_act_joint_; ++i) {
    jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
    jvel_cmd[i] = qdot[i + 6];
  }
  return true;
}
// Projection Matrix 投影矩阵 N=I*J^-1*J
template <typename T>
void KinWBC<T>::_BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N) {
  DMat<T> J_pinv;
  _PseudoInverse(J, J_pinv);
  N = I_mtx - J_pinv * J;
}
//伪逆
template <typename T>
void KinWBC<T>::_PseudoInverse(const DMat<T> J, DMat<T>& Jinv) {
  pseudoInverse(J, threshold_, Jinv);
}

template class KinWBC<float>;
template class KinWBC<double>;
