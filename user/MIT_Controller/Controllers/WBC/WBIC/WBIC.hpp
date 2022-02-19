#ifndef WHOLE_BODY_IMPULSE_CONTROL_H
#define WHOLE_BODY_IMPULSE_CONTROL_H

#include <Utilities/Utilities_print.h>
#include <Goldfarb_Optimizer/QuadProg++.hh>
#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <WBC/WBC.hpp>

template <typename T>
class WBIC_ExtraData
{
public:
  // Output
  DVec<T> _opt_result; //输出结果 加速度
  DVec<T> _qddot;      //关节加速度？
  DVec<T> _Fr;         //反力 3n*1

  // Input
  DVec<T> _W_floating; //机体状态 6*1
  DVec<T> _W_rf;       //腿反力 3n*1 n:接触腿数

  WBIC_ExtraData() {}
  ~WBIC_ExtraData() {}
};

template <typename T>
class WBIC : public WBC<T>
{
public:
  WBIC(size_t num_qdot, const std::vector<ContactSpec<T> *> *contact_list,
       const std::vector<Task<T> *> *task_list);
  virtual ~WBIC() {}

  virtual void UpdateSetting(const DMat<T> &A, const DMat<T> &Ainv,
                             const DVec<T> &cori, const DVec<T> &grav,
                             void *extra_setting = NULL);

  virtual void MakeTorque(DVec<T> &cmd, void *extra_input = NULL);

private:
  const std::vector<ContactSpec<T> *> *_contact_list;
  const std::vector<Task<T> *> *_task_list;

  void _SetEqualityConstraint(const DVec<T> &qddot);
  void _SetInEqualityConstraint();
  void _ContactBuilding();

  void _GetSolution(const DVec<T> &qddot, DVec<T> &cmd);
  void _SetCost();
  void _SetOptimizationSize();

  //下面是所有接触腿组成的矩阵 n是接触腿数
  size_t _dim_opt;     //(6+3n) Contact pt delta, First task delta, reaction force
  size_t _dim_eq_cstr; //  equality constraints 等式约束维度 6

  size_t _dim_rf; // inequality constraints 不等式约束维度 3n
  size_t _dim_Uf; // inequality constraints 好像是摩擦约束 6n

  size_t _dim_floating; //浮动基座维度 6

  WBIC_ExtraData<T> *_data;

  GolDIdnani::GVect<double> z;//求的解
  // Cost
  GolDIdnani::GMatr<double> G;  //qp问题矩阵 (6+3n)*(6+3n)
  GolDIdnani::GVect<double> g0; //(6+3n)

  // Equality
  GolDIdnani::GMatr<double> CE;  //等式约束矩阵 //(6+3n)*6
  GolDIdnani::GVect<double> ce0; //等式约束值 6

  // Inequality
  GolDIdnani::GMatr<double> CI;  //不等式约束矩阵 (6+3n)*6n
  GolDIdnani::GVect<double> ci0; //不等式约束值 6n
  //动态 感觉相当于转置
  DMat<T> _dyn_CE;  //等式约束矩阵  //6*(6+3n)
  DVec<T> _dyn_ce0; //等式约束值    //6
  DMat<T> _dyn_CI;  //不等式约束矩阵//6n*(6+3n)
  DVec<T> _dyn_ci0; //不等式约束值  //6n

  DMat<T> _eye;          //单位阵 18*18
  DMat<T> _eye_floating; //单位阵 6*6

  DMat<T> _S_delta;
  DMat<T> _Uf;         //摩擦约束 6n*3n 对角线填充uf(6*3)
  DVec<T> _Uf_ieq_vec; //摩擦约束向量 6n*1 排成一列

  DMat<T> _Jc;        //contact jacobian 3n*18 接触雅可比
  DVec<T> _JcDotQdot; //Jc_dot*q_dot 3n*1
  DVec<T> _Fr_des;    // 期望力 3n*1 从mpc来

  DMat<T> _B;
  DVec<T> _c;
  DVec<T> task_cmd_;
};

#endif
