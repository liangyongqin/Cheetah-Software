#include "WBIC.hpp"
#include <Utilities/Timer.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

//num_qdot=18
template <typename T>
WBIC<T>::WBIC(size_t num_qdot, const std::vector<ContactSpec<T> *> *contact_list,
              const std::vector<Task<T> *> *task_list)
    : WBC<T>(num_qdot), _dim_floating(6)
{
  _contact_list = contact_list;
  _task_list = task_list;

  _eye = DMat<T>::Identity(WB::num_qdot_, WB::num_qdot_);
  _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating);
}

template <typename T>
void WBIC<T>::MakeTorque(DVec<T> &cmd, void *extra_input)
{
  if (!WB::b_updatesetting_)
  {
    printf("[Wanning] WBIC setting is not done\n");
  }
  //求WBIC论文 式（16）之后的
  if (extra_input)
    _data = static_cast<WBIC_ExtraData<T> *>(extra_input);

  // resize G, g0, CE, ce0, CI, ci0
  _SetOptimizationSize();
  //设置G矩阵
  _SetCost();


//支撑腿最高优先级，摆动腿投影到支撑腿零空间运行
  DVec<T> qddot_pre;
  DMat<T> JcBar; //动态一致伪逆
  DMat<T> Npre;
//首要任务 接触腿力控 即输出加速度
  if (_dim_rf > 0)
  {
    // Contact Setting 接触设置
    _ContactBuilding();

    // Set inequality constraints 设置不等式约束
    _SetInEqualityConstraint();

    WB::_WeightedInverse(_Jc, WB::Ainv_, JcBar); //求动态一致伪逆 WBC-式(23)
    qddot_pre = JcBar * (-_JcDotQdot);           //式21-2
    Npre = _eye - JcBar * _Jc;                   //式20-1 N0
    // pretty_print(JcBar, std::cout, "JcBar");
    // pretty_print(_JcDotQdot, std::cout, "JcDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 1");
  }
  else
  {
    qddot_pre = DVec<T>::Zero(WB::num_qdot_);
    Npre = _eye;
  }

  // Task
  Task<T> *task;
  DMat<T> Jt, JtBar, JtPre;
  DVec<T> JtDotQdot, xddot;
//次要任务  摆动腿的加速度
  for (size_t i(0); i < (*_task_list).size(); ++i)
  {
    task = (*_task_list)[i];

    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);

    JtPre = Jt * Npre;                             //式19-1 J1|pre
    WB::_WeightedInverse(JtPre, WB::Ainv_, JtBar); //式23 J1|pre^-1

    qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre); //式18 qddot_pre i=1
    Npre = Npre * (_eye - JtBar * JtPre);                      //式19-2+式20-2 N1

    // pretty_print(xddot, std::cout, "xddot");
    // pretty_print(JtDotQdot, std::cout, "JtDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 2");
    // pretty_print(Jt, std::cout, "Jt");
    // pretty_print(JtPre, std::cout, "JtPre");
    // pretty_print(JtBar, std::cout, "JtBar");
  }

  // Set equality constraints 设置等式约束
  _SetEqualityConstraint(qddot_pre);

  // printf("G:\n");
  // std::cout<<G<<std::endl;
  // printf("g0:\n");
  // std::cout<<g0<<std::endl;

  // Optimization
  // Timer timer;
  /* min 0.5 * x G x + g0 x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0
*/

  T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
  // std::cout<<"\n wbic old time: "<<timer.getMs()<<std::endl;
  (void)f;

  // pretty_print(qddot_pre, std::cout, "qddot_cmd");
  for (size_t i(0); i < _dim_floating; ++i)
    qddot_pre[i] += z[i];
  _GetSolution(qddot_pre, cmd);

  _data->_opt_result = DVec<T>(_dim_opt);
  for (size_t i(0); i < _dim_opt; ++i)
  {
    _data->_opt_result[i] = z[i];
  }

  // std::cout << "f: " << f << std::endl;
  //std::cout << "x: " << z << std::endl;

  // DVec<T> check_eq = _dyn_CE * _data->_opt_result + _dyn_ce0;
  // pretty_print(check_eq, std::cout, "equality constr");
  // std::cout << "cmd: "<<cmd<<std::endl;
  // pretty_print(qddot_pre, std::cout, "qddot_pre");
  // pretty_print(JcN, std::cout, "JcN");
  // pretty_print(Nci_, std::cout, "Nci");
  // DVec<T> eq_check = dyn_CE * data_->opt_result_;
  // pretty_print(dyn_ce0, std::cout, "dyn ce0");
  // pretty_print(eq_check, std::cout, "eq_check");

  // pretty_print(Jt, std::cout, "Jt");
  // pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
  // pretty_print(xddot, std::cout, "xddot");

  // printf("CE:\n");
  // std::cout<<CE<<std::endl;
  // printf("ce0:\n");
  // std::cout<<ce0<<std::endl;

  // printf("CI:\n");
  // std::cout<<CI<<std::endl;
  // printf("ci0:\n");
  // std::cout<<ci0<<std::endl;
}

//设置等式约束 
/* min 0.5 * x G x + g0 x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0
    Sf*(Aqddot+b+g)=Sf*Jc^T*(Fr+Fdes)
    Sf*(Aqddot+b+g-Jc^T*(Fr+Fdes))=0
    CE^T=-Sf*Jc^t
    ce0=sf*(Aqddot+b+g-Jc^T*Fdes)
    x=Fr
    */
   //_CE=[A,(-Sf*Jc^t)]^T ce0=sf*(Aqddot+b+g-Jc^T*Fdes)
   
template <typename T>
void WBIC<T>::_SetEqualityConstraint(const DVec<T> &qddot)
{
  //_dim_floating=6=_dim_eq_cstr

  
  if (_dim_rf > 0)
  {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
        WB::A_.block(0, 0, _dim_floating, _dim_floating);

    _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =
        -WB::Sv_ * _Jc.transpose();

    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _Fr_des); 
  }
  else
  {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
        WB::A_.block(0, 0, _dim_floating, _dim_floating);
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_);
  }

  for (size_t i(0); i < _dim_eq_cstr; ++i)
  {
    for (size_t j(0); j < _dim_opt; ++j)
    {
      CE[j][i] = _dyn_CE(i, j);//转置
    }
    ce0[i] = -_dyn_ce0[i];
  }
  // pretty_print(_dyn_CE, std::cout, "WBIC: CE");
  // pretty_print(_dyn_ce0, std::cout, "WBIC: ce0");
}
//设置不等式约束
/* min 0.5 * x G x + g0 x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0*/
template <typename T>
void WBIC<T>::_SetInEqualityConstraint()
{
  _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;

  _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

  for (size_t i(0); i < _dim_Uf; ++i)
  {
    for (size_t j(0); j < _dim_opt; ++j)
    {
      CI[j][i] = _dyn_CI(i, j);// 转置
    }
    ci0[i] = -_dyn_ci0[i];
  }
  // pretty_print(_dyn_CI, std::cout, "WBIC: CI");
  // pretty_print(_dyn_ci0, std::cout, "WBIC: ci0");
}

//设置接触腿相关矩阵向量
template <typename T>
void WBIC<T>::_ContactBuilding()
{
  DMat<T> Uf;         //不等式约束矩阵 6*3
  DVec<T> Uf_ieq_vec; //约束值向量 6*1
  // Initial
  DMat<T> Jc;                            //接触雅可比3*18
  DVec<T> JcDotQdot;                     //J导Q导 3*1
  size_t dim_accumul_rf, dim_accumul_uf; //维度累加变量，用来填充大矩阵的索引 rf=3 uf=6

  //获取第一条接触腿数据
  (*_contact_list)[0]->getContactJacobian(Jc);
  (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
  (*_contact_list)[0]->getRFConstraintMtx(Uf);
  (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);

  dim_accumul_rf = (*_contact_list)[0]->getDim();             //3
  dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint(); //6
  //填充整个_Jc
  _Jc.block(0, 0, dim_accumul_rf, WB::num_qdot_) = Jc;
  //填充整个_JcDotQdot
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  //填充整个_Uf
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  //填充整个_Uf_ieq_vec
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
  //填充整个_Fr_des
  _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired();

  size_t dim_new_rf, dim_new_uf; //维度 rf=3 uf=6
  //将其他腿数据填入
  for (size_t i(1); i < (*_contact_list).size(); ++i)
  {
    //获取第i+1条接触腿数据
    (*_contact_list)[i]->getContactJacobian(Jc);
    (*_contact_list)[i]->getJcDotQdot(JcDotQdot);

    dim_new_rf = (*_contact_list)[i]->getDim();             //3
    dim_new_uf = (*_contact_list)[i]->getDimRFConstraint(); //6

    // Jc append 填充整个_Jc
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_) = Jc;

    // JcDotQdot append 填充整个_JcDotQdot
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

    // Uf 填充整个_Uf 对角线填充
    (*_contact_list)[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;
    //填充成一列
    // Uf inequality vector _Uf_ieq_vec
    (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

    // Fr desired
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) =
        (*_contact_list)[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }
  //pretty_print(_Fr_des, std::cout, "[WBIC] Fr des");
  // pretty_print(_Jc, std::cout, "[WBIC] Jc");
  // pretty_print(_JcDotQdot, std::cout, "[WBIC] JcDot Qdot");
  // pretty_print(_Uf, std::cout, "[WBIC] Uf");
}

//获得解 式26
template <typename T>
void WBIC<T>::_GetSolution(const DVec<T> &qddot, DVec<T> &cmd)
{
  DVec<T> tot_tau;
  //有接触腿
  if (_dim_rf > 0)
  {
    _data->_Fr = DVec<T>(_dim_rf);
    // get Reaction forces 获得反力
    for (size_t i(0); i < _dim_rf; ++i)
      _data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i];

    tot_tau =
        WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _data->_Fr; //式26
  }
  else //无接触腿
  {
    tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_;
  }
  _data->_qddot = qddot;
  cmd = tot_tau.tail(WB::num_act_joint_);

  // Torque check
  // DVec<T> delta_tau = DVec<T>::Zero(WB::num_qdot_);
  // for(size_t i(0); i<_dim_floating; ++i) delta_tau[i] = z[i];
  // pretty_print(tot_tau, std::cout, "tot tau original");
  // tot_tau += delta_tau;
  // pretty_print(tot_tau, std::cout, "tot tau result");
  // pretty_print(qddot, std::cout, "qddot");
  // pretty_print(_data->_Fr, std::cout, "Fr");
  // pretty_print(_Fr_des, std::cout, "Fr des");
}

//设置G矩阵
template <typename T>
void WBIC<T>::_SetCost()
{
  // Set Cost
  //浮动基座部分
  size_t idx_offset(0);
  for (size_t i(0); i < _dim_floating; ++i)
  {
    G[i + idx_offset][i + idx_offset] = _data->_W_floating[i];
  }
  idx_offset += _dim_floating;
  //各腿
  for (size_t i(0); i < _dim_rf; ++i)
  {
    G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];
  }
  // pretty_print(_data->_W_floating, std::cout, "W floating");
  // pretty_print(_data->_W_rf, std::cout, "W rf");
}

//更新参数
template <typename T>
void WBIC<T>::UpdateSetting(const DMat<T> &A, const DMat<T> &Ainv,
                            const DVec<T> &cori, const DVec<T> &grav,
                            void *extra_setting)
{
  WB::A_ = A;
  WB::Ainv_ = Ainv;
  WB::cori_ = cori;
  WB::grav_ = grav;
  WB::b_updatesetting_ = true;

  (void)extra_setting;
}

//设置优化相关矩阵尺寸
template <typename T>
void WBIC<T>::_SetOptimizationSize()
{
  // Dimension 维度尺寸
  _dim_rf = 0; //3
  _dim_Uf = 0; // Dimension of inequality constraint 不等式约束维度 6

  for (size_t i(0); i < (*_contact_list).size(); ++i)
  {
    _dim_rf += (*_contact_list)[i]->getDim();             // 3
    _dim_Uf += (*_contact_list)[i]->getDimRFConstraint(); //6
  }

  _dim_opt = _dim_floating + _dim_rf; //组成的状态变量维度 机身6个+每条接触腿3个
  _dim_eq_cstr = _dim_floating;       //等式约束维度 6

  // Matrix Setting
  G.resize(0., _dim_opt, _dim_opt);      //(6+3n)*(6+3n)
  g0.resize(0., _dim_opt);               //(6+3n)
  CE.resize(0., _dim_opt, _dim_eq_cstr); //(6+3n)*6
  ce0.resize(0., _dim_eq_cstr);          //6

  // Eigen Matrix Setting
  _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt); //6*(6+3n)
  _dyn_ce0 = DVec<T>(_dim_eq_cstr);                //6
  if (_dim_rf > 0)
  {
    CI.resize(0., _dim_opt, _dim_Uf);           //(6+3n)*6n
    ci0.resize(0., _dim_Uf);                    //6n
    _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt); //6n*(6+3n)
    _dyn_ci0 = DVec<T>(_dim_Uf);                //6n

    _Jc = DMat<T>(_dim_rf, WB::num_qdot_); //3n*18
    _JcDotQdot = DVec<T>(_dim_rf);         //3n
    _Fr_des = DVec<T>(_dim_rf);            //3n

    _Uf = DMat<T>(_dim_Uf, _dim_rf); //6n*3n
    _Uf.setZero();
    _Uf_ieq_vec = DVec<T>(_dim_Uf); //6n*1
  }
  else
  {
    CI.resize(0., _dim_opt, 1);
    ci0.resize(0., 1);
  }
}

template class WBIC<double>;
template class WBIC<float>;
