#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include <cppTypes.h>

#define Contact ContactSpec<T>

template <typename T>
class ContactSpec
{
public:
  ContactSpec(size_t dim) : dim_contact_(dim), b_set_contact_(false) //传入dim=3
  {
    idx_Fz_ = dim - 1; // because normally (tau_x,y,z , linear_x,y,z) fz索引 2
    Fr_des_ = DVec<T>::Zero(dim);
  }
  virtual ~ContactSpec() {}

  size_t getDim() const { return dim_contact_; }           //获得维度 （3）
  size_t getDimRFConstraint() const { return Uf_.rows(); } //获得约束维度 6
  size_t getFzIndex() const { return idx_Fz_; }            //获得索引 2

  void getContactJacobian(DMat<T> &Jc) { Jc = Jc_; }                //获得更新后的
  void getJcDotQdot(DVec<T> &JcDotQdot) { JcDotQdot = JcDotQdot_; } //获得更新后的
  void UnsetContact() { b_set_contact_ = false; }

  void getRFConstraintMtx(DMat<T> &Uf) { Uf = Uf_; }                //获得摩擦约束矩阵 6*3
  void getRFConstraintVec(DVec<T> &ieq_vec) { ieq_vec = ieq_vec_; } //获得极值约束向量
  const DVec<T> &getRFDesired() { return Fr_des_; }                 //获得期望力
  void setRFDesired(const DVec<T> &Fr_des) { Fr_des_ = Fr_des; }    //设置期望力

  //更新 Jc JcDotQDot 约束等
  bool UpdateContactSpec()
  {
    _UpdateJc();               //更新Jc
    _UpdateJcDotQdot();        //更新JcDotQdot
    _UpdateUf();               //是否更新摩擦,z极限力的不等式约束
    _UpdateInequalityVector(); //更新不等式约束 摩擦力，z极限力
    b_set_contact_ = true;
    return true;
  }

protected:
  virtual bool _UpdateJc() = 0;               //更新Jc
  virtual bool _UpdateJcDotQdot() = 0;        //更新JcDotQdot
  virtual bool _UpdateUf() = 0;               //是否更新摩擦,z极限力的不等式约束
  virtual bool _UpdateInequalityVector() = 0; //更新不等式约束 摩擦力，z极限力

  int idx_Fz_;      //fz索引
  DMat<T> Uf_;      //摩擦约束 即Fz极值约束 6*3
  DVec<T> ieq_vec_; //力极值[0，0，0，0，0,-max] 6*1
  DVec<T> Fr_des_;  //期望力

  DMat<T> Jc_;         //3*18
  DVec<T> JcDotQdot_;  //3*1
  size_t dim_contact_; //3
  bool b_set_contact_;
};
#endif
