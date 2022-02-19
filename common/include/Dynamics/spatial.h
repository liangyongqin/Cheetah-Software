/*! @file spatial.h
 *  @brief Utility functions for manipulating spatial quantities 用于操纵空间量的实用函数
 *
 *  This file contains functions for working with spatial vectors and
 * transformation matrices.
 * *此文件包含处理空间向量和转换矩阵的函数。
 */

#ifndef LIBBIOMIMETICS_SPATIAL_H
#define LIBBIOMIMETICS_SPATIAL_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "Math/orientation_tools.h"

namespace spatial
{
using namespace ori;
enum class JointType
{
  Prismatic,
  Revolute,
  FloatingBase,
  Nothing
};

/*!
 * Calculate the spatial coordinate transform from A to B where B is rotate by
 * theta about axis.
 * 计算从A到B的空间坐标变换，其中B绕轴旋转。关于轴的。[R,0;0,R]
 */
template <typename T>
SXform<T> spatialRotation(CoordinateAxis axis, T theta)
{
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  RotMat<T> R = coordinateRotation(axis, theta);
  SXform<T> X = SXform<T>::Zero();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomRightCorner<3, 3>() = R;
  return X;
}

/*!
 * Compute the spatial motion cross product matrix.
 * Prefer motionCrossProduct when possible.
 * 计算空间运动叉乘矩阵。如果可能的话，更喜欢运动叉乘。
 */
template <typename T>
auto motionCrossMatrix(const Eigen::MatrixBase<T> &v)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  Mat6<typename T::Scalar> m;
  m << 0, -v(2), v(1), 0, 0, 0,
      v(2), 0, -v(0), 0, 0, 0,
      -v(1), v(0), 0, 0, 0, 0,
      0, -v(5), v(4), 0, -v(2), v(1),
      v(5), 0, -v(3), v(2), 0, -v(0),
      -v(4), v(3), 0, -v(1), v(0), 0;
  return m;
}

/*!
 * Compute spatial force cross product matrix.
 * Prefer forceCrossProduct when possible
 * 计算空间力叉乘矩阵。尽可能选择力叉乘
 */
template <typename T>
auto forceCrossMatrix(const Eigen::MatrixBase<T> &v)
{
  Mat6<typename T::Scalar> f;
  f << 0, -v(2), v(1), 0, -v(5), v(4),
      v(2), 0, -v(0), v(5), 0, -v(3),
      -v(1), v(0), 0, -v(4), v(3), 0,
      0, 0, 0, 0, -v(2), v(1),
      0, 0, 0, v(2), 0, -v(0),
      0, 0, 0, -v(1), v(0), 0;
  return f;
}

/*!
 * Compute spatial motion cross product.  Faster than the matrix multiplication
 * version
 * 计算空间运动叉乘。比矩阵乘法快
 */
template <typename T>
auto motionCrossProduct(const Eigen::MatrixBase<T> &a,
                        const Eigen::MatrixBase<T> &b)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  SVec<typename T::Scalar> mv;
  mv << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2),
      a(0) * b(1) - a(1) * b(0),
      a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1),
      a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
      a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
  return mv;
}

/*!
 * Compute spatial force cross product.  Faster than the matrix multiplication
 * version
 * 计算空间力叉乘 比矩阵乘法快
 */
template <typename T>
auto forceCrossProduct(const Eigen::MatrixBase<T> &a,
                       const Eigen::MatrixBase<T> &b)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  SVec<typename T::Scalar> mv;
  mv << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4),
      b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
      b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3),
      b(5) * a(1) - b(4) * a(2), b(3) * a(2) - b(5) * a(0),
      b(4) * a(0) - b(3) * a(1);
  return mv;
}

/*!
 * Convert a spatial transform to a homogeneous coordinate transformation
 * 将空间变换转换为齐次坐标变换
 */
template <typename T>
auto sxformToHomogeneous(const Eigen::MatrixBase<T> &X)
{
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                "Must have 6x6 matrix");
  Mat4<typename T::Scalar> H = Mat4<typename T::Scalar>::Zero();
  RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
  Mat3<typename T::Scalar> skewR = X.template bottomLeftCorner<3, 3>();
  H.template topLeftCorner<3, 3>() = R;
  H.template topRightCorner<3, 1>() = matToSkewVec(skewR * R.transpose());
  H(3, 3) = 1;
  return H;
}

/*!
 * Convert a homogeneous coordinate transformation to a spatial one
 * 将齐次坐标转换为空间坐标
 */
template <typename T>
auto homogeneousToSXform(const Eigen::MatrixBase<T> &H)
{
  static_assert(T::ColsAtCompileTime == 4 && T::RowsAtCompileTime == 4,
                "Must have 4x4 matrix");
  Mat3<typename T::Scalar> R = H.template topLeftCorner<3, 3>();
  Vec3<typename T::Scalar> translate = H.template topRightCorner<3, 1>();
  Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomLeftCorner<3, 3>() = vectorToSkewMat(translate) * R;
  X.template bottomRightCorner<3, 3>() = R;
  return X;
}

/*!
 * Create spatial coordinate transformation from rotation and translation
 * 从旋转和平移创建空间坐标转换 [R,0;-Rxr,R](6x6)
 */
template <typename T, typename T2>
auto createSXform(const Eigen::MatrixBase<T> &R,
                  const Eigen::MatrixBase<T2> &r)
{
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomRightCorner<3, 3>() = R;
  X.template bottomLeftCorner<3, 3>() = -R * vectorToSkewMat(r);
  return X;
}

/*!
 * Get rotation matrix from spatial transformation
 * 从空间变换得到旋转矩阵
 */
template <typename T>
auto rotationFromSXform(const Eigen::MatrixBase<T> &X)
{
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                "Must have 6x6 matrix");
  RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
  return R;
}

/*!
 * Get translation vector from spatial transformation
 * 从空间变换得到平移向量
 */
template <typename T>
auto translationFromSXform(const Eigen::MatrixBase<T> &X)
{
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                "Must have 6x6 matrix");
  RotMat<typename T::Scalar> R = rotationFromSXform(X);
  Vec3<typename T::Scalar> r =
      -matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
  return r;
}

/*!
 * Invert a spatial transformation (much faster than matrix inverse)
 * 反转空间变换(比矩阵反变换快得多)
 */
template <typename T>
auto invertSXform(const Eigen::MatrixBase<T> &X)
{
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                "Must have 6x6 matrix");
  RotMat<typename T::Scalar> R = rotationFromSXform(X);
  Vec3<typename T::Scalar> r =
      -matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
  SXform<typename T::Scalar> Xinv = createSXform(R.transpose(), -R * r);
  return Xinv;
}

/*!
 * Compute joint motion subspace vector
 * 计算关节运动子空间向量
 */
template <typename T>
SVec<T> jointMotionSubspace(JointType joint, CoordinateAxis axis)
{
  Vec3<T> v(0, 0, 0);
  SVec<T> phi = SVec<T>::Zero();
  if (axis == CoordinateAxis::X)
    v(0) = 1;
  else if (axis == CoordinateAxis::Y)
    v(1) = 1;
  else
    v(2) = 1;

  if (joint == JointType::Prismatic)
    phi.template bottomLeftCorner<3, 1>() = v;
  else if (joint == JointType::Revolute)
    phi.template topLeftCorner<3, 1>() = v;
  else
    throw std::runtime_error("Unknown motion subspace");

  return phi;
}

/*!
 * Compute joint transformation
 * 计算关节变换
 */
template <typename T>
Mat6<T> jointXform(JointType joint, CoordinateAxis axis, T q)
{
  Mat6<T> X = Mat6<T>::Zero();
  if (joint == JointType::Revolute)
  {
    X = spatialRotation(axis, q);
  }
  else if (joint == JointType::Prismatic)
  {
    Vec3<T> v(0, 0, 0);
    if (axis == CoordinateAxis::X)
      v(0) = q;
    else if (axis == CoordinateAxis::Y)
      v(1) = q;
    else if (axis == CoordinateAxis::Z)
      v(2) = q;

    X = createSXform(RotMat<T>::Identity(), v);
  }
  else
  {
    throw std::runtime_error("Unknown joint xform\n");
  }
  return X;
}

/*!
 * Construct the rotational inertia of a uniform density box with a given mass.
 * 构造具有给定质量的均匀密度箱的转动惯量 Matrix（3*3）
 * @param mass Mass of the box 
 * @param dims Dimensions of the box 盒子尺寸
 */
template <typename T>
Mat3<typename T::Scalar> rotInertiaOfBox(typename T::Scalar mass,
                                         const Eigen::MatrixBase<T> &dims)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 vector");
  Mat3<typename T::Scalar> I =
      Mat3<typename T::Scalar>::Identity() * dims.norm() * dims.norm();
  for (int i = 0; i < 3; i++)
    I(i, i) -= dims(i) * dims(i);
  I = I * mass / 12;
  return I;
}

/*!
 * Convert from spatial velocity to linear velocity.
 * Uses spatial velocity at the given point.
 * 将空间速度转换为线速度。使用给定点的空间速度。
 */
template <typename T, typename T2>
auto spatialToLinearVelocity(const Eigen::MatrixBase<T> &v,
                             const Eigen::MatrixBase<T2> &x)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 vector");
  Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
  Vec3<typename T::Scalar> vsLin = v.template bottomLeftCorner<3, 1>();
  Vec3<typename T::Scalar> vLinear = vsLin + vsAng.cross(x);
  return vLinear;
}

/*!
 * Convert from spatial velocity to angular velocity.
 * 将空间速度转换为角速度。
 */
template <typename T>
auto spatialToAngularVelocity(const Eigen::MatrixBase<T> &v)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
  return vsAng;
}

/*!
 * Compute the classical lienear accleeration of a frame given its spatial
 * acceleration and velocity
 * 在给定空间加速度和速度的情况下计算经典线性加速度
 */
template <typename T, typename T2>
auto spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                 const Eigen::MatrixBase<T2> &v)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6,
                "Must have 6x1 vector");

  Vec3<typename T::Scalar> acc;
  // classical accleration = spatial linear acc + omega x v
  acc = a.template tail<3>() + v.template head<3>().cross(v.template tail<3>());
  return acc;
}

/*!
 * Compute the classical lienear acceleration of a frame given its spatial
 * acceleration and velocity
 * 在给定空间加速度和速度的情况下计算经典线性加速度
 */
template <typename T, typename T2, typename T3>
auto spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                 const Eigen::MatrixBase<T2> &v,
                                 const Eigen::MatrixBase<T3> &x)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6,
                "Must have 6x1 vector");
  static_assert(T3::ColsAtCompileTime == 1 && T3::RowsAtCompileTime == 3,
                "Must have 3x1 vector");

  Vec3<typename T::Scalar> alin_x = spatialToLinearVelocity(a, x);
  Vec3<typename T::Scalar> vlin_x = spatialToLinearVelocity(v, x);

  // classical accleration = spatial linear acc + omega x v
  Vec3<typename T::Scalar> acc = alin_x + v.template head<3>().cross(vlin_x);
  return acc;
}

/*!
 * Apply spatial transformation to a point.
 * 对点应用空间变换。
 */
template <typename T, typename T2>
auto sXFormPoint(const Eigen::MatrixBase<T> &X,
                 const Eigen::MatrixBase<T2> &p)
{
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                "Must have 6x6 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 vector");

  Mat3<typename T::Scalar> R = rotationFromSXform(X);
  Vec3<typename T::Scalar> r = translationFromSXform(X);
  Vec3<typename T::Scalar> Xp = R * (p - r);
  return Xp;
}

/*!
 * Convert a force at a point to a spatial force
 * 将一点的力转换为空间力
 * @param f : force 3*1
 * @param p : point 3*1
 */
template <typename T, typename T2>
auto forceToSpatialForce(const Eigen::MatrixBase<T> &f,
                         const Eigen::MatrixBase<T2> &p)
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 vector");
  SVec<typename T::Scalar> fs;
  fs.template topLeftCorner<3, 1>() = p.cross(f);
  fs.template bottomLeftCorner<3, 1>() = f;
  return fs;
}

} // namespace spatial

#endif // LIBBIOMIMETICS_SPATIAL_H
