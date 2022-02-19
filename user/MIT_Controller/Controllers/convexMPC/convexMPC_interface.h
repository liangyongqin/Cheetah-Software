#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

//#include "common_types.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup//mpc参数 
{
  float dt;//时间
  float mu;//摩擦系数
  float f_max;//最大力
  int horizon;//分段数
};

struct update_data_t
{
  float p[3];//位置
  float v[3];//速度
  float q[4];//四元数
  float w[3];//角速度
  float r[12];//四脚从COM指向足端向量
  float yaw;//偏航角
  float weights[12];//权重 优化参数...
  float traj[12*K_MAX_GAIT_SEGMENTS];//参考轨迹（12）--（欧拉角（3x1），机身位置（3x1），机身角速度（3*1），机身速度（3*1）） *片段数
  float alpha;//mpc参数 K 式（31）
  unsigned char gait[K_MAX_GAIT_SEGMENTS];//步态 脚是否悬空
  unsigned char hack_pad[1000];
  int max_iterations;//最大迭代次数
  double rho, sigma, solver_alpha, terminate;//jcqp使用参数
  int use_jcqp;//使用解法jcqp与否
  float x_drag;////z轴方向加速度受x轴方向速度的影响程度
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

void update_x_drag(float x_drag);
#endif