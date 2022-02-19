/*MIT-Cheetah Software
**             Email:@qq.com   QQ:1370780559
**---------------------------------------------------------
**  Description: 此文件注释由本人完成，仅为个人理解,本人水平有限，还请见谅
**  interpreter     : NaCl
*/

主要对MPC控制的移动状态流程进行了注释

对下列程序进行了部分注释：
user\MIT_Controller\main.cpp//启动文件
robot\main_helper//启动文件

HardwareBridge//硬件接口 启动控制器 通信等
RobotRunner//运行器  用来运行状态估计 控制器等 是一个周期任务:
PeriodicTask//周期任务

StateEstimator//状态估计父类
ContactEstimator//接触状态估计 目前版本代码未使用论文中算法 只是直接传递数据
PositionVelocityEstimator//位置估计 卡尔曼滤波 
orientationEstimator //角度估计 
contactEstimator //接触状态估计 目前为使用到 当前功能为将步态时间表传递

RobotController//控制器父类 仅用来继承创造接口
VisionMPC//老版本MPC算法 主要部分和convexMPC相同
convexMPC//实际使用的mpc算法
FootSwingTrajectory//足端轨迹
BalanceController//为QP优化实现虚拟力分配，实际好像未使用？ 在FSM_State<T>* invalid;中使用，这个被设为空指针了
LegController//腿部控制 包括单腿雅可比 运动学 向控制板收发命令和参数  所有的量都在“腿坐标系”中，与身体坐标系有相同的方向

FSM//有限状态机 根据状态不同使用不同的控制器
//ControlFSM.cpp->runFSM()//状态机切换状态代码所在NORMAL, TRANSITIONING, ESTOP, EDAMP为状态机的状态切换用状态
/*struct FSM_StatesList {
  FSM_State<T>* invalid; // 空 
  FSM_State_Passive<T>* passive;//无操作
  FSM_State_JointPD<T>* jointPD;//独立关节控制
  FSM_State_ImpedanceControl<T>* impedanceControl;//独立控制 应该时直角坐标下
  FSM_State_StandUp<T>* standUp;//站起
  FSM_State_BalanceStand<T>* balanceStand;//平衡站立
  FSM_State_Locomotion<T>* locomotion;//移动
  FSM_State_RecoveryStand<T>* recoveryStand;//恢复站立
  FSM_State_Vision<T>* vision;
  FSM_State_BackFlip<T>* backflip;//后空翻
  FSM_State_FrontJump<T>* frontJump;//前跳
};
*/
这些为控制状态，状态机切换的目标状态 即实现的功能
状态机通过遥控在这些状态间切换，猜测应包含大量调试用状态

WBC部分：（仅个人片面理解，这块我也不太清楚）请参照论文 ：
“Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control”

WBC控制器使用零空间投影方法 （null_space_base） 实现任务优先级控制 
（此方法本人未了解透彻，大概参考一篇足球机器人的论文，有理解的大佬请联系我！！！）
个人理解最高优先级任务为接触脚控制，其他摆动脚为次一级任务
支撑腿为最优先任务，
第一条摆动腿在支撑腿任务的零空间执行，第二条摆动腿在第一条摆动腿的零空间执行，
其他摆动腿依次类推？？（这块不确定）
可能其他腿也在支撑腿的零空间执行，即几个摆动腿是同级？？


WBC部分主要看这几个文件，未注即未使用代码（仅在locomotion状态）
user\MIT_Controller\Controllers\WBC\WBIC中全部//算法关键
user\MIT_Controller\Controllers\WBC\ContactSpec.hpp//父类
user\MIT_Controller\Controllers\WBC\Task.hpp//父类
user\MIT_Controller\Controllers\WBC\WBC.hpp//父类
user\MIT_Controller\Controllers\WBC_Ctrl\ContactSet\SingleContact.cpp&hpp//子类 算法关键
user\MIT_Controller\Controllers\WBC_Ctrl\LocomotionCtrl\LocomotionCtrl.cpp&hpp//子类 算法关键
user\MIT_Controller\Controllers\WBC_Ctrl\TaskSet\LinkPosTask.cpp&hpp//子类 算法关键
user\MIT_Controller\Controllers\WBC_Ctrl\WBC_Ctrl.cpp&hpp//父类

父类   子类
Task->LinkPosTask
ContactSpec->SingleContact
WBC-> WBIC
WBC_Ctrl->LocomotionCtrl

LocomotionCtrl主要用来启动和传出传入数据
前者是进行任务更新 启动SingleContact，LinkPosTask，WBIC，kinWBC

移动部分启动流程：
fsm_locomotion(将MPC产生的足端坐标，力等参数传入)->LocomotionCtrl->LinkPosTask&SingleContact&kinWBC&WBIC&FloatingBaseModel


WBC控制器主要分 ：
kinWBC：运动学WBC 将摆动腿按优先级处理并计算出控制量 主要实现论文式16 ，17，不处理支撑腿控制量
WBIC :动力学WBC部分，控制支撑腿 //主要实现式25及前面加速度公式 控制各腿加速度 支撑腿最高优先级 



/*
user\main.cpp是启动文件传入控制器对象MIT_Controller->启动main_helper->
启动hardwarebridge启动周期任务控制器PeriodicTask->
添加robotrunner到任务管理器->robotrunner启动MIT_Controller,实例化需要的类传入MIT_Controller->
MIT_Controller启动gait schedule，DesiredStateCommand，ControlFSM->ControlFSM进行状态切换

*/

注释思路主要来源于文件内附的论文外加个人猜测，如有错误，请联系我