//定义常量

#define _USE_MATH_DEFINES
#define SETTING_H
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <queue>
#include <array>
#include <list>
using namespace std;
// 预定义参数 规范应该写在其余头文件中
#define MAX 0x3f3f3f
#define MAP_SIZE 50                 // 地图大小 (m)
#define MAP_TRUE_SIZE 100           // 地图大小 （100 * 100 的字符矩阵）
#define MAX_WORKER_NUM 50           // 最大工作台数量
#define ROBOT_NUM 4                 // 机器人数量
#define FPS 50                      // 帧率
#define FRAME_INTERVAL 1.0 / FPS    // 帧间隔
#define INIT_MONEY 200000           // 初始资金
#define ITEMS_NUM 8                 // 物品种类数量 ，包括没有物品
#define WS_TYPE_NUM 9               // 工作台种类数量
#define ROBOT_WORK_DISTANCE 0.4     // 工作台交互距离 (m)
#define ROBOT_NORM_RADIUS 0.45      // 机器人正常半径 (m)        
#define ROBOT_CARRY_RADIUS 0.53     // 机器人携带半径 (m)
#define ROBOT_DENSITY 20            // 机器人密度 (kg/m^2)
#define MAX_FORWARD_SPD 6           // 机器人最大前进速度 (m/s)
#define MAX_BACKWARD_SPD -2         // 机器人最大后退速度 (m/s)
#define MAX_ANGULAR_SPD M_PI        // 机器人最大角速度 (rad/s)
#define MAX_TRACTION 250            // 机器人最大牵引力 (N)
#define MAX_TORQUE 50               // 机器人最大扭矩 (N*m)
#define MAX_FRAME 15000              // 最大帧数
#define MIN_ANGLE 0.08              // 最小角速度
// 调试输出
#define DEBUG

#define PRED_T 0.5

//需要多线程计算时，定义一下这个宏
// #define MUTI_THREAD
#define THREAD_NUM 2
static float ad = 4.01;
static float ai = 0.02;
static float si = 0.05;
static float ta = 6;