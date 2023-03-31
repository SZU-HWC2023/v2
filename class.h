//定义各种类

#include <iostream>
#include <vector>
#include <bitset>
#include <set>
#include <map>
#include <cmath>
#include <algorithm>

#ifndef UTILS_H
#include "utils.h"
#endif

#ifndef SETTING_H
#include "setting.h"
#endif

using namespace std;

struct Item;
class Workstation;
class Robot;

class Map;

class RVO;
class AStar;

struct VW;
struct DW;
struct DWA_state;
class DWA;


extern Map g_Map;                                   //地图类

extern int g_ws_requirement[WS_TYPE_NUM+1];             //工作台需要的原材料材料   全局变量

extern multimap<int, Workstation*> g_item_from_ws;        //物品类型->提供该物品的工作台    全局变量
extern multimap<int, Workstation*> g_item_to_ws;         //物品类型->需要该物品的工作台   全局变量

extern map<int,Item> g_items;                       //物品类型->物品信息   全局变量
extern vector<Workstation*> g_workstations;          //工作台列表           全局变量
extern vector<Robot*> g_robots;                      //机器人列表            全局变量



// 工作台需要的原材料材料
const set<int> WORKERSTATION_TO_RECYCLE[10] = {
        {},                   // 工作台0不回收
        {},                   // 工作台1不回收
        {},                   // 工作台2不回收
        {},                   // 工作台3不回收
        {1, 2},               // 工作台4回收1、2
        {1, 3},               // 工作台5回收1、3
        {2, 3},               // 工作台6回收2、3
        {4, 5, 6},            // 工作台7回收4、5、6
        {7},                  // 工作台8回收7
        {1, 2, 3, 4, 5, 6, 7} // 工作台9回收1-7
};




//实现于 items.cpp
//物品
struct Item{
    int type;                   //物品类型
    bitset<ITEMS_NUM> formula;  //生产配方
    float buy_price;            //买入价格
    float sell_price;           //卖出价格
    int priority;               //优先级
    int production_time;        //生产时间 (帧)

    /*物品利润*/
    float profit() const{
        return sell_price - buy_price;
    };
};

void init_items();

//工作台帧信息
struct ws_frame{
    int ws_type;            //工作台类型
    float x;                //坐标x        
    float y;                //坐标y
    int remaing_frames;      //剩余生产时间
    int raw_status_code;      //原材料状态码
    int product_status;      //产品状态
};

//机器人帧信息
struct robot_frame{
    int workshop_located;    //所在工作台
    int item_carried;        //携带物品
    float time_value;        //时间价值
    float collision_value;   //碰撞价值
    float ang_spd;           //角速度 (rad/s)
    float lin_spd_x;          //线速度x (m/s)
    float lin_spd_y;          //线速度y (m/s)
    float heading;          //方位角 (rad)
    float x;                //坐标x
    float y;                //坐标y
};

// 实现于io.cpp
//读地图和读帧的相关函数
bool read_map();
bool readUntilOK();

// 实现于workstation.cpp
//工作台
class Workstation{
    public:
    int id;                 //工作台ID
    vec2 coordinate;        //工作台坐标
    int type;               //平台类型
    int remaing_frames;      //-1:缺少材料  0:阻塞    >0:剩余帧数
    int product_status;      //产品状态
    Item production_item;   //生产物品信息
    int raw_status_code;      //原材料状态码    每一帧实时变化
    bitset<ITEMS_NUM> accept_items; //接受物品类型    常量

    set<int> need;            // 还需要物品的类型
    // 动态维护的量
    map<int, int> locked;          // 已经被锁定的物品id(包括生产的物品)  被锁物品编号 锁定的机器人编号

    bool production_needed(int production_type);
    bool production_locked(int production_type);

    bool can_production_recycle(int production_type);
    bool can_production_sell();

    void rel_locked_production(int production_type);
    void add_locked_production(int production_type, int robot_id);

    void putIn(int production_type);
    map<int, int> getLocked();
    bool haveRawLocked();
    void changeLocked(int production_type, int robot_id);

    int getMissingNum();
    int getLeftTime();
    int getCycleTime();
    int getWeight();

    int getBuyPrice();
    int getSellPrice();
    int getRank();
    double getProfit();


    Workstation(int workstationID, int type, float x, float y);
    void update(ws_frame f);
};

// 实现于robot.cpp
//机器人
class Robot{
    public:
    int id;                 //机器人ID
    vec2 coordinate;        //机器人坐标
    float heading;          //机器人方位角
    float angular_speed;    //机器人角速度
    vec2 linear_speed;      //机器人线速度
    int item_carried;       //机器人携带物品
    int workshop_located;   //机器人所在工作台

    float crt_radius;       //当前半径 (m)
    float crt_mass;         //当前质量 (kg)
    float crt_lin_acc;      //当前线加速度 (m/s^2)
    float crt_ang_acc;      //当前角加速度 (rad/s^2)

    // 需要维护的量 1
    tuple<Workstation*, int> action = {NULL, -1};         // 奔向的工作台指针 物品编号(1-7)
    int next_worker_id = -1;                // -1表示下一个工作台未指定 注意对这个工作台不会进行加锁操作

    vector<Robot*> other_robots;    //其他机器人列表

    vector<DWA_state> trajectory;   //轨迹
    DWA* dwa;                       //DWA指针


    Robot(int robotID, float x, float y);
    void update(robot_frame f);

    void initOtherRobot();

    void forward(float tgtSpd);
    void rotate(float tgtAngSpd);
    void buy();
    void sell();
    void destory();

    bool isAble2Brake(float brake_dist);
    void move2ws(Workstation* ws);


    void resetAction();                     //重置机器人的动作
    const tuple<Workstation*, int> getAction();
    void setAction(tuple<Workstation*, int> action);
    
    void setNextWorkerId(int id);
    int getNextWorkerId();
};

//实现于map.cpp
//地图类
class Map{
    public:
    //地图数组，'.'为可通行区域，'#'为障碍物
    array<array<char, MAP_TRUE_SIZE>, MAP_TRUE_SIZE> map;


    Map();

    bool isObstacle(vec2 pos);
    bool isObstacle(vec2_int pos);
    float dist2Obstacle(vec2 pos);
    bool isCollide2Obstacle(Robot* robot);

};


// 实现于astar.cpp
typedef struct Point{
    int x;
    int y;
    float cost;
    struct Point *parent_node;
    Point(int x,int y, float cost,Point* parent_node){
        this->x = x;
        this->y = y;
        this->cost = cost;
        this->parent_node = parent_node;
    }
}Point;

class AStar{
public:
    vector<tuple<int,int, float >> motion;
    AStar(){
        this->motion = this->get_motion_model();
    }
    vector<Point*> planning(int sx,int sy,int gx,int gy);

    vector<Point*> calc_final_path(Point* goal_node,map<tuple<int,int>,Point*> &closed_map);
    //判断下标是否合法
    bool verify(Point* p);
    tuple<int,int> getIndex(Point* p);
    float calc_heuristic(Point* a, Point *b);
    vector<tuple<int,int, float >> get_motion_model();

};

void test_astar();

//线速度-角速度对
struct VW{
    float v;   
    float w;
};

struct DW{
    float v_min;
    float v_max;
    float ang_v_min;
    float ang_v_max;

    float v_step(int v_samples){
        return (v_max - v_min) / v_samples;
    }

    float ang_v_step(int ang_v_samples){
        return (ang_v_max - ang_v_min) / ang_v_samples;
    }
};

struct DWA_state{
    vec2 pos; //位置 m [0,1]
    float heading; //航向角 rad [2]
    vec2 linSpd; //线速度 m/s .len()->[3]
    float angSpd; //角速度 rad/s [4]
    Robot* robot;   //指向机器人

    DWA_state(Robot* robot){
        this->pos = robot->coordinate;
        this->heading = robot->heading;
        this->linSpd = robot->linear_speed;
        this->angSpd = robot->angular_speed;
        this->robot = robot;
    };

    DW calcDW(float dt = FRAME_INTERVAL*5){
        DW vs = {0, MAX_FORWARD_SPD, -MAX_ANGULAR_SPD, MAX_ANGULAR_SPD};
        float linAcc = this->robot->crt_lin_acc, angAcc = this->robot->crt_ang_acc;
        DW vd = {this->linSpd.len() - linAcc * dt,
                 this->linSpd.len() + linAcc * dt,
                 this->angSpd - angAcc * dt,
                 this->angSpd + angAcc * dt};

        return {max(vs.v_min, vd.v_min), min(vs.v_max, vd.v_max),
                max(vs.ang_v_min, vd.ang_v_min), min(vs.ang_v_max, vd.ang_v_max)};
    }

    void move_dt(VW vw, float dt){
        this->heading += vw.w * dt;
        this->heading = clampHDG(this->heading);
        vec2 delta_pos = {cos(this->heading) * vw.v * dt, sin(this->heading) * vw.v * dt};
        this->pos += delta_pos;
        this->linSpd = {vw.v * cos(this->heading), vw.v * sin(this->heading)};
        this->angSpd = vw.w;
    }

    vector<DWA_state> calcTrajectory(VW vw, float pred_t, float dt = FRAME_INTERVAL*5){
        vector<DWA_state> trajectory;
        trajectory.push_back(*this);
        for(float t = dt; t < pred_t; t += dt){
            this->move_dt(vw,dt);
            trajectory.push_back(*this);
        }
        return trajectory;
    }
};

class DWA{
    public:
    Robot* robot;
    
    static constexpr float alpha = 1;
    static constexpr float beta = 6.0;
    static constexpr float gamma = 1;
    static constexpr float pred_t = PRED_T;
    // static constexpr float ang_spd_intv = 0.05;
    // static constexpr float lin_spd_intv = 0.1;
    static constexpr float v_samples = 5;
    static constexpr float ang_v_samples = 30;

    DWA(Robot* robot);

    float tgt_cost(vector<DWA_state> trajectory, vec2 tgt_pos);
    float obs_cost(vector<DWA_state> trajectory);
    float vel_cost(vector<DWA_state> trajectory, vec2 tgt_pos);

    float calc_cost(VW vw, vec2 tgt_pos, bool log=false);

    VW find_vw(vec2 tgt_pos);
};

