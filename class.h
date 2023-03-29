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
class RVO;
class AStar;
extern char g_map[MAP_TRUE_SIZE][MAP_TRUE_SIZE];    //地图的字符矩阵

extern int g_ws_requirement[WS_TYPE_NUM+1];             //工作台需要的原材料材料   全局变量
extern map<int,Item> g_items;                       //物品类型->物品信息   全局变量

extern vector<Workstation*> g_workstations;          //工作台列表           全局变量
extern multimap<int, Workstation*> g_item_from_ws;        //物品类型->提供该物品的工作台    全局变量
extern multimap<int, Workstation*> g_item_to_ws;         //物品类型->需要该物品的工作台   全局变量

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
    tuple<int, int> action = {-1, -1};         // 奔向的工作台编号(<50) 物品编号(1-7)
    int next_worker_id = -1;                // -1表示下一个工作台未指定 注意对这个工作台不会进行加锁操作

    vector<Robot*> other_robots;    //其他机器人列表


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
    const tuple<int, int> getAction();
    void setAction(tuple<int, int> action);
    void setNextWorkerId(int id);
    int getNextWorkerId();
};
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
//读地图和读帧的相关函数
bool read_map();
bool readUntilOK();
