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

map<int, Item> items;                       //物品类型->物品信息

vector<Workstation*> workstations;          //工作台列表
map<int, Workstation*> item2seller;        //物品类型->卖出工作台
map<int, Workstation*> item2buyer;         //物品类型->买入工作台

vector<Robot*> robots;                      //机器人列表


//物品
struct Item{
    int type;                   //物品类型
    bitset<ITEMS_NUM> formula;  //生产配方
    bitset<WS_TYPE_NUM> need;   //需要的工作台
    float buy_price;            //买入价格
    float sell_price;           //卖出价格
    int priority;               //优先级
    int production_time;        //生产时间 (帧)

    /*物品利润*/
    float profit() const{
        return sell_price - buy_price;
    };
};

//工作台帧信息
struct ws_frame{
    int frameID;            //帧ID
    int ws_type;            //工作台类型
    float x;                //坐标x        
    float y;                //坐标y
    int remaingFrames;      //剩余生产时间
    int rawStatusCode;      //原始状态码
    int productStatus;      //产品状态
};

//机器人帧信息
struct robot_frame{
    int frameID;            //帧ID
    int workshopLocated;    //所在工作台
    int itemCarried;        //携带物品
    float timeValue;        //时间价值
    float collisionValue;   //碰撞价值
    float angSpd;           //角速度 (rad/s)
    float linSpdx;          //线速度x (m/s)
    float linSpdy;          //线速度y (m/s)
    float heading;          //方位角 (rad)
    float x;                //坐标x
    float y;                //坐标y
};

//帧信息
struct frame{
    int frameID;            //帧ID
    int crtMoney;           //当前资金
    vector<ws_frame> ws;    //工作台帧信息
    vector<robot_frame> robot;  //机器人帧信息
};

//工作台
class Workstation{
    public:
    int id;                 //工作台ID
    vec2 coordinate;        //工作台坐标
    int production_type;    //生产物品类型
    Item production_item;   //生产物品信息
    bitset<ITEMS_NUM> accept_items; //接受物品类型
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
};






