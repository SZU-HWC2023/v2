//调度
#include <map>
#include "class.h"

class Manager{
public:
    Manager();
    map<int, int> historyGetMap;       // 存储历史取货信息--------要去哪个工作台取东西根据这个设置权重(平衡历史买卖)
    map<tuple<int, int>, int> historyFillMap;   // 历史填入次数  {工作台编号 原材料编号} 次数------卖东西的时候卖给哪个平台根据这个设置权重(平衡历史买卖)
    void mainHistoryGetMap(Workstation *w);
    void mainHistoryFillMap(int w_type, int item);
    int getMinimumFromMap(map<tuple<int, int>, int> dict);
    int getMinimumFromMap(map<int, int> dict);
    void handleFps(int frame_id);
    void handleTask(Robot* r);       
    void assignTask(int frame_id, Robot* r, queue<int> robot_ids);   
    void handleGetTask(Robot* r);
    void handleSetTask(Robot* r);   
    virtual void assignGetTask(int frame_id, Robot* r,queue<int> robot_ids)=0;   // 纯虚函数，针对不同地图进行不同实现
    virtual void assignSetTask(int frame_id,  Robot* r)=0;                         // 纯虚函数，针对不同地图进行不同实现
};
class Map1:public Manager{
public:
    // 放物品调度
    void assignSetTask(int frame_id, Robot* r);
    // 取物品调度
    void assignGetTask(int frame_id, Robot* r, queue<int> robot_ids);
};
class Map2:public Manager{
public:
    // 放物品调度
    void assignSetTask(int frame_id, Robot* r);
    // 取物品调度
    void assignGetTask(int frame_id, Robot* r, queue<int> robot_ids);
};
class Map3:public Manager{
public:
    // 放物品调度
    void assignSetTask(int frame_id, Robot* r);
    // 取物品调度
    void assignGetTask(int frame_id, Robot* r, queue<int> robot_ids);
};
class Map4:public Manager{
public:
    // 放物品调度
    void assignSetTask(int frame_id, Robot* r);
    // 取物品调度
    void assignGetTask(int frame_id, Robot* r, queue<int> robot_ids);
};
