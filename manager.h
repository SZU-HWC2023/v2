//调度
#include <map>
#include "class.h"

class Manager{
public:
    Manager()=default;
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
