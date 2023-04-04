#include "manager.h"

Manager::Manager(){
    // 初始化get
    historyGetMap[1] = 0;
    historyGetMap[2] = 0;
    historyGetMap[3] = 0;
    historyGetMap[4] = 0;
    historyGetMap[5] = 0;
    historyGetMap[6] = 0;
    historyGetMap[7] = 0;
    // 初始化 fill
    historyFillMap[{4, 1}] = 0;
    historyFillMap[{4, 2}] = 0;
    historyFillMap[{5, 1}] = 0;
    historyFillMap[{5, 3}] = 0;
    historyFillMap[{6, 2}] = 0;
    historyFillMap[{6, 3}] = 0;
}
int Manager::getMinimumFromMap(map<tuple<int, int>, int> dict){
    int res = MAX;
    if(dict.size() == 0) return -1;
    for(auto d:dict){
        if(d.second<res) res = d.second;
    }
    return res-1;
}
int Manager::getMinimumFromMap(map<int, int> dict){
    int res = MAX;
    if(dict.size() == 0) return -1;
    for(auto d:dict){
        if(d.second<res) res = d.second;
    }
    return res-1;
}

// 维护历史取货字典
void Manager::mainHistoryGetMap(Workstation *w){
    historyGetMap[w->type] += w->getRank();
}
// 维护历史填充字典
void Manager::mainHistoryFillMap(int w_type, int item){
    // 只对456的填充使用fillMap  因为123不需要填充 789填充不需要根据fillMap抉择
    if(item < 4){
        tuple<int, int> fill = {w_type, item};
        historyFillMap[fill] += 1;
    }
}

void Manager::handleGetTask(Robot* r){
    const tuple<int, int> &action = r->getAction();         // 当前机器的指令规划  工作台序号  物品序号
    Workstation* w = g_workstations[get<0>(action)];
    // 取物品 判断可行与否
    if(g_workstations[r->getNextWorkerId()]->can_production_recycle(get<1>(action))){
        // 取
        cout<<"buy "+to_string(r->id)<<endl;
        // 取之后
        // 对机器人
        r->resetAction();
        // 对工作台
        w->rel_locked_production(get<1>(action));
        // 维护历史字典
        mainHistoryGetMap(w);
    }else{
        // 不可行 任务终止
        r->resetAction();
        g_workstations[get<0>(action)]->rel_locked_production(get<1>(action));
    }
}

void Manager::handleSetTask(Robot* r){
    int item = r->item_carried;
    const tuple<int, int> &action = r->getAction();         // 当前机器的指令规划  工作台序号  物品序号
    Workstation* w = g_workstations[get<0>(action)];
    // 放
    cout<<"sell "+to_string(r->id)<<endl;
    // 放之后
    // 对机器人
    r->resetAction();
    // 对工作台
    w->rel_locked_production(item);
    w->putIn(item);
}

void Manager::handleTask(Robot* r){
    int item = r->item_carried;
    const tuple<int, int> &action = r->getAction();         // 当前机器的指令规划  工作台指针  物品序号
    Workstation* w = g_workstations[get<0>(action)];
    // 先判断是否到达
    if(r->workshop_located == get<0>(action)){
        if(item == 0 && w->product_status==1){
            handleGetTask(r);
        }
        else if(item>0&&w->production_needed(item)){
            handleSetTask(r);
        }
    }else{
        // 说明未到达 仍需判断调整！ 重难点 如何调整
        // 对于取物品 每回合都要判断一下调度任务是否依然可行 对放物品则不需要如此 因为放物品会加锁但取物品只对第一个站台加锁 防止出现取到东西后没地方放的情况
        if(item > 0) r->move2ws(w);
        else{
            // 取物品 判断可行与否
            if(g_workstations[r->getNextWorkerId()]->can_production_recycle(get<1>(action))){
                // 可行 继续调整
                r->move2ws(w);
            }else{
                // 不可行 任务终止
                r->resetAction();
                g_workstations[get<0>(action)]->rel_locked_production(get<1>(action));
            }
        }
    }
}

void Manager::assignTask(int frame_id, Robot* r, queue<int> robot_ids){
    int item = r->item_carried;
    // 没有物品要去取
    // 取到的物品必须要有地方放
    if(item == 0){
        // 分配取货物的任务
        assignGetTask(frame_id, r, robot_ids);
    }else{
        // 分配送货物的任务
        assignSetTask(frame_id, r);
    }
}

void Manager::handleFps(int frame_id){
    queue<int> robot_ids;  // 4个机器人下标记录哪个机器人没有被分配任务 被抢占的机器人要被压入队列中重新分配任务
    for(int i = 0; i < g_robots.size(); i++){robot_ids.push(g_robots[i]->id);}
    // 循环每个机器人分配任务   只有当前空闲状态或者上一帧的任务被抢走的工作台才会执行assignTasks   已有任务的机器人执行handleTasks
    while(!robot_ids.empty()){
        int i = robot_ids.front();
        robot_ids.pop();
        Robot* r = g_robots[i];
        const tuple<int, int> &action = r->getAction();         // 当前机器的指令规划  工作台序号  物品序号
        if(get<0>(action) != -1){                               // 机器人的工作台编号不为-1 说明有该机器有任务调度
            Manager::handleTask(r);
        }else{                                                     // 没有任务调度 安排任务喽
            Manager::assignTask(frame_id, r, robot_ids);
        }
    }

}