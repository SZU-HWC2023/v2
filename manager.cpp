#include "manager.h"



// 这个应该是去读取

// 买货物（取货物）的代价
tuple<double, int> Manager::getTimePriceForBuy01(Robot* r, Workstation *w, int frame_id){
    // 判断是否有工作站接收
    double time02 = MAX;
    int nxt_worker_id = -1;
    for(auto iter=g_item_to_ws.equal_range(w->production_item.type); iter.first != iter.second; ++iter.first){
        Workstation* nxt_w = iter.first->second;
        if(g_map[nxt_w->coordinate.toIndex()] == '$' || w->ban) continue;
        if(nxt_w->can_production_recycle(w->production_item.type)){
            double distance = calcDistance(w->coordinate, nxt_w->coordinate);
            double tmp = distance/MAX_FORWARD_SPD;
            if(tmp < time02){
                time02 = tmp;
                nxt_worker_id = nxt_w->id;
            }
        }
    }
    // 剩余时间双倍代价   取货时
    double res = w->product_status==1?0:w->getLeftTime()*0.02;
    // 计算到达工作站台取货物的时间。  max(剩余时间， 距离耗时+转向耗时)
    double dis = calcDistance(r->coordinate, w->coordinate);
    double time01 = max(res, dis/MAX_FORWARD_SPD);
    // 返回值为-1说明不可取
    if(time01 == res) return {-1, -1};
    int lat_time_free = w->type<=3?200:100;
    if(time02/0.02 > (MAX_FRAME-frame_id-time01/0.02-lat_time_free)) return {-1, -1};
    return {(time01+0.5*time02), nxt_worker_id};
}

// w1代表当前机器人所属的工作台  w2代表目标工作台
tuple<double, int> Manager::getTimePriceForBuy02(Robot* r, Workstation *w, int frame_id){
    Workstation* r_w = g_workstations[r->workshop_located];
    vec2_int w_i = w->coordinate.toIndex();
    vec2_int r_w_i = r_w->coordinate.toIndex();
    // 判断是否有工作站接收
    double time02 = MAX;
    int nxt_worker_id = -1;
    for(auto iter=g_item_to_ws.equal_range(w->production_item.type); iter.first != iter.second; ++iter.first){
        Workstation* nxt_w = iter.first->second;
        if(g_map[nxt_w->coordinate.toIndex()] == '$' || w->ban) continue;
        if(g_connected_areas_uc[w->coordinate]!=g_connected_areas_uc[nxt_w->coordinate] || nxt_w->ban) {continue;}
        if(nxt_w->can_production_recycle(w->production_item.type)){
            vec2_int nxt_w_i = nxt_w->coordinate.toIndex();
            double distance = g_astar_path_distance[{w_i.row, w_i.col, nxt_w_i.row, nxt_w_i.col}];
            double tmp = distance/MAX_FORWARD_SPD;
            if(tmp < time02){
                time02 = tmp;
                nxt_worker_id = nxt_w->id;
            }
        }
    }
    // 剩余时间双倍代价   取货时  4s损失代价
    double res = w->product_status==1?0:w->getLeftTime()*0.02;
    // 计算到达工作站台取货物的时间。  max(剩余时间， 距离耗时+转向耗时)
    double dis = g_astar_path_distance[{r_w_i.row, r_w_i.col, w_i.row, w_i.col}];
    double time01 = max(res, dis/MAX_FORWARD_SPD+1);
    // 返回值为-1说明不可取
    if(time01 == res && time01 != 0) return {-1, -1};
    int lat_time_free = w->type<=3?250:150;
    if(time02/0.02 > (MAX_FRAME-frame_id-time01/0.02-lat_time_free)) return {-1, -1};
    return {(time01+0.5*time02), nxt_worker_id};
}


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
    if(item < 4 && w_type<=6){
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
        if(item > 0) {
            r->move2ws(w);}
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
    for(int i = 0; i < ROBOT_NUM; i++){
        if(!g_robots[i]->ban) robot_ids.push(g_robots[i]->id);
    }
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