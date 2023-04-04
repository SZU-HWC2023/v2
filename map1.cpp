#include "manager.h"

// 这个应该是去读取

// 买货物（取货物）的代价   是否可以优化？？？？
tuple<double, int> getTimePriceForBuy(Robot* r, Workstation *w, int frame_id){
    // 判断是否有工作站接收
    double time02 = MAX;
    int nxt_worker_id = 0;
    for(auto iter=g_item_to_ws.equal_range(w->production_item.type); iter.first != iter.second; ++iter.first){
        Workstation* nxt_w = iter.first->second;
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

// 放物品调度
void Map1::assignSetTask(int frame_id, Robot* r){
    priority_queue<tuple<double, int>, vector<tuple<double, int>>, greater<tuple<double, int>>> pq; // 总代价(时间表示 帧) 目标工作台
    int item = r->item_carried;
    // 有物品 需要去某个站台出售该物品 这就很简单 朝向理论时间最短的去站台去。
    // 情况1：选中的物品可能会没有工作站台接收他  ： 在时间代价里计算是否有空闲机器
    // 优先送差这一个就能接收的站台
    for(auto iter=g_item_to_ws.equal_range(item); iter.first != iter.second; ++iter.first){
        Workstation *w = iter.first->second;
        if(g_connected_areas_c[w->coordinate]!=g_connected_areas_c[r->coordinate]) continue;
        vec2_int r_local_coor = {0,0};
        if( r->workshop_located>=0)
            r_local_coor = g_workstations[r->workshop_located]->coordinate.toIndex();
        vec2_int w_coor = w->coordinate.toIndex();
        double timePrice = MAX;     // 帧代价         不需要考虑距离因素  放物品的时候
        if(g_astar_path_distance.count({r_local_coor.row, r_local_coor.col, w_coor.row, w_coor.col})>0)
            timePrice = g_astar_path_distance[{r_local_coor.row, r_local_coor.col, w_coor.row, w_coor.col}];
        if(w->can_production_recycle(item)){
            int left_frame = MAX_FRAME-frame_id;
            if(item == 7){
                // 七号物品 仅仅考虑距离
                pq.push({timePrice, w->id});
            }else if(item >= 4){
                // 456号物品 考虑距离 和 接受它的站台缺失物品数量
                double weight = pow(w->getWeight(), 2);
                if(left_frame < 1500) weight = 1.0;
                pq.push({timePrice*weight, w->id});
            }else{
                // 123号物品 考虑距离 和 历史填充数量 和 缺失物品数量
                double weight = pow(w->getWeight(), 2);
                tuple<int, int> fill = {w->type, item};
                int fill_count = historyFillMap.count(fill)>0?historyFillMap[fill]:0;
                weight *= pow(fill_count - getMinimumFromMap(historyFillMap), 2);
                if(left_frame < 1000) weight = 1.0;
                pq.push({timePrice*weight, w->id});
            }
        }
    }
    // 下达指令 朝向最小的工作站台
    if(pq.size()>0){

        // while循环可以类比取货物  看看能否加一些东西？？？？
        priority_queue<tuple<double, int>, vector<tuple<double, int>>, greater<tuple<double, int>>> tmp_pq = pq;
        tuple<double, int> minTimePriceWorker = tmp_pq.size()>0?tmp_pq.top():pq.top();
        Workstation* w = g_workstations[get<1>(minTimePriceWorker)];
        // 维护历史填充记录
        mainHistoryFillMap(w->type, item);
        // 对机器人
        r->setAction({get<1>(minTimePriceWorker), item});
        // 对工作台
        w->add_locked_production(item, r->id);
        // 调整 前进
        r->move2ws(w);
    }
}
// 取物品调度
void Map1::assignGetTask(int frame_id, Robot* r, queue<int> robot_ids){
    priority_queue<tuple<double, int, int>, vector<tuple<double, int, int>>, greater<tuple<double, int, int>>> pq; // 总代价(时间表示 帧) 目标工作台 下一个工作台
    // 没有物品 要去向某个站点取物品
    for(int j = 0; j < g_workstations.size(); j++){
        Workstation* w = g_workstations[j];
        if(g_connected_areas_uc[w->coordinate]!=g_connected_areas_uc[r->coordinate]) continue;
        if(w->can_production_sell()){
            // 当前机器人的取货代价。
            if(w->production_locked(w->production_item.type)){
                continue;
            }
            double weight = pow(historyGetMap[w->type]-getMinimumFromMap(historyGetMap), 4);
            // 没有continue说明要么工作台没有被锁 要么当前的机器人距离代价比上一个锁住工作台的机器人代价更小 压入pq优先队列中
            tuple<double, int> tup01 = getTimePriceForBuy(r, w, frame_id);
            int nxt_worker_id = get<1>(tup01);   // 取完后可以送的下一个工作台id
            double time_price = get<0>(tup01);   // 平均利润(性价比)
            if(time_price >= 0) pq.push({time_price*weight, w->id, nxt_worker_id});
        }
    }
    if(pq.size()>0){
        // 下达指令 朝向最小的工作站台
        tuple<double, int, int> minTimePriceWorker = pq.top();
        Workstation* w = g_workstations[get<1>(minTimePriceWorker)];
        if(w->production_locked(w->production_item.type)){
            // 在这里才真正实现易主
            int last_locked_id = w->getLocked()[w->production_item.type];
            // 上一个机器人重置
            g_robots[last_locked_id]->resetAction();
            // 上一个机器人重新分配任务
            robot_ids.push(last_locked_id);
            // 工作台被新的机器人锁定
            w->changeLocked(w->production_item.type, r->id);
        }else{
            // 对工作台 并没有释放锁 所以不会有什么变化
            w->add_locked_production(w->production_item.type, r->id);
        }
        // 对这个机器人
        r->setNextWorkerId(g_workstations[get<2>(minTimePriceWorker)]->id);
        r->setAction({get<1>(minTimePriceWorker), w->production_item.type});
        // 调整 前进
        r->move2ws(w);
    }else if(frame_id > 50 && MAX_FRAME-frame_id>500){
        // 对工作台 并没有释放锁 所以不会有什么变化
        int min_dis_worker_id = 0;
        double time = MAX;
        for(int type_id = 1; type_id <= 3;type_id++){
            for(auto iter=g_item_from_ws.equal_range(type_id); iter.first != iter.second; ++iter.first){
                Workstation* w= iter.first->second;
                double tmp = 1.0;
                double weight = 1.0;
                // 加入历史平衡的决策
                if(MAX_FRAME-frame_id>1500) weight = pow(historyGetMap[w->type]-getMinimumFromMap(historyGetMap), 4);   // 根据历史购买记录平衡去买哪个货物
                if(tmp*weight < time){
                    time = tmp*weight;
                    min_dis_worker_id = w->id;
                }
            }
        }
        Workstation* w = g_workstations[min_dis_worker_id];
        // 对工作台 并没有释放锁 所以不会有什么变化
        w->add_locked_production(w->production_item.type, r->id);
        // 对这个机器人
        r->setNextWorkerId(6);
        r->setAction({min_dis_worker_id, w->type});
        // 调整 前进
        r->move2ws(w);
    }
}
