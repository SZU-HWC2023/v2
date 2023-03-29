#include "manager.h"

 // 买货物（取货物）的代价   是否可以优化？？？？
tuple<double, Workstation*> getTimePriceForBuy(Robot* r, Workstation *w, int frame_id){
    // 判断是否有工作站接收
    double time02 = MAX;
    Workstation* res_nxt_w;
    for(auto iter=g_item_to_ws.equal_range(w->production_item.type); iter.first != iter.second; ++iter.first){
        Workstation* nxt_w = iter.first->second;
        if(nxt_w->can_production_recycle(w->production_item.type)){
            double distance = calcDistance(w->coordinate, nxt_w->coordinate);
            double tmp = distance/MAX_FORWARD_SPD;
            if(tmp < time02){
                time02 = tmp;
                res_nxt_w = nxt_w;
            }
        }
    }
    // 剩余时间双倍代价   取货时
    double res = w->product_status==1?0:w->getLeftTime()*0.02;
    // 计算到达工作站台取货物的时间。  max(剩余时间， 距离耗时+转向耗时)
    double dis = calcDistance(r->coordinate, w->coordinate);
    double time01 = max(res, dis/MAX_FORWARD_SPD);
    // 返回值为-1说明不可取
    if(time01 == res) return {-1, NULL};
    int lat_time_free = w->type<=3?200:100;
    if(time02/0.02 > (MAX_FRAME-frame_id-time01/0.02-lat_time_free)) return {-1, NULL};
    return {(time01+0.5*time02), res_nxt_w};
}

// 放物品调度
void Map1::assignSetTask(int frame_id, Robot* r){
    priority_queue<tuple<double, Workstation*>, vector<tuple<double, Workstation*>>, greater<tuple<double, Workstation*>>> pq; // 总代价(时间表示 帧) 目标工作台
    int item = r->item_carried;
    // 有物品 需要去某个站台出售该物品 这就很简单 朝向理论时间最短的去站台去。
    // 情况1：选中的物品可能会没有工作站台接收他  ： 在时间代价里计算是否有空闲机器
    // 优先送差这一个就能接收的站台
    for(auto iter=g_item_to_ws.equal_range(item);
        iter.first != iter.second; ++iter.first){
        Workstation *w = iter.first->second;
        double timePrice = 1;     // 帧代价         不需要考虑距离因素  放物品的时候
        if(w->can_production_recycle(item)){
            int left_frame = MAX_FRAME-frame_id;
            if(item < 7 && w->type == 9) continue;
            if(item == 7){
                // 七号物品 仅仅考虑距离
                pq.push({timePrice, w});
            }else if(item >= 4){
                // 456号物品 考虑距离 和 接受它的站台缺失物品数量
                double weight = pow(w->getWeight(), 4);
                if(left_frame < 1500) weight = 1.0;
                pq.push({timePrice*weight, w});
            }else{
                // 123号物品 考虑距离 和 生产产品数量 和 缺失物品数量
                double weight = pow(w->getWeight(), 2);
                if(left_frame < 1000) weight = 1.0;
                pq.push({timePrice*weight, w});
            }
        } 
    }
    // 下达指令 朝向最小的工作站台
    if(pq.size()>0){
        // while循环可以类比取货物  看看能否加一些东西？？？？
        priority_queue<tuple<double, Workstation*>, vector<tuple<double, Workstation*>>, greater<tuple<double, Workstation*>>> tmp_pq = pq;
        tuple<double, Workstation*> minTimePriceWorker = tmp_pq.size()>0?tmp_pq.top():pq.top();
        Workstation* w = get<1>(minTimePriceWorker);
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
    priority_queue<tuple<double, Workstation*, Workstation*>, vector<tuple<double, Workstation*, Workstation*>>, greater<tuple<double, Workstation*, Workstation*>>> pq; // 总代价(时间表示 帧) 目标工作台 下一个工作台
    // 没有物品 要去向某个站点取物品
    for(int j = 0; j < g_workstations.size(); j++){
        Workstation* w = g_workstations[j];
        if(w->can_production_sell()){
            // 当前机器人的取货代价。
            if(w->production_locked(w->production_item.type)){
                continue;
            }
            double weight = 1.0;
            // 没有continue说明要么工作台没有被锁 要么当前的机器人距离代价比上一个锁住工作台的机器人代价更小 压入pq优先队列中
            tuple<double, Workstation*> tup01 = getTimePriceForBuy(r, w, frame_id);
            Workstation* nxt_W = get<1>(tup01);   // 取完后可以送的下一个工作台id
            double time_price = get<0>(tup01);   // 平均利润(性价比)
            if(time_price >= 0) pq.push({time_price, w, nxt_W});
        }
    }
    if(pq.size()>0){
        // 下达指令 朝向最小的工作站台
        tuple<double, Workstation*, Workstation*> minTimePriceWorker = pq.top();
        Workstation* w = get<1>(minTimePriceWorker);
        if(w->production_locked(w->production_item.type)){
            // 在这里才真正实现易主
            int last_locked_id = w->getLocked()[w->production_item.type];
            // 上一个机器人重置
            g_robots[last_locked_id]->resetAction();
            g_robots[last_locked_id]->setNextWorkerId(-1);
            // 上一个机器人重新分配任务
            robot_ids.push(last_locked_id);
            // 工作台被新的机器人锁定
            w->changeLocked(w->production_item.type, r->id);
        }else{
            // 对工作台 并没有释放锁 所以不会有什么变化
            w->add_locked_production(w->production_item.type, r->id);
        }
        // 对这个机器人
        r->setNextWorkerId(get<2>(minTimePriceWorker)->id);
        r->setAction({get<1>(minTimePriceWorker), w->production_item.type});
        // 调整 前进
        r->move2ws(w);
    }
}
