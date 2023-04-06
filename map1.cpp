#include "manager.h"

// 这个应该是去读取

// 买货物（取货物）的代价
tuple<double, int> getTimePriceForBuy01(Robot* r, Workstation *w, int frame_id){
    // 判断是否有工作站接收
    double time02 = MAX;
    int nxt_worker_id = -1;
    for(auto iter=g_item_to_ws.equal_range(w->production_item.type); iter.first != iter.second; ++iter.first){
        Workstation* nxt_w = iter.first->second;
        if(g_map[nxt_w->coordinate.toIndex()] == '$') continue;
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
tuple<double, int> getTimePriceForBuy02(Robot* r, Workstation *w, int frame_id){
    Workstation* r_w = g_workstations[r->workshop_located];
    vec2_int w_i = w->coordinate.toIndex();
    vec2_int r_w_i = r_w->coordinate.toIndex();
    // 判断是否有工作站接收
    double time02 = MAX;
    int nxt_worker_id = -1;
    for(auto iter=g_item_to_ws.equal_range(w->production_item.type); iter.first != iter.second; ++iter.first){
        Workstation* nxt_w = iter.first->second;
        if(g_map[nxt_w->coordinate.toIndex()] == '$') continue;
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


// 取物品调度
void Map1::assignGetTask(int frame_id, Robot* r, queue<int> robot_ids){
    priority_queue<tuple<double, int, int>, vector<tuple<double, int, int>>, greater<tuple<double, int, int>>> pq; // 总代价(时间表示 帧) 目标工作台 下一个工作台
    // 没有物品 要去向某个站点取物品
    for(int j = 0; j < g_workstations.size(); j++){
        Workstation* w = g_workstations[j];
        if(g_connected_areas_uc[w->coordinate]!=g_connected_areas_uc[r->coordinate] || w->ban) {continue;}
        double price_cur = calcDistance(r->coordinate, w->coordinate);
        if(w->can_production_sell()){
            // 当前机器人的取货代价。
            if(w->production_locked(w->production_item.type)){
                // Robot *last_locked_r = g_robots[w->locked[w->type]];
                // double price_last = calcDistance(last_locked_r->coordinate, w->coordinate);
                // if(price_cur >= price_last){
                //     // 被锁了 又代价更大 不会有解锁的想法
                //     continue;
                // }
                continue;
            }
            tuple<double, int> tup;
            double weight = pow(historyGetMap[w->type]-getMinimumFromMap(historyGetMap), 2);
            // 没有continue说明要么工作台没有被锁 要么当前的机器人距离代价比上一个锁住工作台的机器人代价更小 压入pq优先队列中
            if(r->workshop_located == -1){
                tup = getTimePriceForBuy01(r, w, frame_id);
            }else{
                tup = getTimePriceForBuy02(r, w, frame_id);
            }
            int nxt_worker_id = get<1>(tup);   // 取完后可以送的下一个工作台id
            double time_price = get<0>(tup);   // 平均利润(性价比)
            if(nxt_worker_id >= 0) pq.push({time_price*weight, w->id, nxt_worker_id});
        }
    }
    if(pq.size()>0){
        // 下达指令 朝向最小的工作站台
        priority_queue<tuple<double, int, int>, vector<tuple<double, int, int>>, greater<tuple<double, int, int>>> tmp_pq = pq;
            // 距离不是唯一选取标准 可以考虑一些特殊情况下优先选取
            while(tmp_pq.size()>0){
                Workstation* cur_w = g_workstations[get<1>(tmp_pq.top())];
                Workstation* nxt_w = g_workstations[get<2>(tmp_pq.top())];
                double dis01 = calcDistance(r->coordinate, cur_w->coordinate);
                double dis02 = calcDistance(nxt_w->coordinate, cur_w->coordinate);

                if(MAX_FRAME-frame_id < 1000 && cur_w->type == 7) break;
                // 送到后取优先级提高(如456送到后可以顺手取了7)
                if(cur_w->type == 7){
                    if(dis01 < r->crt_radius*4){
                        break;
                    }
                }
                // 456送到后取的优先级提高(如123送到后可以顺手取了456)
                if((nxt_w->type == 7) && MAX_FRAME-frame_id>1500){
                    if(dis01 < r->crt_radius*4){
                        break;
                    }
                }
                tmp_pq.pop();
            }
            // 说明没有可以取的
            if(tmp_pq.size()==0){
                tmp_pq = pq;
                while(tmp_pq.size()>0){
                    Workstation* cur_w = g_workstations[get<1>(tmp_pq.top())];
                    Workstation* nxt_w = g_workstations[get<2>(tmp_pq.top())];
                    // 如果已经有人锁定他的产出则不考虑该工作台 因为这个锁定住工作台的机器人可以顺手取了它的产出
                    if(cur_w->haveRawLocked() && MAX_FRAME-frame_id>500){
                        tmp_pq.pop();
                        continue;
                    }
                    // 4567只有送过去才会去取 不会大老远跑过去取
                    if(cur_w->type >= 4 && MAX_FRAME-frame_id > 2000 && cur_w->getMissingNum()!=0){
                        tmp_pq.pop();
                        continue;
                    }
                    break;
                }
            }

        tuple<double, int, int> minTimePriceWorker = (tmp_pq.size()>0?tmp_pq.top():pq.top());
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
                if(g_connected_areas_uc[w->coordinate]!=g_connected_areas_uc[r->coordinate] || w->ban) {continue;}
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


// 放物品调度
void Map1::assignSetTask(int frame_id, Robot* r){
    priority_queue<tuple<double, int>, vector<tuple<double, int>>, greater<tuple<double, int>>> pq; // 总代价(时间表示 帧) 目标工作台
    int item = r->item_carried;
    // 有物品 需要去某个站台出售该物品 这就很简单 朝向理论时间最短的去站台去。
    // 情况1：选中的物品可能会没有工作站台接收他  ： 在时间代价里计算是否有空闲机器
    // 优先送差这一个就能接收的站台
    for(auto iter=g_item_to_ws.equal_range(item); iter.first != iter.second; ++iter.first){
        Workstation *w = iter.first->second;
        if(g_map[w->coordinate.toIndex()] == '$') continue;
        if(g_connected_areas_c[w->coordinate]!=g_connected_areas_c[r->coordinate] || w->ban) continue;
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
                double weight = w->getWeight();
                if(left_frame < 1500) weight = 1.0;
                pq.push({timePrice*weight, w->id});
            }else{
                // 123号物品 考虑距离 和 历史填充数量 和 缺失物品数量
                double weight = w->getWeight();
                tuple<int, int> fill = {w->type, item};
                int fill_count = historyFillMap.count(fill)>0?historyFillMap[fill]:50;
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