#include "class.h"

vector<Point*> AStar::planning(int sx,int sy,int gx,int gy){
    Point* start_node = new Point(sx,sy,0.0, nullptr);
    Point* goal_node = new Point(gx,gy,0.0, nullptr);
    map<tuple<int,int>,Point*> open_map;    // 存储待检测节点
    map<tuple<int,int>,Point*> closed_map;  // 存储已经检测过的节点
    open_map[getIndex(start_node)] = start_node;
    while(true){
        if(open_map.size()==0)break;

        float cost = MAX;
        tuple<int,int> c_id;
        // 从待检测节点中，找到一个到目标节点代价最小的节点
        for(auto iter = open_map.begin();iter!=open_map.end();iter++){
            Point* p = iter->second;
            float tmp_cost = this->calc_heuristic(p,goal_node) +p->cost;
            if(tmp_cost < cost){
                cost = tmp_cost;
                c_id = iter->first;
            }
        }
        Point* current = open_map[c_id];
        // 已经找到目标节点，退出循环
        if(current->x == goal_node->x && current->y == goal_node->y){
            goal_node->cost = current->cost;
            goal_node->parent_node = current->parent_node;
            break;
        }
        open_map.erase(c_id);   //将已经访问过的节点从开放列表移除
        closed_map[c_id] = current;
        //从当前节点往各个方向探索
        for(tuple<int,int, float> mot:this->motion){
            Point* point = new Point(current->x + get<0>(mot),current->y + get<1>(mot),current->cost+get<2>(mot),current);
            tuple<int,int> p_id = getIndex(point);
            if(!verify(point))continue;
            if(closed_map.find(p_id)!=closed_map.end())continue;
            if(open_map.find(p_id)==open_map.end()){
                open_map[p_id] = point;
            }else{
                if(open_map[p_id]->cost > point->cost){
                    open_map[p_id] = point;
                }
            }

        }
    }
    //回溯路径
    return this->calc_final_path(goal_node,closed_map);

}
vector<Point*> AStar::calc_final_path(Point* goal_node,map<tuple<int,int>,Point*> &closed_map){
    if(goal_node->parent_node== nullptr)return {};
    vector<Point*> result;
    result.emplace_back(goal_node);
    Point* parent = goal_node->parent_node;
    while(parent!=nullptr){
        Point* p = closed_map[getIndex(parent)];
        result.emplace_back(p);
        parent = p->parent_node;
    }
    reverse(result.begin(),result.end());
    return result;
}
//判断下标是否合法
bool AStar::verify(Point* p){
    if(p->x<0 || p->y<0 ||p->x>=MAP_TRUE_SIZE||p->y>=MAP_TRUE_SIZE)return false;
    if(g_map[p->x][p->y] == '#')return false;
//        if(p->x >0){
//            if(g_map[p->x-1][p->y] == '#')return false;
//            if(p->y<MAP_TRUE_SIZE-1){
//                if(g_map[p->x][p->y+1] =='#')return false;
//            }
//        }
    return true;
}
tuple<int,int> AStar::getIndex(Point* p){
    return {p->x,p->y};
}
float AStar::calc_heuristic(Point* a, Point *b){
    float x = a->x - b->x;
    float y = a->y - b->y;
    return sqrt(x*x + y*y);
}
vector<tuple<int,int, float >> AStar::get_motion_model(){
    vector<tuple<int,int, float >> motion = {
            {1, 0, 1.0},
            {0, 1,  1.0},
            {-1, 0, 1.0},
            {0, -1, 1.0},
            {-1, -1,sqrt(2.0)},
            {-1, 1,sqrt(2.0)},
            {1, -1,sqrt(2.0)},
            {1, 1,sqrt(2.0)}
    };
    return motion;
}
void test_astar(){
    AStar* aStartPlannesr = new AStar();
    vector<Point*> result = aStartPlannesr->planning(89,4,71,35);

    char local_map[MAP_TRUE_SIZE][MAP_TRUE_SIZE];
    for(int i=0;i<MAP_TRUE_SIZE;i++){
        for(int j=0;j<MAP_TRUE_SIZE;j++){
            local_map[i][j] = g_map[i][j];
        }
    }
    fprintf(stderr,"%d\n",result.size());
    for(Point* point:result){
        int x = point->x;
        int y = point->y;
        local_map[x][y] = 'O';
    }
    for(int i=0;i<MAP_TRUE_SIZE;i++){
        for(int j=0;j<MAP_TRUE_SIZE;j++){
            fprintf(stderr,"%c",local_map[i][j]);
        }
        fprintf(stderr,"\n");
    }
}