#include "class.h"

map<tuple<int,int,int,int>,vector<Point*>> g_astar_path; //存储平台之间的关键路径，srow,scol,grow,gcol 起点到终点的坐标
map<tuple<int,int,int,int>,float> g_astar_path_distance; //存储平台之间的关键路径长度，srow,scol,grow,gcol 起点到终点的坐标
map<tuple<int,int,int,int>,vector<Point*>> g_astar_product_path; //带有产品时，存储平台之间的关键路径，sx,sy,gx,gy 起点到终点的坐标
AStar *g_astar;
AStarTest *g_astartest;
DoubleDirectionAstar* g_directionAstar;

//计算路径的长度
float calc_distance_path(vector<Point*> &vec_paths){
    float distance = 0.0;
    for(int i=1;i<vec_paths.size();i++){
        vec2 pre_point = vec_paths[i-1]->coordinate.toCenter();
        vec2 current_point = vec_paths[i]->coordinate.toCenter();
        distance += calcDistance(pre_point,current_point);
    }
    return distance;
}
//初始化A*算法、平台之间的关键路径及其路径长度
void init_points(){
    // 初始化A*算法相关的类
    g_astartest = new AStarTest();
    for(int i=0;i<g_workstations.size();i++){
        if(g_workstations[i]->ban) {continue;}
        for(int j = i+1; j < g_workstations.size();j++){
            if(g_workstations[i]->ban) {continue;}
            if(g_workstations[j]->type <= 3 && g_workstations[j]->type <= 3) continue;
            Workstation* src_workstation = g_workstations[i];
            Workstation* des_workstation = g_workstations[j];
            vec2_int src = g_direction_map.to_pos_idx(src_workstation->coordinate);
            vec2_int des = g_direction_map.to_pos_idx(des_workstation->coordinate);
             // 不带物品都不在同一连通域中 无需找路径
            if(g_connected_areas_uc.map[src.row][src.col] != g_connected_areas_uc.map[des.row][des.col]){
                continue;
            }
            //为起点工作台和终点工作台规划一条路径 不带物品规划路径
            vector<Point*> result = g_astartest->planning(src,des,false);
            if(result.size() == 0){
                continue;
            }
            //计算该路径的长度
            float distance = calc_distance_path(result);
            //将结果存储到map中
            g_astar_path_distance[{src.row,src.col,des.row,des.col}] = distance;
            g_astar_path[{src.row,src.col,des.row,des.col}] = result;
            vector<Point*> result_inv = result;
            reverse(result_inv.begin(),result_inv.end());
            g_astar_path[{des.row,des.col,src.row,src.col}] = result_inv;
            g_astar_path_distance[{des.row,des.col,src.row,src.col}] = distance;

            //有产品的路径
            vector<Point*> result_product = g_astartest->planning(src,des,true);
            if(result_product.size()==0)continue;
            g_astar_product_path[{src.row,src.col,des.row,des.col}] = result_product;
            vector<Point*> result_product_inv = result_product;
            reverse(result_product_inv.begin(),result_product_inv.end());
            g_astar_product_path[{des.row,des.col,src.row,src.col}] = result_product_inv;
        }
    }
}
bool judgeAroundObstacle(int row, int col){
    if(g_map[row+1][col] == '#' || g_map[row][col+1] == '#' || g_map[row-1][col] == '#' || g_map[row][col-1] == '#') return true;
    if(g_map[row+1][col+1] == '#' || g_map[row-1][col-1] == '#' || g_map[row-1][col+1] == '#' || g_map[row+1][col-1] == '#') return true;
    return false;
}
// 优先队列比较函数
struct cmp
{
    bool operator() (const Point* x1, const Point* x2)
    {
        return x1->cost + x1->current_to_goal_cost > x2->cost +x2->current_to_goal_cost;
    }
};

vector<Point*> AStarTest::planning(vec2_int src_point,vec2_int des_point,bool has_product){

    vis.fill({false});
    open_map.fill({nullptr});
    closed_map.fill({nullptr});
    Point* start_node = new Point(src_point,0.0, nullptr);
    Point* goal_node = new Point(des_point,0.0, nullptr);
    open_map[src_point.row][src_point.col] = start_node;
    int open_map_size = 1;

    priority_queue<Point*, vector<Point*>, cmp> que;
    que.push(start_node);
    while(true){
        if(open_map_size==0) break;

        Point* current = que.top();
        que.pop();

        int cx  = current->coordinate.row;
        int cy = current->coordinate.col;
        vis[cx][cy] = true;
        // 已经找到目标节点，退出循环
        if(current->coordinate == goal_node->coordinate){
            goal_node->cost = current->cost;
            goal_node->parent_node = current->parent_node;
            break;
        }
        open_map[cx][cy] = nullptr;
        open_map_size --;
        closed_map[cx][cy] = current;

        //从当前节点往各个方向探索
        vector<vec2_int> directions =  g_direction_map.get_directions(current->coordinate);

        for(vec2_int v: directions){
            vec2_int new_coor = current->coordinate + v;
            float baseCost = abs(v.row) + abs(v.col);
            Point* point = new Point(new_coor ,current->cost+baseCost,current);
            int px = point->coordinate.row;
            int py = point->coordinate.col;
            if(!verify(point,goal_node,has_product))continue;
            if(vis[px][py])continue;
            if(closed_map[px][py]!= nullptr)continue;
            point->current_to_goal_cost = this->calc_heuristic(point,goal_node);

            if(open_map[px][py] == nullptr){
                open_map[px][py] = point;
                que.push(point);
                open_map_size++;
            }else{
                if(open_map[px][py]->cost > point->cost){
                    open_map[px][py] = point;
                }
            }
        }
    }
    return calc_final_path(goal_node, has_product);
}
vector<Point*> AStarTest::calc_final_path(Point* goal_node, bool has_product){
    if(goal_node->parent_node== nullptr)return {};
    vector<Point*> result;
    result.emplace_back(goal_node);
    Point* parent = goal_node->parent_node;
    while(parent!=nullptr){
        int px = parent->coordinate.row;
        int py = parent->coordinate.col;
        Point* p = closed_map[px][py];
        result.emplace_back(p);
        parent = p->parent_node;
    }
    reverse(result.begin(),result.end());
    vector<Point*> simplify_result = simplify_path(result, has_product);
    return simplify_result;
    // return result;
}

/*
    简化A*算法求得的路径
 @param vec_points A*算法找到的路径
 @param has_product 机器人是否携带产品
 @return vector<Point*> 简化后的路径，包含关键节点
 */
vector<Point *> AStarTest::simplify_path(vector<Point*> &vec_points,bool has_product){

    //路径的点的数小于等于2,不用简化，直接返回
    if(vec_points.size()<=2)return vec_points;
    //起点和终点之间没有障碍物,说明这条路径可以直接由起点和终点表示
    if(!g_map.obstacle_in_line(vec_points[0],vec_points.back(),has_product)) return {vec_points[0],vec_points.back()};
    vector<Point*> result;
    int i = 0, size = vec_points.size();
    result.emplace_back(vec_points[i]);
    while(i < size-1){
        for(int j = size-1; j>i; j--){
            if(!g_map.obstacle_in_line(vec_points[i],vec_points[j],has_product)){
                // 直到没有障碍物
                if(j-1 > i)  result.emplace_back(vec_points[j-1]);
                result.emplace_back(vec_points[j]);
                if(j+1 < size-1) result.emplace_back(vec_points[j+1]);
                i = j;
                break;
            }
        }
        // 如果都有障碍物
        if(i+1<size){
            i++;
            result.emplace_back(vec_points[i]);
        }
    }
    return result;
}

//判断下标是否合法
bool AStarTest::verify(Point * p,Point *goal_node,bool has_product){
    if(p->coordinate == goal_node->coordinate)return true;
    //下标超出地图
    if(p->coordinate.col <= 0 || p->coordinate.row <= 0 ||
    p->coordinate.col>=MAP_TRUE_SIZE || p->coordinate.row >= MAP_TRUE_SIZE
    || !g_direction_map.is_carry_passable(p->coordinate)&&has_product)return false;
    
    return true;
}
float AStarTest::calc_heuristic(Point * a, Point *b){
    float row = a->coordinate.row - b->coordinate.row;
    float col = a->coordinate.col - b->coordinate.col;
    return abs(row) + abs(col);
}
