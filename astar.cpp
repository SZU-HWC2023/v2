#include "class.h"

map<tuple<int,int,int,int>,vector<Point*>> g_astar_path; //存储平台之间的关键路径，srow,scol,grow,gcol 起点到终点的坐标
map<tuple<int,int,int,int>,float> g_astar_path_distance; //存储平台之间的关键路径长度，srow,scol,grow,gcol 起点到终点的坐标
map<tuple<int,int,int,int>,vector<Point*>> g_astar_product_path; //带有产品时，存储平台之间的关键路径，sx,sy,gx,gy 起点到终点的坐标
AStar *g_astar;
DoubleDirectionAstar* g_directionAstar;

bool near_obstacle(int row,int col){
    if(g_map[row][col] == '#')return true;
    vector<tuple<int,int>> motion = {
            {1,0},
            {0,1},
            {0,-1},
            {-1,0},
            {-1,-1},
            {-1,1},
            {1,-1},
            {1,1}
    };
    for(tuple<int,int> mot:motion){
        int drow = row + get<0>(mot);
        int dcol = col + get<1>(mot);

        if(g_map[drow][dcol]=='#')return true;
    }
    return false;
}
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
    g_astar = new AStar();
    // g_directionAstar = new DoubleDirectionAstar();
    for(int i=0;i<g_workstations.size();i++){
        for(int j = i+1; j < g_workstations.size();j++){
            Workstation* src_workstation = g_workstations[i];
            Workstation* des_workstation = g_workstations[j];
            vec2_int src = src_workstation->coordinate.toIndex();
            vec2_int des = des_workstation->coordinate.toIndex();
             // 不带物品都不在同一连通域中 无需找路径
            if(g_connected_areas_uc.map[src.row][src.col] != g_connected_areas_uc.map[des.row][des.col]){
                continue;
            }
            //为起点工作台和终点工作台规划一条路径 不带物品规划路径
            vector<Point*> result = g_astar->planning(src.row,src.col,des.row,des.col,false);
            if(result.size() == 0){
                continue;
            }
//            test_astar(result);
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
            vector<Point*> result_product = g_astar->planning(src.row,src.col,des.row,des.col,true);
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

vector<Point*> AStar::planning(int srow,int scol,int grow,int gcol, bool has_product){
    Point* start_node = new Point(srow,scol,0.0, nullptr);
    Point* goal_node = new Point(grow,gcol,0.0, nullptr);
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
            float tmp_cost = this->calc_heuristic(p,goal_node)+p->cost;
            if(tmp_cost < cost){
                cost = tmp_cost;
                c_id = iter->first;
            }
        }
        Point* current = open_map[c_id];
        // 已经找到目标节点，退出循环
        if(current->coordinate.col == goal_node->coordinate.col && current->coordinate.row == goal_node->coordinate.row){
            goal_node->cost = current->cost;
            goal_node->parent_node = current->parent_node;
            break;
        }
        open_map.erase(c_id);   //将已经访问过的节点从开放列表移除
        closed_map[c_id] = current;
        //从当前节点往各个方向探索
        for(tuple<int,int, float> mot:this->motion){
            float baseCost = get<2>(mot);
            if(judgeAroundObstacle(current->coordinate.row + get<0>(mot),current->coordinate.col + get<1>(mot)))baseCost *=1;
            Point* point = new Point(current->coordinate.row + get<0>(mot),current->coordinate.col + get<1>(mot),current->cost+baseCost,current);
            tuple<int,int> p_id = getIndex(point);
            if(!verify(current,point,has_product))continue;
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
    return this->calc_final_path(goal_node,closed_map,has_product);

}
vector<Point*> AStar::calc_final_path(Point* goal_node,map<tuple<int,int>,Point*> &closed_map,bool has_product){
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

    vector<Point*> simplified_path = this-> simplify_path(result,has_product); //路径简化
//     return simplified_path;
    return result;
}
//判断下标是否合法, has_product为true时表示机器人有东西
bool AStar::verify(Point* from,Point* p,bool has_product){
    //下标超出地图
    if(p->coordinate.col<0 || p->coordinate.row<0 ||p->coordinate.col>=MAP_TRUE_SIZE||p->coordinate.row>=MAP_TRUE_SIZE)return false;
    //p所处位置为墙
    if(g_map[p->coordinate] == '#')return false;
    //不拿东西走不通
    if(g_map[p->coordinate] == '!')return false;
    //墙角能取不能放
    if(has_product && g_map[p->coordinate] == '$')return false;
    if(has_product){
        //机器人有产品时，不能过只有两个空的
        if(g_map[p->coordinate] == '@'){
            return false;
        }
    }

    return true;
}
tuple<int,int> AStar::getIndex(Point* p){
    return {p->coordinate.row, p->coordinate.col};
}
float AStar::calc_heuristic(Point* a, Point *b){
    float row = a->coordinate.row - b->coordinate.row;
    float col = a->coordinate.col - b->coordinate.col;
    return sqrt(row*row + col*col);
}
//机器人移动的方向
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
void test_astar(vector<Point*> &result){

    char local_map[MAP_TRUE_SIZE][MAP_TRUE_SIZE];
    for(int i=0;i<MAP_TRUE_SIZE;i++){
        for(int j=0;j<MAP_TRUE_SIZE;j++){
            local_map[i][j] = g_map[i][j];
        }
    }
    fprintf(stderr,"%d\n",result.size());
    for(Point* point:result){
        int row = point->coordinate.row;
        int col = point->coordinate.col;
        local_map[row][col] = 'O';
    }
    for(int i=0;i<MAP_TRUE_SIZE;i++){
        for(int j=0;j<MAP_TRUE_SIZE;j++){
            fprintf(stderr,"%c",local_map[i][j]);
        }
        fprintf(stderr,"\n");
    }
}

bool help(Point* p){
    int row = p->coordinate.row;
    int col = p->coordinate.col;
    int sum = 0;
    for(int j=col-1;j>=0;j--){
        if(g_map[row][j]!='#')sum++;
        else{
            break;
        }
    }
    for(int j = col+1;j<MAP_TRUE_SIZE;j++){
        if(g_map[row][col]!='#')sum++;
        else break;
    }
    sum ++;
    return sum < 5;
}
/*
    简化A*算法求得的路径
 @param vec_points A*算法找到的路径
 @param has_product 机器人是否携带产品
 @return vector<Point*> 简化后的路径，包含关键节点
 */
vector<Point *> AStar::simplify_path(vector<Point*> &vec_points,bool has_product){

    //路径的点的数小于等于2,不用简化，直接返回
    if(vec_points.size()<=2)return vec_points;
    //起点和终点之间没有障碍物,说明这条路径可以直接由起点和终点表示
    if(!obstacle_in_line(vec_points[0],vec_points.back(),has_product)) return {vec_points[0],vec_points.back()};
    vector<Point*> result;
    result.emplace_back(vec_points[0]);
    for(int i=1;i<vec_points.size()-1;i++){
        if(g_map[vec_points[i]->coordinate]=='@' ||help(vec_points[i])){
            result.emplace_back(vec_points[i]);
            continue;
        }
        if(obstacle_in_line(result.back(),vec_points[i],has_product)){
            // 上一个没有障碍物 这一个就有障碍物了 选上一个
//            result.emplace_back(vec_points[i-2]);
//            result.emplace_back(vec_points[i-1]);
            result.emplace_back(vec_points[i]);
//            result.emplace_back(vec_points[i+1]);
//            result.emplace_back(vec_points[i+2]);
        }
    }
    result.emplace_back(vec_points.back());
    return result;
}

// 暂时没有任何使用
void AStar::divide_conquer(vector<Point*> &result,int left,int right,vector<Point*> &vec_points,bool has_product){
   if(left > right)return;
   if(left == right)return;
   // left和right没有障碍物 
   if(!obstacle_in_line(vec_points[left],vec_points[right],has_product)){
       result.emplace_back(vec_points[left]);
       return;
   }
   int mid = (right-left)/2 + left;
   divide_conquer(result,left,mid,vec_points,has_product);
   divide_conquer(result,mid+1,right,vec_points,has_product);
}
/*
判断字符矩阵的点是否相邻 上、下、左、右、左上、左下、右上、右下
 @param a 点a
 @param b 点b
 @return true 点a和点b相邻
 */
bool is_closed(tuple<int,int> a,tuple<int,int> b){
    int sx = get<0>(a), sy = get<1>(a);
    int gx = get<0>(b), gy = get<1>(b);
    if(sx == gx){
        if(abs(sy - gy) == 1)return true;
        return false;
    }
    if(sy == gy){
        if(abs(sx - gx)==1)return true;
        return false;
    }
    if(abs(sx - gx) + abs(sy - gy) == 2)return true;
    return false;

}


//节点交换
vector<tuple<int,int>> swap_point(tuple<int,int> a,tuple<int,int> b){
    int sx = get<0>(a), sy = get<1>(a);
    int gx = get<0>(b), gy = get<1>(b);
    if(sx<=gx)return {a,b};
    return {b,a};
}
/*
 判断水平方向上是否有障碍物
 @param src_point 起点
 @param des_point 终点
 @return true 起点和终点之间有障碍物
 */
bool row_obstacle(tuple<int,int> a,tuple<int,int> b){
    int srow = get<0>(a), scol = get<1>(a);
    int grow = get<0>(b), gcol = get<1>(b);
    int left = scol<gcol ? scol:gcol;
    int right = scol < gcol ? gcol:scol;
    for(int j=left;j<=right;j++){
        if(g_map[srow][j]=='#')return true;
    }
    return false;
}
/*
 判断垂直方向上是否有障碍物
 @param src_point 起点
 @param des_point 终点
 @return true 起点和终点之间有障碍物
 */
bool col_obstacle(tuple<int,int> a,tuple<int,int> b){
    int srow = get<0>(a), scol = get<1>(a);
    int grow = get<0>(b), gcol = get<1>(b);
    int left = srow<grow ? srow:grow;
    int right = srow < grow ? srow:grow;
    for(int i=left;i<=right;i++){
        if(g_map[i][scol]=='#')return true;
    }
    return false;
}
/*
在100*100的字符矩阵中，给出起点和终点，判断连线是否有障碍物
@param src_point 起点坐标
@param des_point 终点坐标
return true 连线有障碍物
return false 连线没有障碍物
 */
bool AStar::obstacle_in_line(Point* src_point,Point* des_point,bool has_product) { //
    tuple<int,int> src = {src_point->coordinate.row,src_point->coordinate.col};
    tuple<int,int> des = {des_point->coordinate.row,des_point->coordinate.col};
    vector<tuple<int,int>> vec = swap_point(src,des);
    tuple<int,int> a = vec[0];
    tuple<int,int> b = vec[1];
    //两个点如果邻近，说明没有障碍物
    if(is_closed(a,b))return false;
    int srow = get<0>(a), scol = get<1>(a);
    int grow = get<0>(b), gcol = get<1>(b);

    //水平方向看是否有障碍物
    if(srow == grow)return row_obstacle(a,b);
    //垂直方向看是否有障碍物
    if(scol == gcol)return col_obstacle(a,b);

    //求两个点之间的直线方程
    float k = (gcol-scol)*1.0/(grow-srow)*1.0;
    float bb = scol*1.0 - k*srow;

    for(int db = -2; db < 3;db++){
        for(int r = srow; r <= grow;r++){
            int c = int(k*r+bb + db);
            int pre_r = r - 1;
            int pre_c = int(k*pre_r + bb +db);

//            if (row_obstacle({pre_r,pre_c},{pre_r,c}) || col_obstacle({pre_r+1,pre_c},{pre_r,c}) || near_obstacle(r,c)){
//                return true;
//            }
            if(g_map[r][c] == '#' || near_obstacle(r,c))return true;
        }
    }

    return false;
}