#include "class.h"
bool judgeAroundObstacle(int x, int y){
    if(g_map[x+1][y] == '#' || g_map[x][y+1] == '#' || g_map[x-1][y] == '#' || g_map[x][y-1] == '#') return true;
    if(g_map[x+1][y+1] == '#' || g_map[x-1][y-1] == '#' || g_map[x-1][y+1] == '#' || g_map[x+1][y-1] == '#') return true;
    return false;
}
vector<Point*> AStar::planning(int sx,int sy,int gx,int gy, bool has_product){
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
            float tmp_cost = this->calc_heuristic(p,goal_node)+p->cost;
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
            float baseCost = get<2>(mot);
            if(judgeAroundObstacle(current->x + get<0>(mot),current->y + get<1>(mot)))baseCost *=8;
            Point* point = new Point(current->x + get<0>(mot),current->y + get<1>(mot),current->cost+baseCost,current);
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

    return this-> simplify_path(result,has_product); //路径简化
//    return result;
}
//判断下标是否合法, has_product为true时表示机器人有东西

bool AStar::verify(Point* from,Point* p,bool has_product){
    //下标超出地图
    if(p->x<0 || p->y<0 ||p->x>=MAP_TRUE_SIZE||p->y>=MAP_TRUE_SIZE)return false;
    //p所处位置为墙
    if(g_map[p->x][p->y] == '#')return false;
    //不拿东西走不通
    if(g_map[p->x][p->y] == '!')return false;
    //墙角不访问
    if(g_map[p->x][p->y] == '$')return false;
    if(has_product){
        //机器人有产品时
        if(g_map[p->x][p->y] == '@')return false;
    }
    int p_x = p->x;
    int p_y = p->y;

    for(tuple<int,int, float> mot:this->motion){
        int dx = p_x + get<0>(mot);
        int dy = p_y + get<1>(mot);
        if(dx<0 || dy<0 || dx>=MAP_TRUE_SIZE || dy >= MAP_TRUE_SIZE)continue;
        if(g_map[dx][dy] == '#')return false;
    }

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

bool help(Point* p){
    int x = p->x;
    int y = p->y;
    int sum = 0;
    for(int j=y-1;j>=0;j--){
        if(g_map[x][j]!='#')sum++;
        else{
            break;
        }
    }
    for(int j = y+1;j<MAP_TRUE_SIZE;j++){
        if(g_map[x][j]!='#')sum++;
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
    if(!obstacle_in_line(vec_points[0],vec_points.back(),has_product))return {vec_points[0],vec_points.back()};
    vector<Point*> result;
    result.emplace_back(vec_points[0]);
    for(int i=1;i<vec_points.size()-1;i++){
        if(g_map[vec_points[i]->x][vec_points[i]->y]=='@' ||help(vec_points[i])){
            result.emplace_back(vec_points[i]);
            continue;
        }
        if(obstacle_in_line(result.back(),vec_points[i],has_product)){
            result.emplace_back(vec_points[i]);
        }
    }
    result.emplace_back(vec_points.back());
    return result;
}

void AStar::divide_conquer(vector<Point*> &result,int left,int right,vector<Point*> &vec_points,bool has_product){
   if(left > right)return;
   if(left == right)return;
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

/*
 点周围是否有障碍物 上、下、左、右、左上、左下、右上、右下
 @param x 字符矩阵的行索引
 @param y 字符矩阵的列索引
 @return true （x,y）附近有障碍物

 */
bool near_obstacle(int x,int y){
    if(g_map[x][y] == '#')return true;
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
        int dx = x + get<0>(mot);
        int dy = y + get<1>(mot);
        if(g_map[dx][dy]=='#')return true;
    }
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
    int sx = get<0>(a), sy = get<1>(a);
    int gx = get<0>(b), gy = get<1>(b);
    int left = sy<gy ? sy:gy;
    int right = sy < gy ? gy:sy;
    for(int j=left;j<=right;j++){
        if(g_map[sx][j]=='#')return true;
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
    int sx = get<0>(a), sy = get<1>(a);
    int gx = get<0>(b), gy = get<1>(b);
    int left = sx<gx ? sx:gx;
    int right = sx < gx ? gx:sx;
    for(int i=left;i<=right;i++){
        if(g_map[i][sy]=='#')return true;
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
    tuple<int,int> src = {src_point->x,src_point->y};
    tuple<int,int> des = {des_point->x,des_point->y};
    vector<tuple<int,int>> vec = swap_point(src,des);
    tuple<int,int> a = vec[0];
    tuple<int,int> b = vec[1];
    //两个点如果邻近，说明没有障碍物
    if(is_closed(a,b))return false;
    int sx = get<0>(a), sy = get<1>(a);
    int gx = get<0>(b), gy = get<1>(b);

    //水平方向看是否有障碍物
    if(sx == gx)return row_obstacle(a,b);
    //垂直方向看是否有障碍物
    if(sy == gy)return col_obstacle(a,b);

    //求两个点之间的直线方程
    float k = (gy-sy)*1.0/(gx-sx)*1.0;
    float bb = sy*1.0 - k*sx;

    //看起点到终点的直线是否有障碍物
    for(int dx = sx; dx <= gx;dx++){
        int dy = int(k*dx+bb);
        int pre_dx = dx - 1;
        int pre_dy = int(k*pre_dx + bb);

        if (row_obstacle({pre_dx,pre_dy},{pre_dx,dy}) || col_obstacle({pre_dx+1,pre_dy},{pre_dx,dy}) || near_obstacle(dx,dy)){
            return true;
        }
        if(g_map[dx][dy] == '#')return true;
    }
    return false;
}