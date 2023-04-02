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
        for(tuple<int,int,float> mot:this->motion){
            int baseCost = get<2>(mot);
            // 路径节点靠墙 代价翻倍
            if(judgeAroundObstacle(current->x + get<0>(mot), current->y + get<1>(mot))) baseCost*=4;
            Point* point = new Point(current->x + get<0>(mot),current->y + get<1>(mot),current->cost+baseCost, current);
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
    return this-> simplify_path(result);
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
vector<Point *> AStar::simplify_path(vector<Point*> &vec_points){
//    fprintf(stderr,"LYW\n");
    if(vec_points.size()<=2)return vec_points;
    if(!obstacle_in_line(vec_points[0], vec_points.back())) return {vec_points[0],vec_points.back()};

    vector<Point*> result;
//    fprintf(stderr,"WLY\n");
    divide_conquer(result,0,vec_points.size()-1,vec_points);
//    fprintf(stderr,"WLY\n");
    result.emplace_back(vec_points.back());
//    fprintf(stderr,"WLY\n");
    return result;
}

void AStar::divide_conquer(vector<Point*> &result,int left,int right,vector<Point*> &vec_points){
   if(left > right)return;
   if(left == right)return;
   if(left+1 == right){
       result.emplace_back(vec_points[left]);
       return;
   }
//   fprintf(stderr,"len:%d %d\n",left,right);
   if(!obstacle_in_line(vec_points[left], vec_points[right])){
       result.emplace_back(vec_points[left]);
       return;
   }
   int mid = (right-left)/2 + left;
   divide_conquer(result,left,mid,vec_points);
   divide_conquer(result,mid,right,vec_points);
}
/*
在100*100的字符矩阵中，给出起点和终点，判断连线是否有障碍物
@param src_point 起点坐标
@param des_point 终点坐标
return true 连线有障碍物
return false 连线没有障碍物
 */
bool AStar::obstacle_in_line(Point* src_point,Point* des_point) { //

    int sx = src_point->x;
    int sy = src_point->y;
    int gx = des_point->x;
    int gy = des_point->y;

    //1. 判断两个坐标点的x轴是否相同
    if(sx == gx || abs(sx-gx)==1){
        if(sy<gy){
            for(int j = sy+1;j<=gy;j++){
                if(g_map[sx][j] == '#')return true;
            }
        }else{
            for(int j = gy;j<=sy;j++){
                if(g_map[sx][j] == '#')return true;
            }
        }
    }
    //2. 判断两个坐标点的y轴是否相同
    if(sy == gy|| abs(sy-gy)==1){
        if(sx < gx){
            for(int i= sx+1;i<=gx;i++){
                if(g_map[i][sy] == '#')return true;
            }
        }else{
            for(int i= gx+1;i<=sx;i++){
                if(g_map[i][sy] == '#')return true;
            }
        }
    }
    //3. 求两个点之间的直线方差
    float k = (gy-sy)*1.0/(gx-sx)*1.0;
    float b = sy*1.0 - k*sx;
    if(sx < gx){

        for(int dx = sx+1;dx <= gx;dx++){
            int dy = int(k*dx + b);
            if(g_map[dx][dy]=='#')return true;
            if(dy>0&&g_map[dx][dy-1] == '#')return true;
            if(dy<MAP_TRUE_SIZE-1 && g_map[dx][dy+1] == '#')return true;

        }
    }else{
        for(int dx = gx+1;dx<=sx;dx++){
            int dy = int(k*dx+b);
            if(g_map[dx][dy] == '#')return true;
            if(dy>0&&g_map[dx][dy-1] == '#')return true;
            if(dy<MAP_TRUE_SIZE-1 && g_map[dx][dy+1] == '#')return true;
        }
    }
    return false;
}