//机器人相关函数

#include "class.h"


/*
@param robotID 机器人ID
@param x 机器人坐标x
@param y 机器人坐标y
*/
Robot::Robot(int robotID, float x, float y){
    this->id = robotID;
    this->coordinate = {x, y};
    this->path->points.clear();
    // 初始化机器人列表
    this->initOtherRobot();
    this->ban = false;
    // 初始化避让信息
    initAvoidInfo();
}
void Robot::initAvoidInfo(){
    this->av_info.avoiding = false;
    this->av_info.robots_id.clear();
}
/*
更新机器人帧状态
@param f 机器人帧信息
*/
void Robot::update(robot_frame f){
    this->coordinate = {f.x, f.y};
    this->heading = f.heading;
    this->angular_speed = f.ang_spd;
    this->linear_speed = {f.lin_spd_x, f.lin_spd_y};
    this->item_carried = f.item_carried;
    this->workshop_located = f.workshop_located;

    this->crt_radius = this->item_carried? ROBOT_CARRY_RADIUS:ROBOT_NORM_RADIUS;

    this->crt_mass = M_PI * powf(this->crt_radius,2) * ROBOT_DENSITY;

    this->crt_lin_acc = MAX_TRACTION / this->crt_mass;
    this->crt_lin_acc =  2*MAX_TORQUE/this->crt_mass/powf(this->crt_radius,2);
}

//初始化其他机器人列表
void Robot::initOtherRobot(){
    this->other_robots.clear();
    for(auto r:g_robots){
        if(r->id != this->id){
            this->other_robots.push_back(r);
        }
    }
}



/*
执行前进指令
@param tgtSpd 目标速度 (m/s) [-2,6]
*/
void Robot::forward(float tgtSpd){
    printf("forward %d %.6f\n", this->id, tgtSpd);
}

/*
执行旋转指令
@param tgtAngSpd 目标角速度 (rad/s) [-pi,pi]
*/
void Robot::rotate(float tgtAngSpd){
    printf("rotate %d %.6f\n", this->id, tgtAngSpd);
}

/*执行购买指令*/
void Robot::buy(){
    printf("buy %d\n", this->id);
}

/*执行卖出指令*/
void Robot::sell(){
    printf("sell %d\n", this->id);
}

/*执行摧毁指令*/
void Robot::destory(){
    printf("destory %d\n", this->id);
}


/*
计算pos到墙壁的距离
@param pos 位置
@return 距离，x为到左右墙壁的距离，y为到上下墙壁的距离
*/
vec2 wallDist(vec2 pos){
    vec2 dist;
    dist.x = min(pos.x, MAP_SIZE-pos.x);
    dist.y = min(pos.y, MAP_SIZE-pos.y);
    return dist;
}

/*
判断能否在刹车距离内刹住
@param brake_dist 刹车距离 (m)
@return true 能刹住
@return false 不能刹住
*/
bool Robot::isAble2Brake(float brake_dist){
    vec2 brake = fromPolar(brake_dist, this->heading);  //刹车距离向量
    vec2_int hdg_sign = toQuadrant(brake);                  //方位角象限
    vec2_int wall_sign = toQuadrant(this->coordinate, {MAP_SIZE/2, MAP_SIZE/2});  //墙壁象限

    //如果方位角存在朝向墙壁的分量
    if(hdg_sign.row == wall_sign.row || hdg_sign.col == wall_sign.col){
        //而且距离墙壁距离小于刹车距离在该方向的分量
        vec2 dist = wallDist(this->coordinate);
        if(abs(brake.x)>dist.x || abs(brake.y)>dist.y)
            return false;   //不能刹住
    }
    return true;    //能刹住
}
/*
根据字符矩阵的坐标，计算实际的坐标, 地图的左上角为(0,0),地图的右下角为（99，0）
@param i 字符矩阵的第i行，从上往下数
@param j 字符矩阵的第j行，从左往右数
 */
inline vec2 getXY(int i, int j){
    return {0.5f * j + 0.25f, 49.75f - 0.5f * i};
}
/*
根据实际坐标，计算在字符矩阵的坐标, 地图的左上角为(0.25,49.75),地图的右下角为（0.25，49.75）
@param x，实际坐标中的x轴的距离从上往下数
@param y 实际坐标中的y轴的距离，从左往右数
 */
inline vec2 GetPoint(float x, float y){
    return {2*(49.75f - y),2*(x-0.25f)};
}

/*
初始化机器人路径导航
@path 路径 Vector
@size 
 */
void Robot::initPath(vector<Point*> points){
//    for(auto iter:points){
//        this->path->points.push_back(iter);
//    }
    for(int i=1;i<points.size();i++){
        this->path->points.push_back(points[i]);
    }
    this->path->iter=this->path->points.begin();
}
/*
没有路时开辟一条道路
@w 目标工作站
 */
void Robot::allocate_path(Workstation* w){
    vec2_int s = this->coordinate.toIndex();
    vec2_int g = w->coordinate.toIndex();
    vector<Point*> result = g_astar->planning(int(s.row),int(s.col),int(g.row),int(g.col), this->item_carried!=0);
    // 初始化路径
    initPath(result);
}
void Robot::BFS_avoid_robot(){
    // 避让成功点的几个要素
    // 1. 首先需要知道需要避让的对象是谁，在哪一点开始避让，避让对象可能不止一个
    // 2. 寻找一个可以成功避让的点
    // 3. 避让持续到避让对象顺利通过避让点为止
    // 4. BFS优先找最近的避让点 A*
    
}
// 判断是否会出现走独木桥需要避让的情况
bool Robot::judge_need_avoid(Robot* r2){
    
    return false;
}
void Robot::addPathPoint(vector<Point*> result){
    path->points.erase(path->iter++);
    for(int i = result.size()-1; i>=0; i--){
        path->iter = path->points.insert(path->iter, result[i]);
    }
}

void Robot::avoidPointsAdd(Point *p){
    // 先判断有没有撞墙
    // 这段代码目前存在bug 图四会死
    bool carry = item_carried==0?false:true;
    if(g_map.obstacle_in_line(coordinate.toIndex(), p->coordinate, carry)){
        vec2_int s = this->coordinate.toIndex();
        vec2_int g = p->coordinate;
        vector<Point*> result = g_astar->planning(int(s.row),int(s.col),int(g.row),int(g.col), this->item_carried!=0);
        addPathPoint(result);
    }
    
}

/*
获得机器人行动的导航点
@ws 目标工作站
 */
Point* Robot::getNaviPoint(Workstation* w){
    vec2_int g = w->coordinate.toIndex();
    //路径为空，为机器人规划一条前往工作台ws的路径
    if(this->path->points.empty()){
        // 判断数据结构中有没有 没有再取
        vec2_int s = {-1, -1};
        if(workshop_located != -1) s = g_workstations[this->workshop_located]->coordinate.toIndex();
        if(this->item_carried == 0){ //未携带产品
            if(s.row !=-1 && g_astar_path.count({s.row, s.col, g.row, g.col})>0){
                initPath(g_astar_path[{s.row, s.col, g.row, g.col}]);
            }else{
                // 数据结构中没有路径 规划路径
                this->allocate_path(w);
            }
        }else{ //携带产品
            if(s.row !=-1 && g_astar_product_path.count({s.row, s.col, g.row, g.col})>0){
                initPath(g_astar_product_path[{s.row, s.col, g.row, g.col}]);
            }else{
                // 数据结构中没有路径 规划路径
                this->allocate_path(w);
            }
        }
    }
    vec2 w_coor = w->coordinate;       // 目标工作台的坐标
    auto &iter = this->path->iter;
    if(this->path->points.empty())return nullptr;
    list<Point*> &points = this->path->points;
    Point* p = *iter;                 // 导航点
    vec2 des = p->coordinate.toCenter();           // 坐标对应的地图中的位置(浮点)
    // 判断是否会相撞 撞墙 机器人独木桥 并向list中加入避让算法
    avoidPointsAdd(p);
    // 没到工作台且到了导航点附近 iter++
    auto iter_end = points.end();
    if(iter!=(--iter_end)&&calcDistance(des,this->coordinate) < crt_radius*3){
        iter++;
        p = *iter;
    }
    return p;
}

// 上下左右是墙 偏移0.25
vec2 Robot::judgeWallDirection(Point *point){
    vec2_int coor = point->coordinate;
    vec2 res = coor.toCenter();
    float bias = 0.25;
    // // 上边是墙
    // if(g_map[point->coordinate.row+1][point->coordinate.col]){
    //     res.y -= bias;
    // }
    // // 下边是墙
    // if(g_map[point->coordinate.row-1][point->coordinate.col]){
    //     res.y += bias;
    // }
    // // 左边是墙
    // if(g_map[point->coordinate.row][point->coordinate.col-1]){
    //     res.x += bias;
    // }
    // // 右边是墙
    // if(g_map[point->coordinate.row][point->coordinate.col+1]){
    //     res -= bias;
    // }
    // 左上是墙
    if(g_map[point->coordinate.row+1][point->coordinate.col-1]){
        res.y -= bias;
        res.x += bias;
    }
    // 右上是墙
    if(g_map[point->coordinate.row+1][point->coordinate.col+1]){
        res.y -= bias;
        res.x -= bias;
    }
    // 左下是墙
    if(g_map[point->coordinate.row-1][point->coordinate.col-1]){
        res.y += bias;
        res.x += bias;
    }
    // 右下是墙
    if(g_map[point->coordinate.row-1][point->coordinate.col+1]){
        res.y += bias;
        res.x -= bias;
    }
    return res;
}

void Robot::move2ws(Workstation* ws){
    Point* p = getNaviPoint(ws);        // 获取当前路径的导航点
    if(p== nullptr)return;  //这行别删了
    vec2 v = judgeWallDirection(p);
    vec2 tgt_pos = v;  //目标位置
    float tgt_lin_spd = this->linear_speed.len(), tgt_ang_spd = this->angular_speed;    //线速度和角速度

    float dist2ws = calcDistance(this->coordinate, tgt_pos);    //距离工作台距离
    float tgt_hdg = calcHeading(this->coordinate, tgt_pos);     //目标方位角
    float delta_hdg = clampHDG(tgt_hdg - this->heading);        //方位角差

    // simple——demo中的速度角度控制方式
    const double maxRotateSpeed = (delta_hdg > 0 ? MAX_ANGULAR_SPD : -MAX_ANGULAR_SPD);
    if (abs(delta_hdg) < MIN_ANGLE) { // 如果朝向和目标点的夹角很小，直接全速前进
        tgt_lin_spd = MAX_FORWARD_SPD;
        tgt_ang_spd = 0.01;
    }else {
        if (abs(delta_hdg)>M_PI/2) {
            // 角度太大，全速扭转
            // 速度控制小一点，避免靠近不了工作台
            tgt_lin_spd = 1;
            tgt_ang_spd = maxRotateSpeed;
        } else {
            tgt_lin_spd = MAX_FORWARD_SPD * pow(cos(abs(delta_hdg)), 2); // 前进速度随角度变小而变大
            tgt_ang_spd = maxRotateSpeed * sin(abs(delta_hdg));    // 旋转速度随角度变小而变小
        }
    }
    
     if(abs(abs(delta_hdg) - M_PI/2) < 0.1)
         tgt_lin_spd = this->linear_speed.len()/sqrtf(1.2);
    
//    //刹车距离 ----判断刹车
//    float brake_dist = powf(this->linear_speed.len(), 2) / (2 * this->crt_lin_acc);
//    brake_dist += 2*this->crt_radius + 0.05;
//    if(!isAble2Brake(brake_dist))
//        tgt_lin_spd = 0;
//    tgt_lin_spd = 1;
//    tgt_ang_spd = 0;

    this->forward(tgt_lin_spd);
    this->rotate(tgt_ang_spd);

}
/*对机器人的动作进行重置*/
void Robot::resetAction(){
    this->action = {-1,-1};
    this->next_worker_id = -1;
    this->path->points.clear();
}
/*
获得机器人当前的动作
@return tuple<int,int> 元组的第一项为工作台id, 元组的第二项为物品的编号（1-7）
*/
const tuple<int, int> Robot::getAction(){
    return this->action;
}
/*设置机器人的动作*/
void Robot::setAction(tuple<int, int> action){
    this->action = action;
}
/*
设置机器人下一个要去的工作台
@param id 下一个工作台的id
 */
void Robot::setNextWorkerId(int id){
    this->next_worker_id = id;
}
/*
获取机器人下一个工作台id
@return 下一工作台的id
 */
int Robot::getNextWorkerId(){
    return this->next_worker_id;
}