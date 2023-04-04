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
        // 初始化机器人列表
    this->initOtherRobot();
    this->path->points.clear();
    this->trajectory.clear();
    this->dwa = new DWA(this);
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
    this->crt_ang_acc =  2*MAX_TORQUE/this->crt_mass/powf(this->crt_radius,2);

    DWA_state s(this);
    this->trajectory = s.calcTrajectory({this->linear_speed.len(), this->angular_speed},PRED_T);
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
void BFS_avoid_robot(){

}

bool judge_need_avoid(Robot *r1, Robot* r2){
    float dis = calcDistance(r1->coordinate, r2->coordinate);
    // float angle_diff = 
    // if()
    
}
void Robot::avoidPointsAdd(){
    // 先判断有没有撞墙
    


    // 再判断是否存在机器人走独木桥的情况

    for(Robot* o_r:this->other_robots){
        // 判断是否会出现走独木桥的情况
        if(judge_need_avoid(this, o_r)){

        }
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
//                fprintf(stderr,"机器人：%d 未带产品分配路径, 目的工作台：%d 类型:%d\n",this->id,w->id,w->type);
            }
        }else{ //携带产品
            if(s.row !=-1 && g_astar_product_path.count({s.row, s.col, g.row, g.col})>0){
                initPath(g_astar_product_path[{s.row, s.col, g.row, g.col}]);
            }else{
                // 数据结构中没有路径 规划路径
                this->allocate_path(w);
//                fprintf(stderr,"机器人：%d 带产品分配路径 目的工作台：%d 类型：%d\n",this->id,w->id,w->type);
            }
        }
    }
    vec2 w_coor = w->coordinate;       // 目标工作台的坐标
    auto &iter = this->path->iter;
    if(this->path->points.empty())return nullptr;
    list<Point*> &points = this->path->points;
    Point* p = *iter;                 // 导航点
    vec2 des = p->coordinate.toCenter();           // 坐标对应的地图中的位置(浮点)
    // 没到工作台且到了导航点附近 index++
    auto iter_end = points.end();
    if(iter!=(--iter_end)&&calcDistance(des,this->coordinate) < crt_radius){
        iter++;
        p = *iter;
    }
    // 判断是否会相撞 撞墙 机器人独木桥 并向list中加入避让算法
    avoidPointsAdd();
    return p;
}

void Robot::move2ws(Workstation* ws){
    Point* p = getNaviPoint(ws);        // 获取当前路径的导航点
    if(p== nullptr)return;  //这行别删了
    vec2 v = p->coordinate.toCenter();
    vec2 tgt_pos = v;  //目标位置
    VW desire = this->dwa->find_vw(v);

    this->forward(desire.v);
    this->rotate(desire.w);
    
//      if(abs(abs(delta_hdg) - M_PI/2) < 0.1)
//          tgt_lin_spd = this->linear_speed.len()/sqrtf(1.2);
    
// //    //刹车距离 ----判断刹车
// //    float brake_dist = powf(this->linear_speed.len(), 2) / (2 * this->crt_lin_acc);
// //    brake_dist += 2*this->crt_radius + 0.05;
// //    if(!isAble2Brake(brake_dist))
// //        tgt_lin_spd = 0;
// //    tgt_lin_spd = 1;
// //    tgt_ang_spd = 0;

    // this->forward(tgt_lin_spd);
    // this->rotate(tgt_ang_spd);

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