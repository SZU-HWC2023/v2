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
    vec2 hdg_sign = toQuadrant(brake);                  //方位角象限
    vec2 wall_sign = toQuadrant(this->coordinate, {MAP_SIZE/2, MAP_SIZE/2});  //墙壁象限

    //如果方位角存在朝向墙壁的分量
    if(hdg_sign.x == wall_sign.x || hdg_sign.y == wall_sign.y){
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
//为机器人规划一条路径
void Robot::allocate_path(Workstation* w){
    vec2 s = GetPoint(this->coordinate.x,this->coordinate.y);
    vec2 g = GetPoint(w->coordinate.x,w->coordinate.y);
    vector<Point*> result = g_astar->planning(int(s.x),int(s.y),int(g.x),int(g.y),this->item_carried!=0);
    for(int i=1;i<result.size();i++){
        paths.push(result[i]);
    }
}
/*
向工作台前进
@param ws 目标工作台
*/
//if(this->id == 2){
//test_astar(result);
//}
void Robot::move2ws(Workstation* ws){

    vec2 w = ws->coordinate;
    Point* p = nullptr;
    //路径为空，为机器人规划一条前往工作台ws的路径
    if(this->paths.empty()){
        this->allocate_path(ws);
        if(this->paths.size()>=1) p = this->paths.front();
        else return;
    }else{
        Point* des_point = paths.front();
        vec2 des = getXY(des_point->x,des_point->y);
        //判断机器人是否到达路径中的某个点
        if(paths.size()>1&&calcDistance(des,this->coordinate) < 0.5){
            paths.pop();
        }else if(paths.size()==1){//还剩下终点未到达
            if(this->workshop_located!=-1){ //已经到达工作台附近
                paths.pop();
                return;
            }else{  //尚未到达工作台附近

            }

        }
        p = paths.front();
    }

    vec2 v = getXY(p->x,p->y);
    vec2 tgt_pos = v;  //目标位置
    float tgt_lin_spd = this->linear_speed.len(), tgt_ang_spd = this->angular_speed;    //线速度和角速度

    float dist2ws = calcDistance(this->coordinate, tgt_pos);    //距离工作台距离
    float tgt_hdg = calcHeading(this->coordinate, tgt_pos);     //目标方位角
    float delta_hdg = clampHDG(tgt_hdg - this->heading);        //方位角差

    // simple——demo中的速度角度控制方式
    const double maxRotateSpeed = (delta_hdg > 0 ? MAX_ANGULAR_SPD : -MAX_ANGULAR_SPD);
    if (abs(delta_hdg) < MIN_ANGLE) { // 如果朝向和目标点的夹角很小，直接全速前进
        tgt_lin_spd = MAX_FORWARD_SPD;
        tgt_ang_spd = 0.001;
    } else {
        if (abs(delta_hdg) > M_PI/2) {
            // 角度太大，全速扭转
            // 速度控制小一点，避免靠近不了工作台
            tgt_lin_spd = MAX_FORWARD_SPD*0.0001;
            tgt_ang_spd = maxRotateSpeed;
        } else {
            tgt_lin_spd = MAX_FORWARD_SPD * cos(abs(delta_hdg)); // 前进速度随角度变小而变大
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
    this->action = {NULL,-1};
}
/*
获得机器人当前的动作
@return tuple<int,int> 元组的第一项为工作台id, 元组的第二项为物品的编号（1-7）
*/
const tuple<Workstation*, int> Robot::getAction(){
    return this->action;
}
/*设置机器人的动作*/
void Robot::setAction(tuple<Workstation*, int> action){
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