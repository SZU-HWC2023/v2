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
    if(hdg_sign.x == wall_sign.x || hdg_sign.y == wall_sign.y){
        //而且距离墙壁距离小于刹车距离在该方向的分量
        vec2 dist = wallDist(this->coordinate);
        if(abs(brake.x)>dist.x || abs(brake.y)>dist.y)
            return false;   //不能刹住
    }
    return true;    //能刹住
}

/*
向工作台前进
@param ws 目标工作台
*/
void Robot::move2ws(Workstation* ws){
    vec2 tgt_pos = ws->coordinate;  //目标位置
    VW vw = this->dwa->find_vw(tgt_pos);  //计算速度和角速度
    this->forward(vw.v);
    this->rotate(vw.w);
}




// void Robot::move2ws(Workstation* ws){
//     vec2 tgt_pos = ws->coordinate;  //目标位置
//     float tgt_lin_spd = this->linear_speed.len(), tgt_ang_spd = this->angular_speed;    //线速度和角速度

//     float dist2ws = calcDistance(this->coordinate, tgt_pos);    //距离工作台距离
//     float tgt_hdg = calcHeading(this->coordinate, tgt_pos);     //目标方位角
//     float delta_hdg = clampHDG(tgt_hdg - this->heading);        //方位角差

//     // simple——demo中的速度角度控制方式
//     const double maxRotateSpeed = (delta_hdg > 0 ? MAX_ANGULAR_SPD : -MAX_ANGULAR_SPD);
//     if (abs(delta_hdg) < MIN_ANGLE) { // 如果朝向和目标点的夹角很小，直接全速前进
//         tgt_lin_spd = MAX_FORWARD_SPD;
//         tgt_ang_spd = 0.;
//     } else {
//         if (abs(delta_hdg) > M_PI / 2) {
//             // 角度太大，全速扭转
//             // 速度控制小一点，避免靠近不了工作台
//             tgt_lin_spd = MAX_FORWARD_SPD*0.5;
//             tgt_ang_spd = maxRotateSpeed;
//         } else {
//             tgt_lin_spd = MAX_FORWARD_SPD * cos(abs(delta_hdg)); // 前进速度随角度变小而变大
//             tgt_ang_spd = maxRotateSpeed * sin(abs(delta_hdg));    // 旋转速度随角度变小而变小
//         }
//     }
    
//     // if(abs(abs(delta_hdg) - M_PI/2) < 0.1)
//     //     tgt_lin_spd = this->linear_speed.len()/sqrtf(1.2);
    
//     //刹车距离 ----判断刹车
//     float brake_dist = powf(this->linear_speed.len(), 2) / (2 * this->crt_lin_acc);
//     brake_dist += 2*this->crt_radius + 0.05;
//     if(!isAble2Brake(brake_dist))
//         tgt_lin_spd = 0;

//     this->forward(tgt_lin_spd);
//     this->rotate(tgt_ang_spd);
// }
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