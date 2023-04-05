#include "utils.h"

using namespace std;
vec2 vec2_int::toCenter(){
    return {0.5f * col + 0.25f, 0.5f * row + 0.25f};
}

vec2 vec2_int::vertice(vec2_int quadrant) {
    vec2 this_vec2 = toCenter();
    vec2 quadrant_vec2 = quadrant_vec2.toCenter();
    return {this_vec2.x + quadrant_vec2.x, this_vec2.y + quadrant_vec2.y};
}

// 计算代价系数
float func_f(float x, float maxX, float minRate){
    if(x < maxX)
        return (1-sqrtf(1-powf(1-x/maxX,2))) * (1-minRate) + minRate;
    return minRate;
}

// 时间价值
float timeValue(float frames){
    return func_f(frames, 9000, 0.8);
}

// 碰撞价值
float collisionValue(float impulse){
    return func_f(impulse, 1000, 0.8);
}

/*
符号函数
@param x 输入值
@return 符号值，正数或0返回1，负数返回-1
*/
int sign(float x){
    if(x>=0)
        return 1;
    else
        return -1;
};

/*
计算 pos1->pos2 的方位角
@param pos1 起点
@param pos2 终点
@return 方位角
*/
float calcHeading(vec2 pos1, vec2 pos2){
    float dx = pos2.x - pos1.x;
    float dy = pos2.y - pos1.y;
    float hdg = atan2(dy, dx);
    return hdg;
}

/*
计算 pos1->pos2 的距离
@param pos1 起点
@param pos2 终点
@return 距离
*/
float calcDistance(vec2 pos1, vec2 pos2){
    float dx = pos2.x - pos1.x;
    float dy = pos2.y - pos1.y;
    float dist = sqrtf(dx*dx + dy*dy);
    return dist;
}

/*
由方位角差计算目标角速度
@param deltaHDG 方位角差
@return 目标角速度
*/
float calcTgtAngSpd(float deltaHDG){
    float upper = 5*M_PI/180, lower = 0.01*M_PI/180;
    if(abs(deltaHDG)>upper)
        return M_PI*sign(deltaHDG);
    else if(abs(deltaHDG)<lower)
        return 0;
    else{
        return (abs(deltaHDG)-lower)/(upper - lower) * M_PI * sign(deltaHDG);
    }
}

/*
钳制方位角到 -pi~pi
@param hdg 方位角
@return 钳制后的方位角
*/
float clampHDG(float hdg){
    if(hdg>M_PI)
        return hdg - 2*M_PI;
    else if(hdg<-M_PI)
        return hdg + 2*M_PI;
    else
        return hdg;
}

/*
从极坐标转换为直角坐标
@param len 长度
@param hdg 方位角
@return 直角坐标
*/
vec2 fromPolar(float len, float hdg){
    return {len*cos(hdg), len*sin(hdg)};
}


/*
求坐标pos在以center为原点的坐标系中的象限符号
@param pos 坐标
@param center 原点
@return 坐标pos在以center为原点的坐标系中的象限符号
*/
vec2_int toQuadrant(vec2 pos, vec2 center){
    vec2_int quadrant = {sign(pos.x-center.x), sign(pos.y-center.y)};
    return quadrant;
}
/*
 * 判断线段AB 与线段CD是否会相交
 * @param point_A 点A
 * @param point_B 点B
 * @param point_C 点C
 * @param point_D 点D
 * @return true 线段AB会和线段CD相交
 * @return false 线段AB不会和线段CD相交
 * */
bool is_line_segment_intersection(vec2 point_A,vec2 point_B,vec2 point_C, vec2 point_D){
    vec2 A_B = point_B - point_A;
    vec2 A_C = point_C - point_A;
    vec2 A_D = point_D - point_A;
    vec2 C_D = point_D - point_C;
    vec2 C_A = point_A - point_C;
    vec2 C_B = point_B - point_C;
    float AB_AC = A_B.cross_product(A_C); //向量AB和AC的叉乘
    float AB_AD = A_B.cross_product(A_D); //向量AB和AD的叉乘
    float CD_CA = C_D.cross_product(C_A); //向量CD和CA的叉乘
    float CD_CB = C_D.cross_product(C_B); //向量CD和CB的叉乘
    if( max(point_A.x,point_B.x) < min(point_C.x,point_D.x) || max(point_C.x,point_D.x) < min(point_A.x,point_B.x)||
        max(point_A.y, point_B.y) < min(point_C.y, point_D.y) || max(point_C.y, point_D.y) < min(point_A.y, point_B.y)){
        return false;
    }else if(AB_AC * AB_AD <=0 && CD_CA*CD_CB <=0)return true;
    return false;
}