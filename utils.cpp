#include "utils.h"

using namespace std;

vec2 vec2_int::toCenter(){
    return {0.5f * x + 0.25f, 0.5f * y + 0.25f};
}

vec2 vec2_int::vertice(vec2_int quadrant) {
    return {
        0.5f * x + 0.25f + 0.25f * quadrant.x,
        0.5f * y + 0.25f + 0.25f * quadrant.y
    };
}

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
@return 符号值，正数返回1，负数返回-1
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
@param center 原点，默认为(0,0)
@return 坐标pos在以center为原点的坐标系中的象限符号
*/
vec2_int toQuadrant(vec2 pos, vec2 center){
    return {sign(pos.x-center.x), sign(pos.y-center.y)};
}
