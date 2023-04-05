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

// 计算向量的叉积
int cross_product(vec2_int a, vec2_int b, vec2_int c) {
    int c1 = b.col - a.col;
    int r1 = b.row - a.row;
    int c2 = c.col - a.col;
    int r2 = c.row - a.row;
    return c1 * r2 - c2 * r1;
}

// 判断线段AB和CD是否相交
bool is_intersect(vec2_int a, vec2_int b, vec2_int c, vec2_int d) {
    int cp1 = cross_product(a, b, c);
    int cp2 = cross_product(a, b, d);
    int cp3 = cross_product(c, d, a);
    int cp4 = cross_product(c, d, b);
    if (cp1 * cp2 < 0 && cp3 * cp4 < 0) {
        return true;
    }
    if (cp1 == 0 && cp2 == 0 && cp3 == 0 && cp4 == 0) {
        // 两条线段共线
        if (b.col < a.col) {
            swap(a, b);
        }
        if (d.col < c.col) {
            swap(c, d);
        }
        return !(b.col < c.col || d.col < a.col);
    }
    return false;
}
// 判断线段是否平行
bool is_parallel(vec2_int a, vec2_int b, vec2_int c, vec2_int d) {
    bool res = false;
    if (b.col == a.col) {
        if (d.col == c.col) {
            // 两条线段都垂直于x轴
            res = true;
        } else {
            // 只有线段AB垂直于x轴
            res = false;
        }
    } else if (d.col == c.col) {
        // 只有线段CD垂直于x轴
        res = false;
    } else {
        // 计算两条线段的斜率
        double k1 = (double)(b.row - a.row) / (b.col - a.col);
        double k2 = (double)(d.row - c.row) / (d.col - c.col);
        res = fabs(k1 - k2) < 1e-6;
    }
    // 如果两线段平行还要判断是否有平行交会
    if(res){
        int l1_min_c = min(a.col, b.col), l1_min_r = min(a.row, b.row), l1_max_c = max(a.col, b.col), l1_max_r = max(a.row, b.row);
        int l2_min_c = min(c.col, d.col), l2_min_r = min(c.row, d.row), l2_max_c = max(c.col, d.col), l2_max_r = max(c.row, d.row);
        if(l1_max_c <= l2_min_c || l2_max_c <= l1_min_c || l1_max_r <= l2_min_r || l2_max_r <= l1_min_r){
            res = false;
        }
    }
    return res;
}
bool judgeInterOrPara(vec2_int a, vec2_int b, vec2_int c, vec2_int d){
    return is_intersect(a, b, c, d)||is_parallel(a, b, c, d);
}