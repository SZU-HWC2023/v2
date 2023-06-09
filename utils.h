//存放全局调用的计算函数，不引用其他头文件

#include <iostream>
#include <array>
#include <math.h>
#include <vector>
#include "setting.h"
#define UTILS_H

using namespace std;
struct vec2_int;
struct vec2;

//整型二维矢量（用于地图索引）
struct vec2_int{
    int row;        // 对应的y
    int col;        // 对应的x

    vec2_int(int row_=0, int col_=0) : row(row_), col(col_){}

    vec2_int& operator+=(vec2_int& v){
        row += v.row;
        col += v.col;
        return *this;
    }

    bool operator<(const vec2_int& v) const{
        return row < v.row || (row == v.row && col < v.col);
    }

    bool operator==(const vec2_int& v) const{
        return row == v.row && col == v.col;
    }

    //返回地图索引对应的中心坐标
    vec2 toCenter();

    //返回到p的切比雪夫距离
    int chebyshevDist(const vec2_int& p){
        return max(abs(row-p.row), abs(col-p.col));
    }

    //返回地图方格在对应象限的顶点坐标，例如第一象限为右上顶点
    vec2 vertice(vec2_int quadrant);


};

//二维矢量
struct vec2{
    float x;   //x坐标
    float y;   //y坐标

    /*
    从直角坐标构造二维矢量
    */
    vec2(float x_=0.0f, float y_=0.0f) : x(x_), y(y_){}

    vec2 operator+(const vec2& v) const{
        return {x+v.x, y+v.y};
    }

    template<typename T>
    vec2 operator+(const T& v) const{
        return {x+v, y+v};
    }

    vec2 operator-(const vec2& v) const{
        return {x-v.x, y-v.y};
    }

    template<typename T>
    vec2 operator-(const T& v) const{
        return {x-v, y-v};
    }

    vec2 operator*(const vec2& v) const{
        return {x*v.x, y*v.y};
    }

    template<typename T>
    vec2 operator*(const T& v) const{
        return {x*v, y*v};
    }

    vec2 operator/(const vec2& v) const{
        return {x/v.x, y/v.y};
    }

    template<typename T>
    vec2 operator/(const T& v) const{
        return {x/v, y/v};
    }

    vec2& operator+=(const vec2& v){
        x += v.x;
        y += v.y;
        return *this;
    }

    template<typename T>
    vec2& operator+=(const T& v){
        x += v;
        y += v;
        return *this;
    }

    vec2& operator-=(const vec2& v){
        x -= v.x;
        y -= v.y;
        return *this;
    }

    template<typename T>
    vec2& operator-=(const T& v){
        x -= v;
        y -= v;
        return *this;
    }

    template<typename T>
    vec2& operator*=(const T& v){
        x *= v;
        y *= v;
        return *this;
    }

    //返回矢量长度
    float len() const{
        return sqrtf(x*x + y*y);
    }

    //返回矢量方位角
    float hdg() const{
        return atan2(y, x);
    }

    //返回该坐标在地图上的索引
    vec2_int toIndex() const{
       return {int(y/0.5), int(x/0.5)};
    }

    //返回该坐标所在格的中心坐标
    vec2 toCenter() const{
        vec2_int idx = this->toIndex();
        return idx.toCenter();
    }

    //返回该向量的法线向量
    vec2 normal() const{
        return {-y, x};
    }

    //返回到向量p的夹角(恒正)
    float angle2(const vec2& p) const{
        return acosf((x*p.x + y*p.y)/(len()*p.len()));
    }
    //返回和向量v的叉积
    float cross_product(vec2& v){
        return v.y*x - y*v.x;
    }

};


/*
 * 判断线段AB 与线段CD是否会相交
 * @param point_A 点A
 * @param point_B 点B
 * @param point_C 点C
 * @param point_D 点D
 * @return true 线段AB会和线段CD相交
 * @return false 线段AB不会和线段CD相交
 * */
bool is_line_segment_intersection(vec2 point_A,vec2 point_B,vec2 point_C, vec2 point_D);

int sign(float x);
float func_f(float x, float maxX, float minRate);
float timeValue(float frames);
float collisionValue(float impulse);

float calcHeading(vec2 pos1, vec2 pos2);
float calcDistance(vec2 pos1, vec2 pos2);
float calcTgtAngSpd(float deltaHDG);
float clampHDG(float hdg);

vec2 fromPolar(float len, float hdg);
vec2_int toQuadrant(vec2 pos, vec2 center={0., 0.});
// 判断ab cd是否相交
int cross_product(vec2_int a, vec2_int b, vec2_int c);
bool is_intersect(vec2_int a, vec2_int b, vec2_int c, vec2_int d);
bool is_parallel(vec2_int a, vec2_int b, vec2_int c, vec2_int d);
bool judgeInterOrPara(vec2_int a, vec2_int b, vec2_int c, vec2_int d);