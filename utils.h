//存放全局调用的计算函数，不引用其他头文件

#include <iostream>
#include <array>
#include <math.h>
#include <vector>
#include "setting.h"
#define UTILS_H

using namespace std;

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
};



float sign(float x);
float func_f(float x, float maxX, float minRate);
float timeValue(float frames);
float collisionValue(float impulse);

float calcHeading(vec2 pos1, vec2 pos2);
float calcDistance(vec2 pos1, vec2 pos2);
float calcTgtAngSpd(float deltaHDG);
float clampHDG(float hdg);

vec2 fromPolar(float len, float hdg);
vec2 toQuadrant(vec2 pos, vec2 center={0., 0.});