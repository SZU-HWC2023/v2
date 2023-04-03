//地图相关类
#include "class.h"


Map::Map(){
    this->map.fill({'.'});
}



char Map::operator[](vec2_int pos){
    return this->map[pos.row][pos.col];
}

char Map::operator[](vec2 pos){
    return this->operator[](pos.toIndex());
}

array<char,MAP_TRUE_SIZE> Map::operator[](int row){
    return this->map[row];
}


RawMap::RawMap(){
    Map();
}


/*
判断pos是否位于障碍物内
@param pos: 坐标
@return true 为位于障碍物内
@return false 为不位于障碍物内
*/

bool RawMap::isObstacle(vec2 pos){
    vec2_int pos_idx = pos.toIndex();
    return this->isObstacle(pos_idx);
}

/*
判断pos_idx是否位于障碍物内
@param pos_idx: 地图索引
@return true 为位于障碍物内
@return false 为不位于障碍物内
*/

bool RawMap::isObstacle(vec2_int pos_idx){
    return this->operator[](pos_idx) == '#';
}

/*
计算螺旋遍历的初始朝向
@param pos: 坐标
@return 0 为向上
@return 1 为向右
@return 2 为向下
@return 3 为向左
*/
int initDirection(vec2 pos){
    vec2 d = pos - pos.toCenter();
    if(d.y + d.x > 0){
        if(d.y - d.x>0)
            return 0;
        else
            return 1;
    }
    else{
        if(d.y - d.x>0)
            return 3;
        else
            return 2;        
    }
}

float minDist2Obstacle(vec2 pos, vec2_int obstacle){
    vec2_int quadrant = toQuadrant(pos, obstacle.toCenter());
    vec2 vertice = obstacle.vertice(quadrant);
    return calcDistance(pos, vertice);
}

vec2_int directions[] = {{0,1}, {1,0}, {0,-1}, {-1,0}};


float RawMap::dist2Obstacle(vec2 pos){
    vec2_int start_idx = pos.toIndex();
    vec2_int p = start_idx;
    int dir = 0, steps = 1, step = steps, turns = 0;
    // perform spiral search from center to chebyshevdist 2 
    while(p.chebyshevDist(start_idx) <=2){
        if(this->isObstacle(p))
            return minDist2Obstacle(pos, p);
        p += directions[dir];
        step--;
        if(step == 0){
            dir = (dir + 1) % 4;
            turns++;
            if(turns == 2){
                turns = 0;
                steps++;
            }
            step = steps;
        }
    }
    return 1000.0f;
}
