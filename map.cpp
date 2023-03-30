//地图相关类
#include "class.h"


Map::Map(array<array<char, MAP_TRUE_SIZE>, MAP_TRUE_SIZE> map_){
    this->map = map_;
}

/*
判断pos是否位于障碍物内
@param pos: 坐标
@return true 为位于障碍物内
@return false 为不位于障碍物内
*/
bool Map::isObstacle(vec2 pos){
    vec2_int pos_idx = pos.toIndex();
    return this->isObstacle(pos_idx);
}

/*
判断pos_idx是否位于障碍物内
@param pos_idx: 地图索引
@return true 为位于障碍物内
@return false 为不位于障碍物内
*/
bool Map::isObstacle(vec2_int pos_idx){
    return this->map[pos_idx.x][pos_idx.y] == '#';
}

int initDirection(vec2 pos){

}

float Map::dist2Obstacle(vec2 pos){

}

bool Map::isCollide2Obstacle(Robot* robot){
    vec2 pos = robot->coordinate;
    if(this->isObstacle(pos))return true;


}