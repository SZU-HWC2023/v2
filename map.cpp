//地图相关类
#include "class.h"


RawMap::RawMap(){
    this->map.fill({'.'});
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
    while(p.chebyshevDist(start_idx) <=4){
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


/*
检测src到des的直线上是否有障碍物
@param src 起点坐标
@param heading 朝向
@param radius 搜索直线半径
@param max_dist 最大距离
@return -1 为没有障碍物
@return 其他值为距离起点的距离
*/
float RawMap::dist2ObstacleAhead(vec2 src, float heading, float radius, float max_dist){
    //遍历步长
    float step = 0.5;

    //方向向量
    vec2 direction = fromPolar(1,heading);
    //长度
    float len = max_dist;
    //步长向量
    vec2 step_vec = direction * step;
    //法线向量
    vec2 normal = direction.normal();
    //起点中心点
    vec2 src_center = src + direction * radius;
    //起点左侧偏移机器人半径点
    vec2 src_left = src + normal * radius;
    //起点右侧偏移机器人半径点
    vec2 src_right = src - normal * radius; 

    for(float l=0;l<len;l+=step){
        //分别判断各个点是否有障碍物
        if(this->isObstacle(src_center))
            return l;
        if(this->isObstacle(src_left))
            return l;
        if(this->isObstacle(src_right))
            return l;
        //更新点
        src_center += step_vec;
        src_left += step_vec;
        src_right += step_vec;
    }
    return -1;
}


/*
在100*100的字符矩阵中，给出起点和终点，判断连线是否有障碍物
@param src_point 起点坐标
@param des_point 终点坐标
@param has_product 是否有携带物品
@param max_dist 最大距离，如果为负数则为到终点的距离
return true 连线有障碍物
return false 连线没有障碍物
 */
bool RawMap::obstacle_in_line(vec2_int src_point,vec2_int des_point,bool has_product, float max_dist) { 
    //两个点如果邻近，说明没有障碍物
    if(src_point.chebyshevDist(des_point) == 1)return false;

    vec2 src = src_point.toCenter(), des = des_point.toCenter();

    float heading = calcHeading(src, des);

    if (max_dist < 0) {
        max_dist = calcDistance(src, des);
    }

    float radius = has_product?ROBOT_CARRY_RADIUS:ROBOT_NORM_RADIUS;

    float obs_dist = this->dist2ObstacleAhead(src, heading, radius, max_dist);

    return obs_dist >= 0;
}

bool RawMap::obstacle_in_line(Point* src_point,Point* des_point,bool has_product){
    return this->obstacle_in_line(src_point->coordinate,des_point->coordinate,has_product);
}