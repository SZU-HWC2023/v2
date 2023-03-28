//工作台相关函数
#include "class.h"
/*
工作台是否需要该物品
@param production_id 物品的类型
@return true    工作台需要该物品
@return false   工作台不需要该物品
 */
bool Workstation::production_needed(int production_type){
    return this->need.count(production_type) == 1;

}
/*
物品是否将工作台的原料槽锁住
@param production_type 物品的类型
@return true    物品已经将工作台的原料槽锁住,其它机器人不能再给该工作台送原料
@return false   物品未将工作台的原料槽锁住,其它机器人可以给该工作台送原料
 */
bool Workstation::production_locked(int production_type){
    // 返回true代表物品已经被锁定 不可再指派机器
    if(this->type == 8 || this->type == 9) return false;
    return this->locked.find(production_type) != this->locked.end();
}


/*
物品是否能填充工作台的原料槽  || 工作台是否需要该原料物品用于生产,如果工作台的原料槽已经有了该物品,则不需要再给这个平台送了
该函数使用在
    1. 取货物时计算二层代价(nxt)
    2. 分配放置任务(有物品要去放)
    3. 将要取到物品判断取到物品后是否有地方放(nxt)
    4. 在要去取物品的途中判断要取的物品是否有地方放(nxt)
@param production_id 物品的类型
@return true    物品能填充工作台
@return false   物品不能填充工作台
 */
bool Workstation::can_production_recycle(int production_type){
    return (!production_locked(production_type)) && production_needed(production_type);
}

/*
工作台生产的物品是否能出售
    1. 产品已经生产好 || 生产因输出格满而阻塞, 此时肯定能出售
    2. 产品未生产好, 但剩余帧数大于0时, 只要在机器人到达平台前生产好该产品就认为工作台生产的物品是能出售
@return true    工作台生产的物品能出售
@return false   工作台生产的物品不能出售
 */
bool Workstation::can_production_sell(){
    return this->productStatus == 1 || this->remaingFrames != -1;
}

/*
物品对工作台进行解锁
@param production_type 物品的类型
 */
void Workstation::rel_locked_production(int production_type){
    this->locked.erase(production_type);
}
/*
物品对工作台进行加锁
@param production_type 物品的类型
@parm  robot_id        机器人的编号
 */
void Workstation::add_locked_production(int production_type, int robot_id){
    if(this->locked.find(production_type)!=locked.end()) rel_locked_production(production_type);
    this->locked.insert({production_type, robot_id});
}

/*
工作台获得物品,此时从need这个集合将该物品类型删除
@param production_type 物品的类型
 */
void Workstation::putIn(int production_type){
    this->need.erase(production_type);
}

map<int, int> Workstation::getLocked(){
    return this->locked;
}

/*
是否有物品对工作台的原料台进行加锁
@return true    有物品对工作台的原料槽加锁
@return false   没有物品对工作台的原料台加锁
 */
bool Workstation::haveRawLocked(){
    int n =this->locked.count(production_item.type)>0? this->locked.size()-1 : this->locked.size();
    return n>0;

}

/*
换锁
@param production_type 物品的类型
@parm  robot_id        机器人的编号
 */
void Workstation::changeLocked(int production_type, int robot_id){
    this->locked[production_type] = robot_id;
}

/*
平台缺少原料的个数
 */
int Workstation::getMissingNum(){
    if(this->type <= 7)   return locked.count(production_item.type)>0?need.size()-locked.size()+1:need.size()-locked.size();
        // 表示8号始终就缺失一个 优先
    else if(this->type == 8) return -1;
        // 表示9号始终缺失三个 最次
    else if(this->type == 9) return -1;
    return -1;
}

//获取平台的剩余生产帧数
int Workstation::getLeftTime(){
    return remaingFrames;
}

//获得平台生产该物品的整个工作周期（帧数）
int Workstation::getCycleTime(){
    return this->production_item.production_time;
}

/*
获得本平台的权重
 根据平台的类型和缺少原料的个数给平台分配不同的权重
 */
int Workstation::getWeight(){
    if(type == 8) return 1;
    else if(type == 9) return MAX;
    if(type == 7){
        if(remaingFrames <= 500){
            int n = getMissingNum();
            if(n == 1) return 1;
            if(n == 2) return 2;
            else return 3;
        }else{
            return 64;
        }
    }else{
        return getMissingNum();
    }
}

/*
返回产品格状态
@return 1   有产品
@return 0   没有产品
 */
int Workstation::getProductStatus(){
    return this->productStatus;
}

//机器人收购平台生产物品的价格 ||购买价
int Workstation::getBuyPrice(){
    return this->production_item.buy_price;
}

//机器人卖出平台生产物品的价格    ||原始出售价
int Workstation::getSellPrice(){
    return this->production_item.sell_price;
}

// 获得工作台的类型
int Workstation::getType(){
    return this->type;
}
//获得平台的等级
int Workstation::getRank(){
    return this->production_item.priority;
}
//物品卖出的利润
double Workstation::getProfit(){
    return this->production_item.profit();
}

double Workstation::getX(){
    return this->coordinate.x;
}
double Workstation::getY(){
    return this->coordinate.y;
}