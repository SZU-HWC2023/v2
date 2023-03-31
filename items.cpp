#include "class.h"

int g_ws_requirement[] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000110,
    0b00001010,
    0b00001100,
    0b01110000,
    0b10000000,
    0b11111110
};

int buy_prices[] = {0,3000,4400,5800,15400,17200,19200,76000};
int sell_prices[] = {0,6000,7600,9200,22500,25000,27500,105000};
int item_production_frame[] = {0,50,50,50,500,500,500,1000,1,1};
int item_priority[] = {-1,3,3,3,2,2,2,1};       // 需要改正 有点问题

map<int,Item> g_items;          // 物品编号[1-7]


void init_items(){
    for(int i = 1; i < ITEMS_NUM; i++){
        Item item;
        item.type = i;
        item.formula = g_ws_requirement[i];
        // fprintf(stderr, "formula: %s\n", item.formula.to_string().c_str());
        item.buy_price = buy_prices[i];
        item.sell_price = sell_prices[i];
        item.priority = item_priority[i];
        item.production_time = item_production_frame[i];
        g_items[i] = item;
    }
}
map<tuple<int,int>,Point*> g_point_map;
map<tuple<int,int,int,int>,vector<Point*>> g_astar_path; //存储平台之间的路径
AStar *g_astar;
DoubleDirectionAstar* g_directionAstar;
void init_points(){
    g_astar = new AStar();
    g_directionAstar = new DoubleDirectionAstar();
//    for(int i=0;i<MAP_TRUE_SIZE;i++){
//        for(int j=0;j<MAP_TRUE_SIZE;j++){
//            tuple<int,int> key = {i,j};
//            Point* point = new Point(i,j,0.0, nullptr);
//            g_point_map[key] = point;
//        }
//    }
//    for(int i=0;i<g_workstations.size();i++){
//        for(int j = i+1; j < g_workstations.size();j++){
//            Workstation* src_workstation = g_workstations[i];
//            Workstation* des_workstation = g_workstations[j];
//            vec2 src = GetPoint(src_workstation->coordinate.x,src_workstation->coordinate.y);
//            vec2 des = GetPoint(des_workstation->coordinate.x,des_workstation->coordinate.y);
//            vector<Point*> result = g_astar->planning(int(src.x),int(src.y),int(des.x),int(des.y),false);
//            g_astar_path[{int(src.x),int(src.y),int(des.x),int(des.y)}] = result;
//            vector<Point*> result_inv = result;
//            reverse(result_inv.begin(),result_inv.end());
//            g_astar_path[{int(des.x),int(des.y),int(src.x),int(src.y)}] = result_inv;
//        }
//    }
}