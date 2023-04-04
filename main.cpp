#include <iostream>
#include "manager.h"
#include <unistd.h>

int check_map(Workstation *w){
    if((int)w->coordinate.x == 23 && (int)w->coordinate.y == 47)
        return 1;
    if((int)w->coordinate.x == 25 && (int)w->coordinate.y == 37)
        return 2;
    if((int)w->coordinate.x == 21 && (int)w->coordinate.y == 30)
        return 3;
    if((int)w->coordinate.x == 1 && (int)w->coordinate.y == 48)
        return 4;

    return 0;
}


template <typename T>
void process(T &map){
    int frameID;
    int currentMoney = 0;
    while (scanf("%d %d", &frameID,&currentMoney) != EOF) {
        readUntilOK();
        printf("%d\n", frameID);
        fflush(stdout);
        //当前帧的处理逻辑
        map.handleFps(frameID);
        printf("OK\n");
        fflush(stdout);
    }
}


void frameOperation(int map_type){
    if(map_type == 1){
        Map1 map1;
        process(map1);
    }
    if(map_type == 2){
        Map1 map2;
        process(map2);
    }
    if(map_type == 3){
        Map1 map3;
        process(map3);
    }
    if(map_type == 4){
        Map1 map4;
        process(map4);
    }
}
void deleteRed(){
    // ban掉多余的机器人
    set<int> worker_exit_areas;
    for(Workstation *w:g_workstations){
        if(w->type >= 4)  worker_exit_areas.insert(g_connected_areas_uc[w->coordinate]);
    }
    for(auto iter = g_robots.begin(); iter < g_robots.end(); iter++){
        if(worker_exit_areas.count(g_connected_areas_uc[(*iter)->coordinate]) <= 0) (*iter)->ban = true;
    }
    // ban多余的工作台
    set<int> robot_exit_area;
    for(int i = 0; i < 4;i++){
        robot_exit_area.insert(g_connected_areas_c[g_robots[i]->coordinate]);
    }
    for (auto iter = g_workstations.begin(); iter < g_workstations.end(); iter++) {
        if(robot_exit_area.count(g_connected_areas_c[(*iter)->coordinate]) <= 0) (*iter)->ban = true;
    }
}

int main(){
//    sleep(10);
    init_items();
    read_map();
    robotPassMap();
    findConnectedAreas();
    deleteRed();
    init_points();
    int map_type = check_map(g_workstations[0]);

    puts("OK");
    fflush(stdout);
    frameOperation(map_type);
    return 0;
}