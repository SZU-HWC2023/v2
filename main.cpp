#include <iostream>
#include "manager.h"
// #include <unistd.h>

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

void handleFps(int frame_id){
    Map1 map1;
    map1.handleFps(frame_id);
}

int main(){
    // sleep(10);
    init_items();
    read_map();
    robotPassMap();
    findConnectedAreas();
    int map_type = check_map(g_workstations[0]);

    #ifdef DEBUG
    fprintf(stderr, "map_type: %d\n", map_type);
    #endif
    puts("OK");
    fflush(stdout);

    int frameID;
    int currentMoney = 0;
    while (scanf("%d %d", &frameID,&currentMoney) != EOF) {
        readUntilOK();
        printf("%d\n", frameID);
        fflush(stdout);
        //当前帧的处理逻辑
        handleFps(frameID);

        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;
}