#include <iostream>
#include "manager.h"

void handleFps(int frame_id){
    Map1 map1;
    map1.handleFps(frame_id);
}

int main(){
    // sleep(10);
    init_items();
    read_map();
    // robotPassMap();
    // findConnectedAreas();
    // test_astar();  //测试A*算法

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