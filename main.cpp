#include <iostream>
#include "manager.h"
// #include <unistd.h>

int main(){
    // sleep(10);
    init_items();
    read_map();
    
    puts("OK");
    fflush(stdout);
    test_astar();  //测试A*算法
    int frameID;
    int currentMoney = 0;

    while (scanf("%d %d", &frameID,&currentMoney) != EOF) {
        readUntilOK();
        printf("%d\n", frameID);

        //当前帧的处理逻辑

        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;

}