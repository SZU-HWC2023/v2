#include <iostream>
#include "class.h"

int main(){

    read_map();
    puts("OK");
    fflush(stdout);

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