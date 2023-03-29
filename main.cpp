#include <iostream>
#include "manager.h"

int main(){

    read_map();
    puts("OK");
    fflush(stdout);

    int frameID;
    int currentMoney = 0;
    for(int i=0;i<MAP_TRUE_SIZE+1;i++){
        for(int j=0;j<MAP_TRUE_SIZE+1;j++){
            fprintf(stderr,"%c",g_map[i][j]);
        }
        fprintf(stderr,"\n");
    }
    while (scanf("%d %d", &frameID,&currentMoney) != EOF) {
        readUntilOK();
        printf("%d\n", frameID);

        //当前帧的处理逻辑

        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;

}