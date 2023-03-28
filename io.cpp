
#include "class.h"
vector<Workstation*> g_workstations;          //工作台列表
vector<Robot*> g_robots;

#include <cstring>
using namespace std;
/*
初始化：选手程序初始化时，将输入 100 行*100 列的字符组成的地图数据,然后紧接着一行 OK。
 * */
bool read_map(){
    //读取地图
    char line[1024];
    int current_workstation_id = 0;
    int current_robot_id = 0;
    int row = 0;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        line[strlen(line)-1] = '\0';
        for(int col=0;col < strlen(line);col++){
            if (line[col] == '.')continue;
            float y = 50 - ((row+1)*0.5-0.25);
            float x = (col+1)*0.5 - 0.25;
            if (line[col] == 'A'){
                //机器人
                Robot *robot = new Robot(current_robot_id,x,y);
                g_robots.emplace_back(robot);
                current_robot_id++;
            }
            if(line[col] >='1' && line[col] <='9') {
                //工作台
                Workstation *workstation = new Workstation(current_workstation_id, line[col] - '0', x, y);
                g_workstations.emplace_back(workstation);
                current_workstation_id++;
            }
        }
        row++;
    }
    return false;
}
/*
每一帧交互,更新工作台和机器人的信息
 * */
bool readUntilOK() {
    char line[1024];
    int K = 0;
    if(scanf("%d\n", &K) == EOF)return false;

    for(int i=0;i<K;i++){
        ws_frame wsFrame;
        scanf("%d %f %f %d %d %d\n",&wsFrame.ws_type,&wsFrame.x,&wsFrame.y,&wsFrame.frameID,&wsFrame.rawStatusCode,&wsFrame.productStatus);
        g_workstations[i]->update(wsFrame);
    }
    for (int i=0;i<4;i++){
        robot_frame robotFrame;
        scanf("%d %d %f %f %f %f %f %f %f %f\n",&robotFrame.workshopLocated,&robotFrame.itemCarried,
              &robotFrame.timeValue,&robotFrame.collisionValue,
              &robotFrame.angSpd,&robotFrame.linSpdx,&robotFrame.linSpdy,
              &robotFrame.heading,&robotFrame.x,&robotFrame.y);
        g_robots[i]->update(robotFrame);
    }

    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        line[strlen(line)-1] = '\0';
    }
    return false;
}