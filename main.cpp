#include <iostream>
#include "manager.h"
#include <unistd.h>

int check_map(Workstation *w){
    if(w->type == 2)
        return 1;
    if((int)w->coordinate.x == 25 && (int)w->coordinate.y == 37)
        return 2;
    if((int)w->coordinate.x == 21 && (int)w->coordinate.y == 30)
        return 3;
    if((int)w->coordinate.x == 1 && (int)w->coordinate.y == 48)
        return 4;

    return 2;
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
        Map2 map2;
        process(map2);
    }
    if(map_type == 3){
        Map3 map3;
        process(map3);
    }
    if(map_type == 4){
        Map4 map4;
        process(map4);
    }
}
// 多余去除
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
        // 只有大于等于4的工作台才会被置为'$'
        if(g_map[(*iter)->coordinate] == '$') {
            (*iter)->ban = true;
        }
    }
}
bool judgeWOrR(int row, int col){
    if(g_map[row][col]=='A'||(g_map[row][col]>='0'&&g_map[row][col]<='9') || g_map[row][col]=='@') return true;
    return false;
}
bool judgeObs(int row, int col){
    if(row<0||col<0||row>=MAP_TRUE_SIZE||col>=MAP_TRUE_SIZE) return false;
    return true;
}
// 地图膨胀
void expandMap(){
    for(int row = MAP_TRUE_SIZE-1; row >= 0; row--){
        for(int col = 0; col < MAP_TRUE_SIZE-1; col++){
            if(g_map[row][col] == '#'){
                // 左上
                if(!(judgeWOrR(row+1, col)||judgeWOrR(row+1, col-1)||judgeWOrR(row, col-1))){
                    if(judgeObs(row+1, col)) g_map.map[row+1][col] = '#';
                    if(judgeObs(row, col-1)) g_map.map[row][col-1] = '#';
                    if(judgeObs(row+1, col-1)) g_map.map[row+1][col-1] = '#';
                }
            }
        }
    }
}

int main(){
//    sleep(10);
    init_items();
    read_map();
    robotPassMap();
    findConnectedAreas();
    deleteRed();
    expandMap();

    init_points();
    AStarTest* aStarTest = new AStarTest();
//    int begin_workstation = 14;
//    vec2_int begin_vec = g_direction_map.to_pos_idx(g_workstations[begin_workstation]->coordinate);
//    int end_workstation = 18;
//    vec2_int end_vec = g_direction_map.to_pos_idx(g_workstations[end_workstation]->coordinate);
//    fprintf(stderr,"起始工作台: %d 类型：%d \n",g_workstations[begin_workstation]->id,g_workstations[begin_workstation]->type);
//    fprintf(stderr,"终止工作台: %d 类型：%d \n",g_workstations[end_workstation]->id,g_workstations[end_workstation]->type);
//    vector<Point*> result = aStarTest->planning(begin_vec,end_vec,true);
//    fprintf(stderr,"==================================\n");
//    fprintf(stderr,"result.size:%d\n",result.size());
////    print_path(result);
//    fprintf(stderr,"==================================\n");
    int map_type = check_map(g_workstations[0]);

    puts("OK");
    fflush(stdout);
    frameOperation(map_type);
    return 0;
}