#include "class.h"
#include <stack>

const int N = MAP_TRUE_SIZE;
const int dr[4] = {1, 0, -1, 0};
const int dc[4] = {0, -1, 0, 1};
Map<int> g_connected_areas_c;    // 携带物品全局连通区域
Map<int> g_connected_areas_uc;   // 未携带物品全局连通区域


bool judgeWall(int row, int col){
    return row < 0 || col < 0 || row >= MAP_TRUE_SIZE || col >= MAP_TRUE_SIZE || g_map[row][col] == '#';
}
// 
bool hinderWithPro(int row, int col){
    bool up1=judgeWall(row-1,col),down1=judgeWall(row+1,col),left1=judgeWall(row,col-1),right1=judgeWall(row,col+1);
    bool up2=judgeWall(row-2,col),down2=judgeWall(row+2,col),left2=judgeWall(row,col-2),right2=judgeWall(row,col+2);
    bool u1l1=judgeWall(row-1,col-1),u1r1=judgeWall(row-1,col+1),d1l1=judgeWall(row+1,col-1),d1r1=judgeWall(row+1,col+1);
    bool u2l2=judgeWall(row-2,col-2),u2l1=judgeWall(row-2,col-1),u2r1=judgeWall(row-2,col+1),u2r2=judgeWall(row-2,col+2);
    bool u1l2=judgeWall(row-1,col-2),u1r2=judgeWall(row-1,col+2),d1l2=judgeWall(row+1,col-2),d1r2=judgeWall(row+1,col+2);
    bool d2l2=judgeWall(row+2,col-2),d2l1=judgeWall(row+2,col-1),d2r1=judgeWall(row+2,col+1),d2r2=judgeWall(row+2,col+2);
    if(u2l1&&down1||up2&&down1||up2&&d1l1||up2&&d1r1||u2r1&&down1) return true;
    if(u1l2&&right1||u1l1&&down2||u1l1&&right2||up1&&d2l1||up1&&down2||up1&&d2r1||u1r1&&down2||u1r1&&left2||u1r2&&left1) return true;
    if(left2&&right1||left2&&u1r1||left2&&d1r1||left1&&u1r2||left1&&d1r2||left1&&right2||right1&&d1l2||right2&&d1l1) return true;
    return false;
}
bool hinderWithoutPro(int row, int col){
    bool up=judgeWall(row-1,col), down=judgeWall(row+1,col),left=judgeWall(row,col-1),right=judgeWall(row,col+1);
    bool u1l1=judgeWall(row-1,col-1),u1r1=judgeWall(row-1,col+1),d1l1=judgeWall(row+1,col-1),d1r1=judgeWall(row+1,col+1);
    if(up&&down||left&&right||u1l1&&d1r1||u1r1&&d1l1) return true;
    if(up&&d1l1||up&&d1r1||down&&u1l1||down&&u1r1) return true;
    if(left&&u1r1||left&&d1r1||right&&u1l1||right&&d1l1) return true;
    else return false;
}
bool hinderByCorner(int row, int col){
    bool up=judgeWall(row-1,col), down=judgeWall(row+1,col),left=judgeWall(row,col-1),right=judgeWall(row,col+1);
    if(up&&left||up&&right||left&&down||down&&right) return true;
    return false;
}
// 读地图的时候 判断一些不可穿过的点，分为机器人不带物品(!)和机器人带物品(@)
// 不带物品的机器人都过不去，带物品的机器人更过不去
void robotPassMap(){
    for(int r=MAP_TRUE_SIZE-1; r>=0; r--){
        for(int c=0; c<MAP_TRUE_SIZE; c++){
            if(g_map.map[r][c] == '.'){
                if(hinderByCorner(r, c)) g_map.map[r][c] = '$';
                else if(hinderWithoutPro(r, c))  g_map.map[r][c] = '!';
                else if(hinderWithPro(r, c))  g_map.map[r][c] = '@';
            }
            fprintf(stderr,"%c", g_map[r][c]);
        }
        fprintf(stderr,"\n");
    }
}
// 是不是障碍物，是障碍物返回true  不带物品
bool judgeObstacle_uc(int row, int col){
    vec2_int coor = {row, col};
    if(g_map[coor] == '#' || g_map[coor] == '!') return true;
    else return false;
}
// 是不是障碍物，是障碍物返回true  带物品
bool judgeObstacle_c(int row, int col){
    vec2_int coor = {row, col};
    if(g_map[coor] == '#' || g_map[coor] == '!' || g_map[coor] == '@') return true;
    else return false;
}

// 深度优先遍历
void DFS_c(int row, int col, int area_cnt_c) {
    stack<pair<int,int>> st;
    st.push({row, col});
    while (!st.empty()) {
        auto cur_node = st.top();
        st.pop();
        for (int i = 0; i < 4; i++) {
            int nrow = cur_node.first + dr[i];
            int ncol = cur_node.second + dc[i];
            if (nrow >= 0 && nrow < N && ncol >= 0 && ncol < N){
                if(!judgeObstacle_c(nrow, ncol) && g_connected_areas_c[nrow][ncol] == 0){
                    g_connected_areas_c.map[nrow][ncol] = area_cnt_c;
                    st.push({nrow, ncol});
                }
            }
        }
    }
}

// 深度优先遍历
void DFS_uc(int row, int col, int area_cnt_uc) {
    stack<pair<int,int>> st;
    st.push({row, col});
    while (!st.empty()) {
        auto cur_node = st.top();
        st.pop();
        for (int i = 0; i < 4; i++) {
            int nrow = cur_node.first + dr[i];
            int ncol = cur_node.second + dc[i];
            if (nrow >= 0 && nrow < N && ncol >= 0 && ncol < N){
                if(!judgeObstacle_uc(nrow, ncol) && g_connected_areas_uc[nrow][ncol] == 0){
                    g_connected_areas_uc.map[nrow][ncol] = area_cnt_uc;
                    st.push({nrow, ncol});
                }
            }
        }
    }
}
void init_g_connect_map(){
    for(int r = 0; r < N; r++){
        for(int c = 0; c < N; c++){
            g_connected_areas_c.map[r][c] = 0;
            g_connected_areas_uc.map[r][c] = 0;
        }
    }
}
// 查找所有连通区域
void findConnectedAreas(){
    init_g_connect_map();
    int area_cnt_uc = 0, area_cnt_c = 0;   // 代表连通域序列号 0代表障碍物
    for (int i = N-1; i >= 0; i--) {
        for (int j = 0; j < N; j++) {
            if (!judgeObstacle_c(i, j) && g_connected_areas_c[i][j] == 0){
                area_cnt_c++;
                g_connected_areas_c.map[i][j] = area_cnt_c;
                DFS_c(i, j, area_cnt_c);
            }
            if (!judgeObstacle_uc(i, j) && g_connected_areas_uc[i][j] == 0){
                area_cnt_uc++;
                g_connected_areas_uc.map[i][j] = area_cnt_uc;
                DFS_uc(i, j, area_cnt_uc);
            }
            fprintf(stderr,"%d",g_connected_areas_c[i][j]);
        }
        fprintf(stderr,"\n");
    }
}
