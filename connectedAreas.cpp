#include "class.h"
#include <stack>

const int N = MAP_TRUE_SIZE;
const int dx[4] = {-1, 0, 1, 0};
const int dy[4] = {0, 1, 0, -1};
int g_connected_areas[MAP_TRUE_SIZE][MAP_TRUE_SIZE];    // 全局连通区域 


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
    for(int r=0; r<MAP_TRUE_SIZE; r++){
        for(int c=0; c<MAP_TRUE_SIZE; c++){
            if(g_map[r][c] == '.'){
                if(hinderByCorner(r, c)) g_map[r][c] = '$';
                else if(hinderWithoutPro(r, c)) g_map[r][c] = '!';
                else if(hinderWithPro(r, c)) g_map[r][c] = '@';
            }
            // fprintf(stderr,"%c", g_map[r][c]);
        }
        // fprintf(stderr,"\n");
    }
}



// 是不是障碍物，是障碍物返回true
bool judgeObstacle(int x, int y){
    if(g_map[x][y] == '#' || g_map[x][y] == '!' || g_map[x][y] == '@') return true;
    else return false;
}

// 深度优先遍历
void DFS(int x, int y, int area_cnt) {
    stack<pair<int,int>> st;
    st.push({x, y});
    while (!st.empty()) {
        auto cur_node = st.top();
        st.pop();
        for (int i = 0; i < 4; i++) {
            int nx = cur_node.first + dx[i];
            int ny = cur_node.second + dy[i];
            if (nx >= 0 && nx < N && ny >= 0 && ny < N && !judgeObstacle(nx, ny) && g_connected_areas[nx][ny] == 0) {
                g_connected_areas[nx][ny] = area_cnt;
                st.push({nx, ny});
            }
        }
    }
}

// 查找所有连通区域
void findConnectedAreas(){
    int area_cnt = 0;   // 0代表障碍物
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (!judgeObstacle(i, j) && g_connected_areas[i][j] == 0){
                area_cnt++;
                g_connected_areas[i][j] = area_cnt;
                DFS(i, j, area_cnt);
            }
            // fprintf(stderr,"%d",j);
        }
        // fprintf(stderr,"\n");
    }
}
