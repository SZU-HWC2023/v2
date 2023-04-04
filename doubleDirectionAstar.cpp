// #include "class.h"

// vector<Point*> DoubleDirectionAstar::planning(int sx,int sy,int gx,int gy){
//     Point* start_node = new Point(sx,sy,0.0, nullptr);
//     Point* goal_node = new Point(gx,gy,0.0, nullptr);
//     map<tuple<int,int>,Point*> open_map_A;    // 存储待检测节点
//     map<tuple<int,int>,Point*> open_map_B;    // 存储待检测节点
//     map<tuple<int,int>,Point*> closed_map_A;  // 存储已经检测过的节点
//     map<tuple<int,int>,Point*> closed_map_B;  // 存储已经检测过的节点
//     open_map_A[getIndex(start_node)] = start_node;
//     open_map_B[getIndex(goal_node)] = goal_node;

//     Point* current_A = start_node;
//     Point* current_B = goal_node;
//     Point* meet_point_A = nullptr;
//     Point* meet_point_B = nullptr;

//     while(true){
//         if(open_map_A.size()==0)break;
//         if(open_map_B.size()==0)break;

//         float cost_A = MAX;
//         tuple<int,int> c_id_A;
//         // 从待检测节点中，找到一个到目标节点代价最小的节点
//         for(auto iter = open_map_A.begin();iter!=open_map_A.end();iter++){
//             Point* p = iter->second;
//             float tmp_cost = this->calc_total_cost(open_map_A,p,current_B);
//             if(tmp_cost < cost_A){
//                 cost_A = tmp_cost;
//                 c_id_A = iter->first;
//             }
//         }
//         current_A = open_map_A[c_id_A];
//         float cost_B = MAX;
//         tuple<int,int> c_id_B;
//         // 从待检测节点中，找到一个到目标节点代价最小的节点
//         for(auto iter = open_map_B.begin();iter!=open_map_B.end();iter++){
//             Point* p = iter->second;
//             float tmp_cost = this->calc_total_cost(open_map_B,p,current_A);
//             if(tmp_cost < cost_B){
//                 cost_B = tmp_cost;
//                 c_id_B = iter->first;
//             }
//         }
//         current_B = open_map_B[c_id_B];

//         // 已经找到目标节点，退出循环
//         if(current_A->coordinate.x == current_B->coordinate.x && current_A->coordinate.y == current_B->coordinate.y){
//             meet_point_A = current_A;
//             meet_point_B = current_B;
//             break;
//         }
//         open_map_A.erase(c_id_A);   //将已经访问过的节点从开放列表移除
//         open_map_B.erase(c_id_B);
//         closed_map_A[c_id_A] = current_A;
//         closed_map_B[c_id_B] = current_B;
//         //从当前节点往各个方向探索
//         for(tuple<int,int, float> mot:this->motion){
//             Point* point_A = new Point(current_A->coordinate.x + get<0>(mot),current_A->coordinate.y + get<1>(mot),current_A->cost+get<2>(mot),current_A);
//             Point* point_B = new Point(current_B->coordinate.x + get<0>(mot),current_B->coordinate.y + get<1>(mot),current_B->cost+get<2>(mot),current_B);
//             tuple<int,int> p_id_A = getIndex(point_A);
//             tuple<int,int> p_id_B = getIndex(point_B);

//             if(verify(current_A,point_A)&& closed_map_A.find(p_id_A)==closed_map_A.end()){
//                 if(open_map_A.find(p_id_A)==open_map_A.end()){
//                     open_map_A[p_id_A] = point_A;
//                 }else{
//                     if(open_map_A[p_id_A]->cost > point_A->cost){
//                         open_map_A[p_id_A] = point_A;
//                     }
//                 }
//             }
//             if(verify(current_B,point_B)&& closed_map_B.find(p_id_B)==closed_map_B.end()){
//                 if(open_map_B.find(p_id_B) == open_map_B.end()){
//                     open_map_B[p_id_B] = point_B;
//                 }else{
//                     if(open_map_B[p_id_B]->cost > point_B->cost){
//                         open_map_B[p_id_B] = point_B;
//                     }
//                 }
//             }

//         }
//     }
//     vector<Point*> result = this->calc_final_doubledirectional_path(meet_point_A,meet_point_B,closed_map_A,closed_map_B);
//     return result;
// }
// vector<Point*> DoubleDirectionAstar::calc_final_path(Point* goal_node,map<tuple<int,int>,Point*> &closed_map){
//     if(goal_node->parent_node== nullptr)return {};
//     vector<Point*> result;
//     result.emplace_back(goal_node);
//     Point* parent = goal_node->parent_node;
//     while(parent!=nullptr){
//         Point* p = closed_map[getIndex(parent)];
//         result.emplace_back(p);
//         parent = p->parent_node;
//     }
//     reverse(result.begin(),result.end());
//     return this-> simplify_path(result);
// }
// vector<Point*> DoubleDirectionAstar::calc_final_doubledirectional_path(Point* meetA, Point* meetB,map<tuple<int,int>,Point*> &cloaes_map_A,map<tuple<int,int>,Point*> &cloaes_map_B){
//     if(meetA== nullptr&&meetB== nullptr)return {};
//     if(meetA == nullptr){
//         vector<Point*> B = this->calc_final_path(meetB,cloaes_map_B);
//         return B;
//     }
//     if(meetB == nullptr){
//         vector<Point*> A = this->calc_final_path(meetA,cloaes_map_A);
//         return A;
//     }
//     vector<Point*> A = this->calc_final_path(meetA,cloaes_map_A);
//     vector<Point*> B = this->calc_final_path(meetB,cloaes_map_B);
//     if(A.size()!=0&&B.size()!=0){
//         A.insert(A.end(),B.begin()+1,B.end());
//         return A;
//     }
//     if(A.size()==0)return B;
//     return A;
// }
// //判断下标是否合法
// bool DoubleDirectionAstar::verify(Point* from,Point* p){
//     if(p->coordinate.x<0 || p->coordinate.y<0 ||p->coordinate.x>=MAP_TRUE_SIZE||p->coordinate.y>=MAP_TRUE_SIZE)return false;
//     if(g_map[p->coordinate.x][p->coordinate.y] == '#')return false;
//     int from_x = from->coordinate.x;
//     int from_y = from->coordinate.y;
//     int p_x = p->coordinate.x;
//     int p_y = p->coordinate.y;


// vector<Point *> DoubleDirectionAstar::simplify_path(vector<Point*> &vec_points){
//     if(vec_points.size()<=2)return vec_points;
//     vector<Point*> result{vec_points[0],vec_points[1]};
//     for(int i=2;i<vec_points.size();i++){
//         Point* pp = result[result.size()-2];
//         Point* p = result[result.size()-1];
//         Point * current = vec_points[i];
//         int p_pp_x = p->coordinate.x - pp->coordinate.x;
//         int p_pp_y = p->coordinate.y - pp->coordinate.y;
//         float k1 = atan2(p_pp_y,p_pp_x);
//         int current_p_x = current->coordinate.x - p->coordinate.x;
//         int current_p_y = current->coordinate.y - p->coordinate.y;
//         float k2 = atan2(current_p_y,current_p_x);
//         if(abs(k1-k2) < 1e-6){
//             result[result.size()-1] = current;
//         }else{
//             result.emplace_back(current);
//         }
//     }
//     return result;

// }