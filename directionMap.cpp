#include "class.h"



bitset<8> BAN_DIRECTION[][4] = {
    {0b00100000,0b01100000,0b11000000,0b10000000},
    {0b00110000,0b11111111,0b11111111,0b10000001},
    {0b00011000,0b11111111,0b11111111,0b00000011},
    {0b00001000,0b00001100,0b00000110,0b00000010}
};


// bitset<8> BAN_DIRECTION[][4] = {
//     {0b00100000,0b11100000,0b11100000,0b10000000},
//     {0b00111000,0b11111111,0b11111111,0b10000011},
//     {0b00111000,0b11111111,0b11111111,0b10000011},
//     {0b00001000,0b00001110,0b00001110,0b00000010}
// };


DirectionMap::DirectionMap(){
}

void DirectionMap::init(RawMap &raw_map){
    for(int row=100; row >=0;row--){
        for(int col=0; col <MAP_TRUE_SIZE+1;col++){
            bitset<8> b = 0b111111111;
            vec2_int dl = this->to_DL_corner({row,col});
            for(int i=0;i<4;i++){
                for(int j=0;j<4;j++){
                    //xor
                    if(raw_map.find(dl + vec2_int{i,j}) == '#'){
                        bitset<8> ban = b & BAN_DIRECTION[i][j];
                        b = b ^ ban;
                    }
                }
                if(b==0)    break;
            }
            this->map[row][col] = b;
        //    fprintf(stderr,"%ld ",b.to_ulong());
        }
    //    fprintf(stderr,"\n");
    }
}

bitset<8> DirectionMap::operator[](vec2_int pos_idx){
    return this->map[pos_idx.row][pos_idx.col];
}

bitset<8> DirectionMap::operator[](vec2 pos){
    return this->operator[](this->to_pos_idx(pos));
}

//方向图索引转坐标
vec2 DirectionMap::to_pos(vec2_int pos_idx,bool compenstate){
    vec2 pos = {0.5f * pos_idx.col, 0.5f * pos_idx.row};
    if(!compenstate) return pos;
    bitset<8> b = this->operator[](pos_idx);
    b.flip();
    b &= 0b01010101;
    for(int i=0;i<8;i+=2){
        if(b[i]){
            pos += this->compenstation[i];
        }
    }
    return pos;
}

//坐标转方向图索引
vec2_int DirectionMap::to_pos_idx(vec2 pos){
    return {(int)roundf(pos.y * 2), (int)roundf(pos.x * 2)};
}

vec2_int DirectionMap::find_passable_vertice(vec2 pos){
    vec2_int pos_idx = {(int) pos.y*2, (int) pos.x*2};
    for(int i=0; i<4;i++){
        vec2_int d = {i>>0&1, i>>1&1};
        if(this->operator[](pos_idx + d).count()){
            return pos_idx + d;
        }
    }
}


vec2_int DirectionMap::to_DL_corner(vec2_int pos_idx){
    return pos_idx + vec2_int{-2,-2};
}
int DirectionMap::direction_num(vec2_int v) { //获得可达方向的数量
    bitset<8> b = this->operator[](v);
    return b.count();

}

//是否携带物品时可通过
bool DirectionMap::is_carry_passable(vec2_int pos_idx){
    bitset<8> b = this->operator[](pos_idx);
    bool res = (b[0] || b[4]) && (b[2] || b[6]);
    if(!res){
        vec2 pos = this->to_pos(pos_idx);
        // fprintf(stderr,"%.2f, %.2f|", pos.x,pos.y);
    }
    return res;
}
<<<<<<< Updated upstream
=======
// //连通方向
// int DirectionMap::direction_num(vec2_int v){
//     bitset<8> b = this->operator[](v);
//     return b.count();
// }
>>>>>>> Stashed changes

//是否携带物品时可通过
bool DirectionMap::is_carry_passable(vec2 pos){
    return this->is_carry_passable(this->to_pos_idx(pos));
}

//返回pos_idx所在点的可通行方向矢量
vector<vec2_int> DirectionMap::get_directions(vec2_int pos_idx){
    vector<vec2_int> d;
    bitset<8> b = this->operator[](pos_idx);
    for(int i=0;i<8;i++){
        if(b[i]){
            d.push_back(directions[i]);
        }
    }
    return d;
}