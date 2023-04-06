#include "class.h"




bitset<8> BAN_DIRECTION[][4] = {
    {0b00100000,0b01100000,0b11000000,0b10000000},
    {0b00110000,0b11111111,0b11111111,0b10000001},
    {0b00011000,0b11111111,0b11111111,0b00000011},
    {0b00001000,0b00001100,0b00000110,0b00000010}
};


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
            fprintf(stderr,"%ld ",b.to_ulong());
        }
        fprintf(stderr,"\n");
    }
}

bitset<8> DirectionMap::operator[](vec2_int pos_idx){
    return this->map[pos_idx.row][pos_idx.col];
}

bitset<8> DirectionMap::operator[](vec2 pos){

}

vec2 DirectionMap::to_pos(vec2_int pos_idx){
    return {0.5f * pos_idx.col, 0.5f * pos_idx.row};
}

vec2_int DirectionMap::to_DL_corner(vec2_int pos_idx){
    return pos_idx + vec2_int{-2,-2};
}