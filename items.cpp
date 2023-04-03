#include "class.h"

int g_ws_requirement[] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000110,
    0b00001010,
    0b00001100,
    0b01110000,
    0b10000000,
    0b11111110
};

int buy_prices[] = {0,3000,4400,5800,15400,17200,19200,76000};
int sell_prices[] = {0,6000,7600,9200,22500,25000,27500,105000};
int item_production_frame[] = {0,50,50,50,500,500,500,1000,1,1};
int item_priority[] = {-1,3,3,3,6,6,6,6};       // 需要改正 有点问题

map<int,Item> g_items;          // 物品编号[1-7]


void init_items(){
    for(int i = 1; i < ITEMS_NUM; i++){
        Item item;
        item.type = i;
        item.formula = g_ws_requirement[i];
        // fprintf(stderr, "formula: %s\n", item.formula.to_string().c_str());
        item.buy_price = buy_prices[i];
        item.sell_price = sell_prices[i];
        item.priority = item_priority[i];
        item.production_time = item_production_frame[i];
        g_items[i] = item;
    }
}
