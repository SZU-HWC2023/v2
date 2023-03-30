#include "class.h"


DWA::DWA(Robot *robot_, vector<Robot *> robots){
    // fprintf(stderr,"Other_robots %d\n", this->other_robots.size());
    // fprintf(stderr,"Other_robots %d\n", this->other_robots.size());
    this->robot = robot_;
    // fprintf(stderr,"Other_robots %d\n", this->other_robots.size());
}

float DWA::tgt_cost(vector<DWA_state> trajectory, vec2 tgt_pos){
    float min_dist = 2;
    for(auto p:trajectory){
        float dist = calcDistance(p.pos, tgt_pos);
        if(dist < 0.03)
            return -1;
        if(dist < min_dist)
            min_dist = dist;
    }
    float tgt_hdg = calcHeading(trajectory.back().pos, tgt_pos);
    float deltaHDG = abs(clampHDG(tgt_hdg - trajectory.back().heading));
    return abs(atan2(sinf(deltaHDG), cosf(deltaHDG))) + min_dist;
}

float wall_dist(vec2 pos){
    return min(
        min(pos.x, 50-pos.x),
        min(pos.y, 50-pos.y)
    );
}


float DWA::obs_cost(vector<DWA_state> trajectory){
    float min_robot_dist = 10, min_wall_dist = 10;
    for(int i=0; i<trajectory.size(); i++){
        min_wall_dist = min(min_wall_dist, wall_dist(trajectory[i].pos));
        for(auto r:this->robot->other_robots){
            float dist = calcDistance(trajectory[i].pos, r->trajectory[i].pos);
            if(dist < min_robot_dist)
                min_robot_dist = dist;
        }
    }
    if(min_robot_dist<ROBOT_CARRY_RADIUS*2)
        return 9999999;
    if(min_wall_dist<this->robot->crt_radius-0.01)
        return 9999999;
    else if(min_wall_dist>1)
        min_wall_dist = 1;
        
    return 1.0/min_robot_dist + 1.0/min_wall_dist * 0.5;
}

float DWA::vel_cost(vector<DWA_state> trajectory, vec2 tgt_pos){
    float v_desire = MAX_FORWARD_SPD;
    float vel = trajectory.back().linSpd.len();
    float tgt_hdg = calcHeading(trajectory.back().pos, tgt_pos);
    float deltaHDG = abs(clampHDG(tgt_hdg - trajectory.back().heading));
    float s_v = pow(MAX_FORWARD_SPD, 2)/(2*this->robot->crt_lin_acc);
    float s0 = 0.8+pow(M_PI-abs(deltaHDG), 3)/pow(M_PI, 3)*MAX_FORWARD_SPD;
    float dist = calcDistance(trajectory.back().pos, tgt_pos);
    if(dist > s_v) s0 = MAX_FORWARD_SPD;
    
    return abs(MAX_FORWARD_SPD - vel);
}

float DWA::calc_cost(VW vw, vec2 tgt_pos, bool log){
    DWA_state state(this->robot);
    vector<DWA_state> trajectory = state.calcTrajectory(vw, pred_t);
    float tgt_cost = this->tgt_cost(trajectory, tgt_pos);
    float obs_cost = this->obs_cost(trajectory);
    float vel_cost = this->vel_cost(trajectory, tgt_pos);
    float cost = this->alpha*tgt_cost + this->beta*obs_cost + this->gamma*vel_cost;
    #ifdef DEBUG
    if(log){
        fprintf(stderr, "tgt_cost: %f, obs_cost: %f, vel_cost: %f, cost: %f\n", tgt_cost, obs_cost, vel_cost, cost);
    }
    #endif
    return cost;
}

VW DWA::find_vw(vec2 tgt_pos){
    DWA_state crt_state(this->robot);
    DW dw = crt_state.calcDW();
    float lin_spd_intv = dw.v_step(this->v_samples), ang_spd_intv = dw.ang_v_step(this->ang_v_samples);
    float min_cost = INFINITY;
    VW vw;
    for(float angle = dw.ang_v_min; angle < dw.ang_v_max; angle += ang_spd_intv){
        for(float spd = dw.v_min; spd < dw.v_max; spd += lin_spd_intv){
            if(abs(spd)<(0.1))
                continue;
            VW vw_tmp = {spd, angle};
            float cost = calc_cost(vw_tmp, tgt_pos);
            if(cost < min_cost){
                min_cost = cost;
                vw = vw_tmp;
            }
        }
    }
    return vw;
}