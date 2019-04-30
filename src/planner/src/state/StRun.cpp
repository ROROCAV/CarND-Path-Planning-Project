//
// Created by luyifan on 19-4-30.
//
#include "state/StRun.h"
#include "state/StStop.h"

/*******************************
* StRun
* *****************************/
StRun::StRun(my_context ctx) : my_base(ctx), StBase<StRun> (std::string("StRun")) {}

StRun::~StRun(){}

sc::result StRun::react(const EvSysTick &evt){
    return forward_event();
}

sc::result StRun::react(const sc::exception_thrown &evt){
    return transit<StStop>();
}

/*******************************
* StStart
* *****************************/
StStart::StStart(my_context ctx) : my_base(ctx), StBase<StStart> (std::string("StStart")){
    Planner& planner = context<Planner>();
    //start.x = self_state->GetCurrentPose().pose.position.x;
    //start.y = self_state->GetCurrentPose().pose.position.y;
}

StStart::~StStart(){}

sc::result StStart::react(const EvSysTick &evt){
    Planner& planner = context<Planner>();

    ///起步10米之后跳转到行驶状态
    //if(distance(start[0], start[1], cur_x, cur_y))
    //    return transit<StDrive>();

    return forward_event();
}

sc::result StStart::react(const sc::exception_thrown &evt){
    return transit<StStop>();
}