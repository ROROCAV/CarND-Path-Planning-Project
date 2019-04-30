//
// Created by luyifan on 19-4-30.
//
#include "state/StRun.h"
#include "state/StStop.h"
#include "state/StDrive.h"

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
    start[0] = planner.ego->poUTM()[0];
    start[1] = planner.ego->poUTM()[1];
    cout<<"Spped up!!!!"<<endl;
}

StStart::~StStart(){}

sc::result StStart::react(const EvSysTick &evt){
    Planner& planner = context<Planner>();
    Ego* ego = planner.ego;
    Trajectory pre_path = *planner.pre_path;
    vector<Vehicle>* predictions = planner.predictions;

    ///起步10米之后跳转到行驶状态
    if(distance(start[0], start[1], ego->poUTM()[0], ego->poUTM()[1]) > 10)
        return transit<StDrive>();

    planner.generator->laneKeeping(pre_path, ego, predictions, planner.next_path);
    return forward_event();
}

sc::result StStart::react(const sc::exception_thrown &evt){
    return transit<StStop>();
}