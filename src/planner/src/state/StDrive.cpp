//
// Created by luyifan on 19-4-30.
//

#include "state/StDrive.h"
#include "state/StStop.h"

/*---------------------------------------------------------------------------
 * StDrive
 *---------------------------------------------------------------------------*/
StDrive::StDrive(my_context ctx) : my_base(ctx), StBase<StDrive>(std::string("StDrive")) {
}

StDrive::~StDrive(){}

sc::result StDrive::react(const EvStop& evt){
    return transit<StStop>();
}

sc::result StDrive::react(const EvSysTick& evt) {

    return forward_event();
}

/*---------------------------------------------------------------------------
 * StDriveStart
 *---------------------------------------------------------------------------*/
StDriveStart::StDriveStart(my_context ctx) : my_base(ctx), StBase<StDriveStart>(std::string("StDriveStart")) {}

StDriveStart::~StDriveStart() {
}

sc::result StDriveStart::react(const EvSysTick& evt){
    return transit<StLaneKeep>();
}

/*---------------------------------------------------------------------------
 * StLaneKeep
 *---------------------------------------------------------------------------*/
StLaneKeep::StLaneKeep(my_context ctx) : my_base(ctx), StBase<StLaneKeep>(std::string("StLaneKeep")) {

}

StLaneKeep::~StLaneKeep() {}

sc::result StLaneKeep::react(const EvSysTick&) {
    Planner& planner = context<Planner>();
    Ego* ego = planner.ego;
    Trajectory pre_path = *planner.pre_path;
    vector<Vehicle>* predictions = planner.predictions;

    planner.generator->laneKeeping(pre_path, ego, predictions, planner.next_path);
    return forward_event();
}