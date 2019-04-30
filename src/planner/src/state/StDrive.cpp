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
    return transit<StDriveOnLane>();
}

/*---------------------------------------------------------------------------
 * StDriveOnLane
 *---------------------------------------------------------------------------*/
StDriveOnLane::StDriveOnLane(my_context ctx) : my_base(ctx), StBase<StDriveOnLane>(std::string("StDriveOnLane")) {

}

StDriveOnLane::~StDriveOnLane() {}

sc::result StDriveOnLane::react(const EvSysTick&) {
    Planner& planner = context<Planner>();
    return forward_event();
}