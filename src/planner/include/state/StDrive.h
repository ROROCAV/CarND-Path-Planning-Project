//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_STDRIVE_H
#define PATH_PLANNING_STDRIVE_H

#include "planner.h"
#include "StBase.h"
#include "state/StRun.h"
/*---------------------------------------------------------------------------
 * StDrive
 *---------------------------------------------------------------------------*/
class StDriveStart;
class StDrive : public sc::state<StDrive, StRun, StDriveStart>, public StBase<StDrive> {
public:
    // on enter
    StDrive(my_context ctx);

    // on exit
    virtual ~StDrive();

    // reactions
    sc::result react(const EvSysTick& evt);
    sc::result react(const EvStop& evt);

    typedef mpl::list
    <
    sc::custom_reaction< EvSysTick >
    > reactions;
};

/*---------------------------------------------------------------------------
 * StDriveStart
 *---------------------------------------------------------------------------*/
class StDriveStart : public sc::state< StDriveStart, StDrive >, StBase<StDriveStart> {
public:
    //on enter
    StDriveStart(my_context ctx);
    //on exit
    virtual ~StDriveStart();

    //reactions
    sc::result react(const EvSysTick& evt);

    //reactions
    typedef mpl::list
    <
    sc::custom_reaction< EvSysTick >
    > reactions;
};

/*---------------------------------------------------------------------------
 * StLaneKeep
 *---------------------------------------------------------------------------*/
class StLaneKeep : public sc::state< StLaneKeep, StDrive >, StBase<StLaneKeep> {
public:
    //on enter
    StLaneKeep(my_context ctx);
    //on exit
    virtual ~StLaneKeep();

    //reactions
    sc::result react(const EvSysTick& evt);

    //reactions
    typedef mpl::list
    <
    sc::custom_reaction< EvSysTick >
    > reactions;
};

#endif //PATH_PLANNING_STDRIVE_H
