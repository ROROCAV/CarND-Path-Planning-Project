//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_STSTOP_H
#define PATH_PLANNING_STSTOP_H

#include "planner.h"
#include "StBase.h"
#include "StWaitForRun.h"

class StStop : public sc::state<StStop, Planner>, public StBase<StStop> {
public:
    //on enter
    StStop(my_context ctx);

    //on exit
    virtual ~StStop();

    //reaction
    sc::result react(const EvSysTick &evt);
    sc::result react(const sc::exception_thrown &evt);

    //reactions
    typedef mpl::list
    <
    sc::custom_reaction<EvSysTick>,
    sc::transition<EvActivate, StWaitForRun>,
    sc::custom_reaction<sc::exception_thrown>
    > reactions;
};

#endif //PATH_PLANNING_STSTOP_H
