//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_STWAITFORRUN_H
#define PATH_PLANNING_STWAITFORRUN_H

#include "planner.h"
#include "StBase.h"

class StWaitForRun : public sc::state<StWaitForRun, Planner>, public StBase<StWaitForRun>{
public:
    //on enter
    StWaitForRun(my_context ctx);
    //exit
    ~StWaitForRun();
    //reaction
    sc::result react(const EvSysTick& evt);
    sc::result react(const sc::exception_thrown& evt);

    //reactions
    typedef mpl::list
    <
    sc::custom_reaction<EvSysTick>,
    sc::custom_reaction< sc::exception_thrown >
    > reactions;
};

#endif //PATH_PLANNING_STWAITFORRUN_H
