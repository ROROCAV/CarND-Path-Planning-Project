//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_STRUN_H
#define PATH_PLANNING_STRUN_H

#include "planner.h"
#include "common/common.h"
#include "StBase.h"

class StStart;
/*******************************
* StRun
* *****************************/
class StRun : public sc::state<StRun, Planner, StStart>, public StBase<StRun> {
public:
    // on enter
    StRun(my_context ctx);

    // on exit
    virtual ~StRun();

    // reaction
    sc::result react(const EvSysTick &evt);

    sc::result react(const sc::exception_thrown &evt);

    // reactions
    typedef mpl::list
    <
    sc::custom_reaction<EvSysTick>,
    sc::custom_reaction<sc::exception_thrown>
    > reactions;
};
/*******************************
* StStart
* *****************************/
class StStart : public sc::state<StStart, StRun>, public StBase<StStart> {
public:
    // on enter
    StStart(my_context ctx);

    // on exit
    ~StStart();

    // reaction
    sc::result react(const EvSysTick &evt);

    sc::result react(const sc::exception_thrown &evt);

    // reactions
    typedef mpl::list
    <
    sc::custom_reaction<EvSysTick>,
    sc::custom_reaction<sc::exception_thrown>
    > reactions;

private:
    Eigen::Vector2d start;
};




#endif //PATH_PLANNING_STRUN_H
