//
// Created by luyifan on 19-4-30.
//

#include "state/StStop.h"

StStop::StStop(my_context ctx) : my_base(ctx), StBase<StStop> (std::string("StStop")) {}

StStop::~StStop() {}

//reaction
sc::result StStop::react(const EvSysTick &evt){

    return forward_event();
}

sc::result StStop::react(const sc::exception_thrown &evt){
    try {
        throw;
    }
        /* we can catch special exceptions here and handle them
         catch ( const std::runtime_error & )
         {
         // only std::runtime_errors will lead to a transition
         // to Defective ...
         return transit< StError >();
         }*/
    catch (...) {
        return forward_event();
    }
}