//
// Created by luyifan on 19-4-30.
//

#include "state/StWaitForRun.h"
#include "state/StRun.h"

StWaitForRun::StWaitForRun(my_context ctx) :
        my_base(ctx), StBase<StWaitForRun>(std::string("StWaitForRun")){

}
StWaitForRun::~StWaitForRun(){

}
sc::result StWaitForRun::react(const EvSysTick& evt){
    /*检测各方面是否存在异常*/
    Planner& planner = context<Planner>();
    std::cout<<"Running!!!"<<std::endl;
    //不用等待
    return transit<StRun>();
}
sc::result StWaitForRun::react(const sc::exception_thrown& evt){
    try {
        throw ;
    }
        /* we can catch special exceptions here and handle them
         catch ( const std::runtime_error & )
         {
         // only std::runtime_errors will lead to a transition
         // to Defective ...
         return transit< StError >();
         }*/
    catch ( ... ) {
        return forward_event();
    }
}