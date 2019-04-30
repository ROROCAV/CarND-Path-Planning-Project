//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_STBASE_H
#define PATH_PLANNING_STBASE_H

#include "state_common.h"
#include "common/common.h"

template<class DerivedState>
class StBase {
public:
    // on enter
    StBase(const std::string& n) : name_(n) {
        cout<<"[PLANNER] Entering "<<name()<<endl;
    };
    // on exit
    virtual ~StBase() {
        cout<<"[PLANNER] Exiting "<<name()<<endl ;
    };

    std::string name() const { return name_; };
    std::string getStateName() {
        std::cout <<name_<< std::endl;
    };

private:
    std::string name_;
};

#endif //PATH_PLANNING_STBASE_H
