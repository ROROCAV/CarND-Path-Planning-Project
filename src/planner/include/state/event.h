//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_EVENT_H
#define PATH_PLANNING_EVENT_H

#include <boost/statechart/event.hpp>

namespace sc = boost::statechart;

class EvStop : public sc::event<EvStop> {
};

class EvDrive : public sc::event<EvDrive> {
};

class EvPause : public sc::event<EvPause> {
};

class EvActivate : public sc::event<EvActivate> {
};

class EvSysTick : public sc::event<EvSysTick> {
};

class EvTemplate : public sc::event<EvTemplate> {
};

#endif //PATH_PLANNING_EVENT_H
