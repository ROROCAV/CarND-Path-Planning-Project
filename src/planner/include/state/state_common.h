//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_STATE_COMMON_H
#define PATH_PLANNING_STATE_COMMON_H

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/exception_translator.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/mpl/list.hpp>
#include "event.h"

namespace mpl = boost::mpl;
namespace sc = boost::statechart;

#endif //PATH_PLANNING_STATE_COMMON_H
