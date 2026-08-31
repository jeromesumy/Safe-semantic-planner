#pragma once
#include <cstdint>
#include <vector>
#include <unordered_set>
#include "state.h"
#include "transition.h"

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    PlanningProblem() : initialState(0), goalState(0) {}

    std::unordered_set<uint64_t> badStateSet() const {
        return std::unordered_set<uint64_t>(badStates.begin(), badStates.end());
    }
};
