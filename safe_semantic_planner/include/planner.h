#pragma once
#include "planning_problem.h"
#include "planning_result.h"

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};
