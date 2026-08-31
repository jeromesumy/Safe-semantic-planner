# Safe Semantic Planner — User Manual

A C++17 implementation of a safety-aware LPA* planner over a finite
Cartesian state space, per PCCST503 Assignment 1.

## Contents

```
safe_planner/
├── CMakeLists.txt
├── REPORT.md            (design report)
├── README.md            (this file)
├── include/
│   ├── state.h
│   ├── transition.h
│   ├── planning_problem.h
│   ├── planning_result.h
│   ├── planner.h            (abstract Planner interface)
│   └── lpastar_planner.h    (LPA* planner)
└── src/
    ├── lpastar_planner.cpp
    └── main.cpp              (6 illustrative test cases)
```

## Building

### Option A — CMake (recommended)

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
./safe_planner
```

### Option B — Direct g++ (no CMake required)

```bash
g++ -std=c++17 -O2 -Iinclude src/main.cpp src/lpastar_planner.cpp -o safe_planner
./safe_planner
```

Requires a C++17-capable compiler (tested with g++ 13).

## Running the Demonstration

Running the built `safe_planner` executable runs all six assignment test
cases in order and prints, for each: success flag, the resulting state path,
total cost, minimum safety distance to any bad state along the path, number
of states explored during that particular solve/replan, and wall-clock
planning time in milliseconds.

## Using the Planner in Your Own Code

```cpp
#include "lpastar_planner.h"

PlanningProblem problem;
problem.initialState = 1;
problem.goalState = 4;
problem.badStates = {5};
problem.states = {
    State(1, {0, 0}), State(2, {1, 0}), State(3, {2, 0}), State(4, {3, 0}),
    State(5, {1.5, 0})
};
problem.transitions = {
    Transition(/*id=*/1, /*from=*/1, /*to=*/2, /*cost=*/1.0, /*safety=*/1.0, /*reliability=*/0.95),
    Transition(2, 2, 3, 1.0),
    Transition(3, 3, 4, 1.0),
};

LPAStarPlanner planner;                 // default weights
PlanningResult result = planner.plan(problem);

if (result.success) {
    // result.statePath, result.transitionPath, result.totalCost, result.safetyScore
}
```

### Tuning the cost/safety trade-off

```cpp
LPAStarPlanner::Weights w;
w.costWeight = 1.0;
w.safetyWeight = 20.0;      // increase to favor routes farther from bad states
w.reliabilityWeight = 2.0;
LPAStarPlanner planner(w);
```

### Reacting to a dynamic environment

Call the relevant update method, then `replan()` (cheap — does **not**
rebuild the graph or priority queue from scratch):

```cpp
planner.setTransitionAvailability(/*transitionId=*/2, false);
PlanningResult updated = planner.replan();

planner.updateTransitionCost(3, 5.0);
planner.addTransition(Transition(10, 1, 4, 0.5));
planner.removeTransition(10);
planner.setBadStates({5, 6});
planner.setGoal(3);
PlanningResult again = planner.replan();
```

## Extending

`Planner` is an abstract interface (`virtual PlanningResult
plan(const PlanningProblem&) = 0`), so an alternative algorithm (e.g. D*
Lite, plain Dijkstra, or a learned heuristic) can be dropped in without
touching `main.cpp`'s test harness beyond swapping the concrete planner type.
