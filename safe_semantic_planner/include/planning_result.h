#pragma once
#include <cstdint>
#include <vector>
#include <limits>

class PlanningResult {
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;     // minimum distance to nearest bad state along the path
    int exploredStates;     // diagnostic: states expanded during search
    double planningTimeMs;  // diagnostic: wall-clock planning time

    PlanningResult()
        : success(false),
          totalCost(std::numeric_limits<double>::infinity()),
          safetyScore(0.0),
          exploredStates(0),
          planningTimeMs(0.0) {}
};
