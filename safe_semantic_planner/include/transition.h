#pragma once
#include <cstdint>

class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // static per-edge safety score, higher = safer, range [0,1]
    double reliability;   // range [0,1]
    bool available;

    Transition()
        : id(0), from(0), to(0), cost(0.0), safety(1.0),
          reliability(1.0), available(true) {}

    Transition(uint64_t id_, uint64_t from_, uint64_t to_, double cost_,
               double safety_ = 1.0, double reliability_ = 1.0, bool available_ = true)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_) {}
};
