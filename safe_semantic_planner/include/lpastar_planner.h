#pragma once
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <limits>
#include "planner.h"

// Lifelong Planning A* (LPA*) adapted with a safety-aware edge cost model.
//
// Edge weight for transition (u -> v):
//   INF                                        if !t.available or v is a bad state
//   costWeight    * t.cost
// + safetyWeight  * safetyPenalty(v)            (higher when v is close to a bad state)
// + reliabilityWeight * (1 - t.reliability)
//
// safetyPenalty(v) = 1 / (1 + minDistanceToNearestBadState(v))
//
// The heuristic h(s) = costWeight * euclideanDistance(s, goal) guides search order.
class LPAStarPlanner : public Planner {
public:
    struct Weights {
        double costWeight = 1.0;
        double safetyWeight = 5.0;
        double reliabilityWeight = 2.0;
    };

    LPAStarPlanner();
    explicit LPAStarPlanner(Weights weights);

    // Full (cold) plan: builds internal graph structures from scratch and solves.
    PlanningResult plan(const PlanningProblem& problem) override;

    // --- Incremental / dynamic environment updates -------------------------
    // Each of these repairs the search incrementally (LPA*'s core feature)
    // and does NOT rebuild the priority queue or graph from scratch.
    void setTransitionAvailability(uint64_t transitionId, bool available);
    void updateTransitionCost(uint64_t transitionId, double newCost);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    void setBadStates(const std::vector<uint64_t>& badStates);
    void setGoal(uint64_t newGoal);

    // Re-run ComputeShortestPath with whatever incremental updates have been
    // queued and extract the resulting path. Cheap relative to plan().
    PlanningResult replan();

private:
    struct Key {
        double k1;
        double k2;
        bool operator<(const Key& other) const {
            if (k1 != other.k1) return k1 < other.k1;
            return k2 < other.k2;
        }
        bool operator>=(const Key& other) const {
            return !(*this < other);
        }
    };

    Weights weights_;
    static constexpr double INF = std::numeric_limits<double>::infinity();

    std::unordered_map<uint64_t, State> statesById_;
    std::unordered_map<uint64_t, std::vector<Transition>> succEdges_; // from -> transitions
    std::unordered_map<uint64_t, std::vector<Transition>> predEdges_; // to -> transitions
    std::unordered_map<uint64_t, Transition> transitionsById_;

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    std::unordered_set<uint64_t> badSet_;

    // Priority queue implemented as an ordered set of (key, id) with a side
    // table for the currently-registered key of each queued id, so we can
    // remove-then-reinsert on key changes (standard LPA* OPEN list pattern).
    std::set<std::pair<Key, uint64_t>> open_;
    std::unordered_map<uint64_t, Key> openKeys_;

    uint64_t startId_ = 0;
    uint64_t goalId_ = 0;
    bool initialized_ = false;
    int lastExploredCount_ = 0;

    double getG(uint64_t s) const;
    double getRhs(uint64_t s) const;
    double euclideanToGoal(uint64_t s) const;
    double edgeWeight(const Transition& t) const;
    double minDistanceToNearestBadState(uint64_t s) const;

    Key calcKey(uint64_t s) const;
    void insertOrUpdateOpen(uint64_t s, const Key& k);
    void removeFromOpen(uint64_t s);
    void updateVertex(uint64_t u);
    void computeShortestPath();
    PlanningResult extractPath() const;

    void buildGraph(const PlanningProblem& problem);
};
