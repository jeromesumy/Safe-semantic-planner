#include "lpastar_planner.h"
#include <algorithm>
#include <chrono>

LPAStarPlanner::LPAStarPlanner() : weights_(Weights()) {}
LPAStarPlanner::LPAStarPlanner(Weights weights) : weights_(weights) {}

double LPAStarPlanner::getG(uint64_t s) const {
    auto it = g_.find(s);
    return it == g_.end() ? INF : it->second;
}

double LPAStarPlanner::getRhs(uint64_t s) const {
    auto it = rhs_.find(s);
    return it == rhs_.end() ? INF : it->second;
}

double LPAStarPlanner::minDistanceToNearestBadState(uint64_t s) const {
    if (badSet_.empty()) return INF;
    auto itS = statesById_.find(s);
    if (itS == statesById_.end()) return INF;
    double best = INF;
    for (uint64_t b : badSet_) {
        auto itB = statesById_.find(b);
        if (itB == statesById_.end()) continue;
        double d = State::euclidean(itS->second, itB->second);
        best = std::min(best, d);
    }
    return best;
}

double LPAStarPlanner::edgeWeight(const Transition& t) const {
    if (!t.available) return INF;
    if (badSet_.count(t.to)) return INF;

    double d = minDistanceToNearestBadState(t.to);
    double safetyPenalty = (d == INF) ? 0.0 : 1.0 / (1.0 + d);

    double w = weights_.costWeight * t.cost
             + weights_.safetyWeight * safetyPenalty
             + weights_.reliabilityWeight * (1.0 - t.reliability);
    return w;
}

double LPAStarPlanner::euclideanToGoal(uint64_t s) const {
    auto itS = statesById_.find(s);
    auto itG = statesById_.find(goalId_);
    if (itS == statesById_.end() || itG == statesById_.end()) return 0.0;
    return weights_.costWeight * State::euclidean(itS->second, itG->second);
}

LPAStarPlanner::Key LPAStarPlanner::calcKey(uint64_t s) const {
    double m = std::min(getG(s), getRhs(s));
    if (m == INF) return Key{INF, INF};
    return Key{m + euclideanToGoal(s), m};
}

void LPAStarPlanner::insertOrUpdateOpen(uint64_t s, const Key& k) {
    removeFromOpen(s);
    open_.insert({k, s});
    openKeys_[s] = k;
}

void LPAStarPlanner::removeFromOpen(uint64_t s) {
    auto it = openKeys_.find(s);
    if (it != openKeys_.end()) {
        open_.erase({it->second, s});
        openKeys_.erase(it);
    }
}

void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != startId_) {
        double best = INF;
        auto it = predEdges_.find(u);
        if (it != predEdges_.end()) {
            for (const auto& t : it->second) {
                double w = edgeWeight(t);
                if (w == INF) continue;
                best = std::min(best, getG(t.from) + w);
            }
        }
        rhs_[u] = best;
    }
    bool inOpen = openKeys_.count(u) > 0;
    bool consistent = (getG(u) == getRhs(u));
    if (inOpen) removeFromOpen(u);
    if (!consistent) {
        insertOrUpdateOpen(u, calcKey(u));
    }
}

void LPAStarPlanner::computeShortestPath() {
    lastExploredCount_ = 0;
    while (!open_.empty() &&
           (open_.begin()->first < calcKey(goalId_) || getRhs(goalId_) != getG(goalId_))) {
        auto top = *open_.begin();
        uint64_t u = top.second;
        Key kOld = top.first;
        Key kNew = calcKey(u);

        if (kOld < kNew) {
            insertOrUpdateOpen(u, kNew);
        } else if (getG(u) > getRhs(u)) {
            g_[u] = getRhs(u);
            removeFromOpen(u);
            lastExploredCount_++;
            auto it = succEdges_.find(u);
            if (it != succEdges_.end()) {
                for (const auto& t : it->second) updateVertex(t.to);
            }
        } else {
            g_[u] = INF;
            lastExploredCount_++;
            updateVertex(u);
            auto it = succEdges_.find(u);
            if (it != succEdges_.end()) {
                for (const auto& t : it->second) updateVertex(t.to);
            }
        }
    }
}

PlanningResult LPAStarPlanner::extractPath() const {
    PlanningResult result;
    result.exploredStates = lastExploredCount_;

    if (getG(goalId_) == INF) {
        result.success = false;
        return result;
    }

    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    uint64_t cur = goalId_;
    statePath.push_back(cur);
    double minSafetyDist = INF;

    int guard = 0;
    const int maxSteps = static_cast<int>(statesById_.size()) + 5;
    while (cur != startId_) {
        auto it = predEdges_.find(cur);
        if (it == predEdges_.end()) { result.success = false; return result; }

        double bestVal = INF;
        const Transition* bestT = nullptr;
        for (const auto& t : it->second) {
            double w = edgeWeight(t);
            if (w == INF) continue;
            double val = getG(t.from) + w;
            if (val < bestVal - 1e-12) {
                bestVal = val;
                bestT = &t;
            }
        }
        if (!bestT) { result.success = false; return result; }

        transitionPath.push_back(bestT->id);
        cur = bestT->from;
        statePath.push_back(cur);

        if (++guard > maxSteps) { result.success = false; return result; }
    }

    std::reverse(statePath.begin(), statePath.end());
    std::reverse(transitionPath.begin(), transitionPath.end());

    for (uint64_t s : statePath) {
        double d = minDistanceToNearestBadState(s);
        if (d != INF) minSafetyDist = std::min(minSafetyDist, d);
    }
    if (minSafetyDist == INF) minSafetyDist = 0.0;

    result.success = true;
    result.statePath = statePath;
    result.transitionPath = transitionPath;
    result.totalCost = getG(goalId_);
    result.safetyScore = minSafetyDist;
    return result;
}

void LPAStarPlanner::buildGraph(const PlanningProblem& problem) {
    statesById_.clear();
    succEdges_.clear();
    predEdges_.clear();
    transitionsById_.clear();
    g_.clear();
    rhs_.clear();
    open_.clear();
    openKeys_.clear();

    for (const auto& s : problem.states) statesById_[s.id] = s;
    badSet_ = problem.badStateSet();

    for (const auto& t : problem.transitions) {
        succEdges_[t.from].push_back(t);
        predEdges_[t.to].push_back(t);
        transitionsById_[t.id] = t;
    }

    startId_ = problem.initialState;
    goalId_ = problem.goalState;

    rhs_[startId_] = 0.0;
    insertOrUpdateOpen(startId_, calcKey(startId_));

    initialized_ = true;
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    auto t0 = std::chrono::steady_clock::now();
    buildGraph(problem);
    computeShortestPath();
    PlanningResult result = extractPath();
    auto t1 = std::chrono::steady_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult LPAStarPlanner::replan() {
    auto t0 = std::chrono::steady_clock::now();
    computeShortestPath();
    PlanningResult result = extractPath();
    auto t1 = std::chrono::steady_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

// --- Incremental update operations -----------------------------------------

void LPAStarPlanner::setTransitionAvailability(uint64_t transitionId, bool available) {
    auto it = transitionsById_.find(transitionId);
    if (it == transitionsById_.end()) return;
    it->second.available = available;

    for (auto& t : succEdges_[it->second.from]) if (t.id == transitionId) t.available = available;
    for (auto& t : predEdges_[it->second.to]) if (t.id == transitionId) t.available = available;

    updateVertex(it->second.to);
}

void LPAStarPlanner::updateTransitionCost(uint64_t transitionId, double newCost) {
    auto it = transitionsById_.find(transitionId);
    if (it == transitionsById_.end()) return;
    it->second.cost = newCost;

    for (auto& t : succEdges_[it->second.from]) if (t.id == transitionId) t.cost = newCost;
    for (auto& t : predEdges_[it->second.to]) if (t.id == transitionId) t.cost = newCost;

    updateVertex(it->second.to);
}

void LPAStarPlanner::addTransition(const Transition& t) {
    transitionsById_[t.id] = t;
    succEdges_[t.from].push_back(t);
    predEdges_[t.to].push_back(t);
    // A new state referenced by the transition but not yet known gets a
    // trivial default embedding at the origin; callers should normally
    // register states via the PlanningProblem beforehand.
    updateVertex(t.to);
}

void LPAStarPlanner::removeTransition(uint64_t transitionId) {
    auto it = transitionsById_.find(transitionId);
    if (it == transitionsById_.end()) return;
    uint64_t from = it->second.from, to = it->second.to;

    auto& succ = succEdges_[from];
    succ.erase(std::remove_if(succ.begin(), succ.end(),
               [&](const Transition& t){ return t.id == transitionId; }), succ.end());
    auto& pred = predEdges_[to];
    pred.erase(std::remove_if(pred.begin(), pred.end(),
               [&](const Transition& t){ return t.id == transitionId; }), pred.end());

    transitionsById_.erase(it);
    updateVertex(to);
}

void LPAStarPlanner::setBadStates(const std::vector<uint64_t>& badStates) {
    // Any state whose "distance to nearest bad state" may have changed needs
    // its incoming edges re-evaluated. We conservatively re-evaluate every
    // vertex that has predecessors, which is cheap relative to a full replan
    // and still avoids rebuilding the graph or priority-queue structures.
    badSet_ = std::unordered_set<uint64_t>(badStates.begin(), badStates.end());
    for (const auto& kv : predEdges_) {
        updateVertex(kv.first);
    }
}

void LPAStarPlanner::setGoal(uint64_t newGoal) {
    // g/rhs values encode shortest distance from the start state and do not
    // depend on the goal, so they remain valid. Only the priority-queue keys
    // (which embed the heuristic to the goal) need to be refreshed.
    goalId_ = newGoal;
    std::vector<std::pair<Key, uint64_t>> entries(open_.begin(), open_.end());
    open_.clear();
    openKeys_.clear();
    for (auto& e : entries) {
        insertOrUpdateOpen(e.second, calcKey(e.second));
    }
    // If the new goal was never reached by the previous search it may not be
    // consistent (or even present in OPEN yet); make sure it's queued so
    // ComputeShortestPath will resolve it.
    if (getG(goalId_) != getRhs(goalId_)) {
        insertOrUpdateOpen(goalId_, calcKey(goalId_));
    }
}
