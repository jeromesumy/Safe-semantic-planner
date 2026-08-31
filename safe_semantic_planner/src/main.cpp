#include <iostream>
#include <iomanip>
#include "lpastar_planner.h"

static void printResult(const std::string& label, const PlanningResult& r) {
    std::cout << "== " << label << " ==\n";
    std::cout << "  success: " << (r.success ? "true" : "false") << "\n";
    if (r.success) {
        std::cout << "  path: ";
        for (size_t i = 0; i < r.statePath.size(); ++i) {
            std::cout << r.statePath[i] << (i + 1 < r.statePath.size() ? " -> " : "");
        }
        std::cout << "\n";
        std::cout << "  totalCost: " << std::fixed << std::setprecision(3) << r.totalCost << "\n";
        std::cout << "  minSafetyDistance: " << r.safetyScore << "\n";
    }
    std::cout << "  exploredStates: " << r.exploredStates << "\n";
    std::cout << "  planningTimeMs: " << std::fixed << std::setprecision(4) << r.planningTimeMs << "\n\n";
}

// Helper: state ids as named constants for readability.
enum : uint64_t { S = 1, A = 2, B = 3, C = 4, D = 5, G = 6, X = 7 };

int main() {
    std::cout << std::boolalpha;

    // ---------------- Test Case 1: Basic Reachability ----------------
    // S -> A -> B -> G
    {
        PlanningProblem p;
        p.initialState = S;
        p.goalState = G;
        p.states = {
            State(S, {0, 0}), State(A, {1, 0}), State(B, {2, 0}), State(G, {3, 0})
        };
        p.transitions = {
            Transition(1, S, A, 1.0),
            Transition(2, A, B, 1.0),
            Transition(3, B, G, 1.0),
        };
        LPAStarPlanner planner;
        printResult("Test 1: Basic Reachability", planner.plan(p));
    }

    // ---------------- Test Case 2: Bad State Avoidance ----------------
    // S -> A -> X -> G   (X is bad)
    // S -> C -> D -> G   (should be chosen)
    {
        PlanningProblem p;
        p.initialState = S;
        p.goalState = G;
        p.badStates = {X};
        p.states = {
            State(S, {0, 0}), State(A, {1, 0}), State(X, {2, 0}), State(G, {3, 0}),
            State(C, {0, 2}), State(D, {1, 2})
        };
        p.transitions = {
            Transition(1, S, A, 1.0),
            Transition(2, A, X, 1.0),
            Transition(3, X, G, 1.0),
            Transition(4, S, C, 1.0),
            Transition(5, C, D, 1.0),
            Transition(6, D, G, 1.0),
        };
        LPAStarPlanner planner;
        printResult("Test 2: Bad State Avoidance", planner.plan(p));
    }

    // ---------------- Test Case 3: Safety Margin ----------------
    // Path 1 (S-A2-G): lower cost, passes close to a bad state.
    // Path 2 (S-C2-G): higher cost, stays far from bad states.
    {
        enum : uint64_t { S3 = 10, A2 = 11, C2 = 12, G3 = 13, BAD = 14 };
        PlanningProblem p;
        p.initialState = S3;
        p.goalState = G3;
        p.badStates = {BAD};
        p.states = {
            State(S3, {0, 0}), State(A2, {1, 0.1}), State(G3, {2, 0}),
            State(C2, {1, 5}), State(BAD, {1, 0})
        };
        p.transitions = {
            Transition(1, S3, A2, 1.0),   // cheap but close to BAD
            Transition(2, A2, G3, 1.0),
            Transition(3, S3, C2, 3.0),   // expensive but far from BAD
            Transition(4, C2, G3, 3.0),
        };
        LPAStarPlanner planner;
        auto r = planner.plan(p);
        printResult("Test 3: Safety Margin", r);
        std::cout << "  (Planner trades cost for safety via safetyWeight; "
                     "increasing safetyWeight favors the C2 route.)\n\n";
    }

    // ---------------- Test Case 4: Dynamic Transition ----------------
    // S -> A -> G, then (A,G) becomes unavailable; expect alternate route S -> A -> B -> G
    {
        PlanningProblem p;
        p.initialState = S;
        p.goalState = G;
        p.states = {
            State(S, {0, 0}), State(A, {1, 0}), State(B, {2, 1}), State(G, {3, 0})
        };
        p.transitions = {
            Transition(1, S, A, 1.0),
            Transition(2, A, G, 1.0),
            Transition(3, A, B, 1.0),
            Transition(4, B, G, 1.0),
        };
        LPAStarPlanner planner;
        printResult("Test 4a: Dynamic Transition (before)", planner.plan(p));

        planner.setTransitionAvailability(2, false); // (A,G) unavailable
        printResult("Test 4b: Dynamic Transition (after A->G removed, incremental replan)",
                    planner.replan());
    }

    // ---------------- Test Case 5: Goal Update ----------------
    // Goal changes from G to B mid-execution; planner should not rebuild all
    // structures (see setGoal()'s incremental key-refresh implementation).
    {
        enum : uint64_t { S5 = 20, A5 = 21, B5 = 22, G5 = 23 };
        PlanningProblem p;
        p.initialState = S5;
        p.goalState = G5;
        p.states = {
            State(S5, {0, 0}), State(A5, {1, 0}), State(B5, {2, 0}), State(G5, {3, 0})
        };
        p.transitions = {
            Transition(1, S5, A5, 1.0),
            Transition(2, A5, B5, 1.0),
            Transition(3, B5, G5, 1.0),
        };
        LPAStarPlanner planner;
        printResult("Test 5a: Goal Update (original goal G)", planner.plan(p));

        planner.setGoal(B5);
        printResult("Test 5b: Goal Update (goal changed to B, incremental)", planner.replan());
    }

    // ---------------- Test Case 6: Transition Addition ----------------
    // S -> A -> B -> G exists; add shortcut S -> G directly; expect improved solution.
    {
        PlanningProblem p;
        p.initialState = S;
        p.goalState = G;
        p.states = {
            State(S, {0, 0}), State(A, {1, 0}), State(B, {2, 0}), State(G, {3, 0})
        };
        p.transitions = {
            Transition(1, S, A, 1.0),
            Transition(2, A, B, 1.0),
            Transition(3, B, G, 1.0),
        };
        LPAStarPlanner planner;
        printResult("Test 6a: Transition Addition (before shortcut)", planner.plan(p));

        planner.addTransition(Transition(4, S, G, 0.5)); // cheap shortcut
        printResult("Test 6b: Transition Addition (after shortcut, incremental replan)",
                    planner.replan());
    }

    return 0;
}
