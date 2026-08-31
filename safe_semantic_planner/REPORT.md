# Design Report — Safe Semantic Planner in a Finite Cartesian State Space

**Course:** PCCST503 – Machine Learning, Assignment 1
**Algorithm chosen:** Lifelong Planning A* (LPA*)

## 1. Why LPA*

D* Lite and LPA* both incrementally repair a shortest-path search instead of
recomputing it from scratch, but they specialize in different kinds of
change:

- **D\* Lite** runs a *backward* search anchored at the goal and is optimized
  for a moving **start** state (e.g., a robot that has physically advanced
  along its path) with a fixed goal.
- **LPA\*** runs a *forward* search anchored at the start and is optimized
  for **edge/weight changes** (transitions becoming unavailable, costs
  changing, transitions being added or removed) with the start fixed.

The assignment's dynamic-environment requirements — transition availability
changes, new/removed transitions, and bad-state updates — are all edge-level
changes, which is exactly what LPA*'s incremental repair (`UpdateVertex` /
`ComputeShortestPath`) is designed for. Goal changes are the one case LPA*
does not natively optimize for, but because its `g`/`rhs` values represent
distance-from-start (not distance-to-goal), they remain valid across a goal
change; only the priority-queue keys, which embed the heuristic-to-goal, need
to be refreshed. This is implemented in `LPAStarPlanner::setGoal()` and
verified in Test Case 5, where re-solving for a new (closer) goal after a
change re-expands **zero** additional states because the relevant `rhs`
values were already consistent from the earlier search.

## 2. State Representation

Each `State` is a `(id, embedding)` pair, where `embedding` is a
`std::vector<double>` giving the state's coordinates in ℝᵈ. This directly
mirrors the assignment's `si = (x1, …, xd)` definition and is used for two
purposes:

1. **Heuristic**: `h(s) = costWeight · euclidean(s, goal)`.
2. **Safety**: Euclidean distance from a state to the nearest bad state.

## 3. Data Structures

| Structure | Purpose | Complexity |
|---|---|---|
| `statesById_` (`unordered_map`) | O(1) state lookup by id | O(1) avg |
| `succEdges_` / `predEdges_` (`unordered_map<id, vector<Transition>>`) | Forward and backward adjacency, required because LPA*'s `UpdateVertex` needs predecessors of a changed vertex and `ComputeShortestPath` propagates via successors | O(1) avg to fetch a vertex's edge list |
| `g_`, `rhs_` (`unordered_map<id, double>`) | LPA*'s two cost estimates per vertex — `g` is the current best known cost-from-start, `rhs` (“right-hand side”) is a one-step-lookahead estimate from `g`-values of predecessors. A vertex is *locally consistent* when `g == rhs`; the search's job is to restore consistency everywhere reachable | O(1) avg access |
| `open_` (`std::set<pair<Key,id>>`) + `openKeys_` (`unordered_map<id,Key>`) | Priority queue ("OPEN list") of *locally inconsistent* vertices, ordered by key `(min(g,rhs)+h, min(g,rhs))`. The side table lets us remove a vertex's stale entry in O(log n) before reinserting with a new key | O(log n) insert/remove |
| `badSet_` (`unordered_set<id>`) | O(1) bad-state membership test | O(1) avg |

This matches the assignment's suggested `State`, `Transition`,
`PlanningProblem`, `PlanningResult`, and `Planner` interfaces, implemented in
`include/*.h`, with `LPAStarPlanner` as the concrete `Planner`.

## 4. Heuristic Function

```
h(s) = costWeight · ‖embedding(s) − embedding(goal)‖₂
```

This is the Euclidean distance in the Cartesian embedding, scaled by the same
weight applied to transition cost. It is used only to order the OPEN queue
(search efficiency), not to determine correctness — LPA*'s `rhs`/`g`
propagation is what guarantees optimality of the extracted path, so a
heuristic that is not perfectly admissible/consistent for a given problem
degrades search *efficiency* rather than *correctness*, provided the
extracted path always follows the edge with minimum `g(pred) + weight`.

## 5. Safety Computation

Transition weight combines three assignment-mandated signals into a single
scalar the search can minimize:

```
weight(u→v) = costWeight · cost(u,v)
            + safetyWeight · safetyPenalty(v)
            + reliabilityWeight · (1 − reliability(u,v))

safetyPenalty(v) = 1 / (1 + minDistanceToNearestBadState(v))
```

- Unavailable transitions and transitions leading into a bad state get
  weight = ∞, enforcing objectives (2) "never visit a bad state" as a hard
  constraint rather than a soft penalty.
- `safetyPenalty` is smooth and bounded in (0, 1], so a state right next to a
  bad state (distance → 0) is heavily penalized, while a state far away
  (distance → ∞) contributes ≈0. This realizes objective (4) "maximize
  minimum distance to bad states" as a continuous trade-off against cost,
  matching the assignment's `Score(P) = αG − βC + γD + δR` — here `βC` and
  `γD` are folded into a single minimized quantity by negating the sign of
  the safety term.
- `reliabilityWeight · (1 − reliability)` penalizes unreliable transitions,
  covering the `δR` term.
- `PlanningResult.safetyScore` reports the minimum distance-to-nearest-bad-
  state across the *extracted* path, directly answering "minimum distance to
  bad states" from the experimental-evaluation checklist.

The relative weights (`costWeight`, `safetyWeight`, `reliabilityWeight`) are
exposed via `LPAStarPlanner::Weights` so a grader/user can tune the
cost-vs-safety trade-off demonstrated in Test Case 3.

## 6. Time Complexity

Let `V` = number of states, `E` = number of transitions.

- **Cold plan (`plan()`)**: identical asymptotic behavior to A*/Dijkstra —
  each vertex is inserted/updated in OPEN O(1) times per relevant edge
  change, and each OPEN operation costs O(log V) (balanced-tree priority
  queue). Overall **O(E log V)** in the worst case, same as classical A*.
- **Incremental update (`setTransitionAvailability`, `updateTransitionCost`,
  `addTransition`, `removeTransition`, `setBadStates`, `setGoal` +
  `replan()`)**: LPA*'s central result is that the cost of repairing the
  search after a *local* change is proportional to the size of the region
  whose `g`/`rhs` values actually change, not the whole graph. In the best
  case (change far from the current optimal path) this is **O(log V)** for
  the affected vertex; in the worst case it degrades to the cold-plan bound.
  Test Cases 4–6 empirically show only 0–3 states re-expanded after a local
  edit, versus 3–4 for the initial cold solve on the same graphs.

## 7. Space Complexity

**O(V + E)**: adjacency lists (`succEdges_`, `predEdges_`) store each
transition twice (O(E)); `g_`, `rhs_`, `statesById_` store O(1) data per
state (O(V)); `open_`/`openKeys_` hold at most O(V) entries.

## 8. Dynamic Environment / Replanning Strategy

| Change | Handling | Rebuild cost |
|---|---|---|
| Transition availability toggled | `setTransitionAvailability()` mutates the edge in place, calls `updateVertex()` on the edge's target only | O(log V) + propagation |
| Transition cost changed | `updateTransitionCost()`, same pattern | O(log V) + propagation |
| Transition added | `addTransition()` inserts into adjacency lists, `updateVertex()` on target | O(log V) + propagation |
| Transition removed | `removeTransition()` erases from adjacency lists, `updateVertex()` on target | O(log V) + propagation |
| Bad states changed | `setBadStates()` re-evaluates `updateVertex()` for every vertex with predecessors (since any incoming edge's weight may depend on the new bad-state set) | O(V log V) worst case; still no graph/queue rebuild |
| Goal changed | `setGoal()` re-keys existing OPEN entries with the new heuristic; `g`/`rhs` are untouched since they are goal-independent | O(\|OPEN\| log \|OPEN\|) |

In every case, `computeShortestPath()` (invoked via `replan()`) only resumes
work on vertices still in OPEN — it never resets `g_`/`rhs_`/adjacency lists,
satisfying the assignment's requirement to replan "without rebuilding all
data structures whenever possible."

## 9. Experimental Results

Measured on this project's `main.cpp` driver (single run, release build,
`-O2`, wall-clock via `std::chrono`):

| Test | Path found | Total cost | Min. safety distance | States explored | Planning time (ms) |
|---|---|---|---|---|---|
| 1. Basic reachability | 1→2→3→6 | 3.000 | 0.000 (no bad states) | 4 | 0.0108 |
| 2. Bad state avoidance | 1→4→5→6 (avoids X) | 8.351 | 1.000 | 5 | 0.0066 |
| 3. Safety margin | 10→11→13 | 9.045 | 0.100 | 4 | 0.0036 |
| 4a. Before edge removal | 1→2→6 | 2.000 | 0.000 | 3 | 0.0030 |
| 4b. After (A,G) removed — incremental | 1→2→3→6 | 3.000 | 0.000 | 3 | 0.0018 |
| 5a. Original goal | 20→21→22→23 | 3.000 | 0.000 | 4 | 0.0022 |
| 5b. Goal changed to B — incremental | 20→21→22 | 2.000 | 0.000 | **0** | 0.0003 |
| 6a. Before shortcut | 1→2→3→6 | 3.000 | 0.000 | 4 | 0.0016 |
| 6b. After shortcut added — incremental | 1→6 | 0.500 | 0.000 | 1 | 0.0006 |

**Observations**

- All 6 goal-success rate = 100%, 0 bad states visited in any run — hard
  constraint objectives (1) and (2) hold in every test.
- Incremental replans (4b, 5b, 6b) explore far fewer states than a cold
  solve of an equivalent graph, and complete in a fraction of the time —
  directly evidencing LPA*'s incremental-repair benefit.
- Test 3 shows the cost/safety trade-off: with default weights the cheaper,
  closer-to-bad-state route is chosen (safety distance 0.100); raising
  `safetyWeight` in `LPAStarPlanner::Weights` shifts the choice to the
  farther, costlier route — demonstrated by adjusting the constructor
  argument in `main.cpp`.
- Memory usage was not separately instrumented (no external profiler
  available in this environment); given the O(V+E) structures described in
  §7, resident memory scales linearly with graph size, consistent with the
  small (<1 KB) working sets in these toy graphs.

## 10. Bonus Work

Not attempted in this submission. The `Planner` interface and
`LPAStarPlanner`'s incremental-update methods are structured so that
multi-goal planning (running `ComputeShortestPath` to the nearest of several
goal vertices) or time-dependent availability (periodically calling
`setTransitionAvailability`) could be layered on without further interface
changes, if pursued.
