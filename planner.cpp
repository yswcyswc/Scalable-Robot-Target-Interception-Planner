/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <cmath>
#include <cstdint>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (((Y)-1)*(XSIZE) + ((X)-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9

static inline bool inBounds(int x, int y, int xs, int ys) {
    return (x >= 1 && x <= xs && y >= 1 && y <= ys);
}

static inline bool traversable(const int* map, int collision_thresh, int idx) {
    int c = map[idx];
    return (c >= 0 && c < collision_thresh);
}

static inline int idxToX(int idx, int x_size) { return (idx % x_size) + 1; }
static inline int idxToY(int idx, int x_size) { return (idx / x_size) + 1; }

static inline int euclidDist(int x1, int y1, int x2, int y2) {
    int dx = x1 - x2;
    int dy = y1 - y2;
    return dx*dx + dy*dy;
}

static std::vector<std::pair<int,int>> g_plan;
static size_t g_plan_pos = 0;
static int g_plan_time = -1;

struct PQNode {
    int idx;
    int steps;
    int64_t cost;
};

struct PQCmp {
    bool operator()(const PQNode& a, const PQNode& b) const {
        if (a.steps != b.steps) return a.steps > b.steps; // fewer steps first
        return a.cost > b.cost; // then lower cost
    }
};

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
)
{
    if (robotposeX == targetposeX && robotposeY == targetposeY) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        g_plan.clear(); g_plan_pos = 0; g_plan_time = -1;
        return;
    }

    const int dX[NUMOFDIRS] = {-1,-1,-1, 0, 0, 1, 1, 1, 0};
    const int dY[NUMOFDIRS] = {-1, 0, 1,-1, 1,-1, 0, 1, 0};

    const int N = x_size * y_size;
    const int startIdx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);

    if (!inBounds(robotposeX, robotposeY, x_size, y_size) ||
        !traversable(map, collision_thresh, startIdx)) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        g_plan.clear(); g_plan_pos = 0; g_plan_time = -1;
        return;
    }

    // Follow cached plan if valid for this timestep
    if (!g_plan.empty() && g_plan_time == curr_time && g_plan_pos < g_plan.size()) {
        int nx = g_plan[g_plan_pos].first;
        int ny = g_plan[g_plan_pos].second;

        int dx = nx - robotposeX;
        int dy = ny - robotposeY;

        if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1 &&
            inBounds(nx, ny, x_size, y_size)) {

            int nidx = GETMAPINDEX(nx, ny, x_size, y_size);
            if (traversable(map, collision_thresh, nidx)) {
                action_ptr[0] = nx;
                action_ptr[1] = ny;
                g_plan_pos++;
                g_plan_time = curr_time + 1;
                return;
            }
        }

        g_plan.clear(); g_plan_pos = 0; g_plan_time = -1;
    }

    const int remaining = (target_steps - 1) - curr_time;
    if (remaining <= 0) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        g_plan.clear(); g_plan_pos = 0; g_plan_time = -1;
        return;
    }

    const int INF_STEPS = std::numeric_limits<int>::max();
    const int64_t INF_COST = (int64_t)9e18;

    // Earliest k>=0 such that target is at idx at time curr_time+k
    std::vector<int> goalAtIdx(N, INF_STEPS);
    for (int t = curr_time; t < target_steps; t++) {
        int k = t - curr_time;
        if (k > remaining) break;

        int tx = target_traj[t];
        int ty = target_traj[t + target_steps];
        if (!inBounds(tx, ty, x_size, y_size)) continue;

        int gidx = GETMAPINDEX(tx, ty, x_size, y_size);
        if (!traversable(map, collision_thresh, gidx)) continue;

        if (k < goalAtIdx[gidx]) goalAtIdx[gidx] = k;
    }

    std::vector<int> bestSteps(N, INF_STEPS);
    std::vector<int64_t> bestCost(N, INF_COST);
    std::vector<int> parent(N, -1);

    std::priority_queue<PQNode, std::vector<PQNode>, PQCmp> pq;
    bestSteps[startIdx] = 0;
    bestCost[startIdx] = 0;
    pq.push({startIdx, 0, 0});

    int bestGoalIdx = -1;
    int bestK = INF_STEPS;
    int64_t bestScore = INF_COST; // move cost + wait cost

    while (!pq.empty()) {
        PQNode cur = pq.top(); pq.pop();

        if (cur.steps != bestSteps[cur.idx] || cur.cost != bestCost[cur.idx]) continue;
        if (cur.steps > remaining) continue;
        if (bestScore != INF_COST && cur.cost >= bestScore) continue;

        int k = goalAtIdx[cur.idx];
        if (k != INF_STEPS && cur.steps <= k) {
            int waitSteps = k - cur.steps;
            int64_t score = cur.cost + (int64_t)waitSteps * (int64_t)map[cur.idx];
            if (score < bestScore || (score == bestScore && k < bestK)) {
                bestScore = score;
                bestK = k;
                bestGoalIdx = cur.idx;
            }
        }

        int x = idxToX(cur.idx, x_size);
        int y = idxToY(cur.idx, x_size);

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int nx = x + dX[dir];
            int ny = y + dY[dir];
            if (!inBounds(nx, ny, x_size, y_size)) continue;

            int nidx = GETMAPINDEX(nx, ny, x_size, y_size);
            if (!traversable(map, collision_thresh, nidx)) continue;

            int ns = cur.steps + 1;
            if (ns > remaining) continue;

            int64_t nc = cur.cost + (int64_t)map[nidx];

            if (ns < bestSteps[nidx] || (ns == bestSteps[nidx] && nc < bestCost[nidx])) {
                bestSteps[nidx] = ns;
                bestCost[nidx]  = nc;
                parent[nidx]    = cur.idx;
                pq.push({nidx, ns, nc});
            }
        }
    }

    // fallback 1
    if (bestGoalIdx < 0) {
        if (inBounds(targetposeX, targetposeY, x_size, y_size)) {
            int goalIdx = GETMAPINDEX(targetposeX, targetposeY, x_size, y_size);
            if (traversable(map, collision_thresh, goalIdx) && bestSteps[goalIdx] != INF_STEPS) {
                bestGoalIdx = goalIdx;
                bestK = bestSteps[goalIdx]; // no extra wait appended
            }
        }
    }

    // fallback 2
    if (bestGoalIdx < 0) {
        int bestNX = robotposeX;
        int bestNY = robotposeY;
        int bestD  = euclidDist(robotposeX, robotposeY, targetposeX, targetposeY);
        int bestC  = map[startIdx];

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int nx = robotposeX + dX[dir];
            int ny = robotposeY + dY[dir];
            if (!inBounds(nx, ny, x_size, y_size)) continue;

            int nidx = GETMAPINDEX(nx, ny, x_size, y_size);
            if (!traversable(map, collision_thresh, nidx)) continue;

            int d = euclidDist(nx, ny, targetposeX, targetposeY);
            int c = map[nidx];

            if (d < bestD || (d == bestD && c < bestC)) {
                bestD = d;
                bestC = c;
                bestNX = nx;
                bestNY = ny;
            }
        }

        action_ptr[0] = bestNX;
        action_ptr[1] = bestNY;

        g_plan.clear(); g_plan_pos = 0; g_plan_time = curr_time + 1;
        return;
    }

    // Reconstruct path
    std::vector<int> rev;
    int cur = bestGoalIdx;
    while (cur != -1 && cur != startIdx) {
        rev.push_back(cur);
        cur = parent[cur];
    }

    if (cur != startIdx) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        g_plan.clear(); g_plan_pos = 0; g_plan_time = -1;
        return;
    }

    std::reverse(rev.begin(), rev.end());
    g_plan.clear();

    for (int id : rev) {
        g_plan.push_back({idxToX(id, x_size), idxToY(id, x_size)});
    }

    int arrivalSteps = (int)rev.size();
    int waitSteps = 0;
    if (bestK != INF_STEPS && arrivalSteps <= bestK) waitSteps = bestK - arrivalSteps;

    if (waitSteps > 0) {
        int wx = idxToX(bestGoalIdx, x_size);
        int wy = idxToY(bestGoalIdx, x_size);
        for (int i = 0; i < waitSteps; i++) g_plan.push_back({wx, wy});
    }

    g_plan_pos = 0;

    if (!g_plan.empty()) {
        action_ptr[0] = g_plan[g_plan_pos].first;
        action_ptr[1] = g_plan[g_plan_pos].second;
        g_plan_pos++;
        g_plan_time = curr_time + 1;
        return;
    }

    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    g_plan_time = curr_time + 1;
}
