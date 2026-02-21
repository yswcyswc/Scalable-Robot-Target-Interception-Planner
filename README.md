# Scalable Robot–Target Interception Planner

## Overview

This project implements a time-aware interception planner for a robot navigating a weighted 2D grid with obstacles. The objective is to intercept a moving target while minimizing accumulated traversal cost (movement + waiting), rather than minimizing arrival time.

At each timestep, the planner treats interception as a bounded multi-goal search over the remaining future timesteps of the target’s trajectory. For each traversable grid cell, it precomputes the earliest relative timestep `k` at which the target will occupy that cell. During search, a cell is considered a feasible interception if the robot can reach it in `s` steps and `s <= k`.

Waiting is explicitly modeled as remaining in place and paying the traversal cost of the interception cell for each extra timestep before the target arrives. The planner selects the feasible interception with minimum total cost, breaking ties by earlier interception time.

To improve efficiency and stability across timesteps, the planner:

- Bounds search by the remaining target horizon  
- Skips stale priority queue states  
- Caches valid plans across timesteps  
- Falls back to safe movement strategies when interception is infeasible  


## Build

```bash
g++ -O2 -std=c++17 runtest.cpp planner.cpp -o planner
```

## Run

```bash
./planner maps/map1.txt
```


## Visualization

```bash
python visualizer.py maps/map1.txt
```


## Example Results

The images below show example executions.

- The colored background represents traversal cost and obstacles.
- The green path is the robot trajectory.
- The yellow path is the target trajectory.

---

### Large Dense Cost Fields (Maps 1 & 2)

<p align="center">
  <img src="assets/map1.png" width="30%" />
  <img src="assets/map2.png" width="30%" />
</p>

These large maps contain extensive high-cost regions. The robot avoids expensive corridors and plans an interception that detours through lower-cost space rather than directly chasing the target. This demonstrates cost-aware routing under scale.


### Structured Cost Barriers (Maps 3 & 4)

<p align="center">
  <img src="assets/map3.png" width="30%" />
  <img src="assets/map4.png" width="30%" />
</p>

These maps emphasize rerouting around high-cost obstacles. The robot does not follow the geometrically shortest path; instead, it selects lower-cost detours and times interception accordingly. This highlights the planner’s cost-minimization objective.


### Limited Horizon / Constrained Interception (Maps 5 & 6)

<p align="center">
  <img src="assets/map5.png" width="30%" />
  <img src="assets/map6.png" width="30%" />
</p>

These scenarios have constrained geometry and limited remaining target time. The robot must choose interception points carefully because late arrival makes interception infeasible. The behavior reflects horizon-bounded reasoning rather than pure pursuit.


### Strong Cost Contrast Regions (Maps 7, 8 & 9)

<p align="center">
  <img src="assets/map7.png" width="25%" />
  <img src="assets/map8.png" width="25%" />
  <img src="assets/map9.png" width="25%" />
</p>

These maps contain large, sharply separated cost regions. The planner deliberately navigates along low-cost boundaries instead of cutting through expensive areas, even when that would be shorter in distance.


### Cluttered Environments (Maps 10, 11 & 12)

<p align="center">
  <img src="assets/map10.png" width="25%" />
  <img src="assets/map11.png" width="25%" />
  <img src="assets/map12.png" width="25%" />
</p>

These cluttered maps test robustness. The planner navigates around multiple obstacles while preserving feasibility within the remaining time horizon. Interception is selected only when both reachable and cost-effective.

## Quantitative Results

| Map | Caught | Time Taken | Moves Made | Path Cost |
|------|--------|------------|------------|------------|
| 1 | Yes | 2640 | 2639 | 2640 |
| 2 | Yes | 952 | 952 | 1847068 |
| 3 | Yes | 241 | 241 | 241 |
| 4 | Yes | 379 | 266 | 379 |
| 5 | Yes | 150 | 150 | 2551 |
| 6 | Yes | 70 | 70 | 2021 |
| 7 | Yes | 251 | 251 | 251 |
| 8 | Yes | 431 | 430 | 431 |
| 9 | Yes | 402 | 300 | 1602 |
| 10 | Yes | 97 | 96 | 97 |
| 11 | Yes | 63 | 62 | 63 |
| 12 | Yes | 101 | 80 | 101 |

All test maps were successfully intercepted.  
The results reflect the cost-minimization objective: in several cases, the robot takes more steps but incurs lower traversal cost compared to naive pursuit.

## Key Properties

- Time-aware multi-goal interception  
- Cost-minimizing objective (movement + waiting)  
- Horizon-bounded search  
- Lexicographic ordering by (steps, cost)  
- Plan caching across timesteps  
- Guaranteed valid output action  


## Summary

The planner balances feasibility, cost optimality, and runtime stability under dynamic constraints. It avoids naive chasing behavior by reasoning over future target occupancy, explicitly models waiting cost, and ensures robustness through bounded search and structured fallback strategies.