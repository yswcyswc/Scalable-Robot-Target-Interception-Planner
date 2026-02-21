# Engineering Decisions

## Project Goal

The goal of this project is to design a planner that intercepts a moving target in a weighted 2D grid while minimizing total accumulated traversal cost (movement + waiting), rather than minimizing arrival time. The target follows a fixed trajectory and disappears after its final timestep, which creates a shrinking time horizon constraint.

## Challenge 1: Interception Is Time-Dependent

### Why it was difficult

The target moves every timestep, so planning toward its current position leads to chasing behavior and often misses better interception opportunities. A valid intercept must also occur before the target disappears.

### Options considered

- Always plan toward the target’s current position  
- Plan toward the final target position  
- Search over all future positions of the target  

### Chosen approach

I precompute, for each grid cell, the earliest future timestep at which the target will occupy that cell. During search, a cell is only considered if the robot can reach it within that time window (`s <= k`). This converts interception into a bounded multi-goal search problem and prevents wasted exploration.

## Challenge 2: Modeling Waiting Correctly

### Why it was difficult

If the robot arrives early at an interception point, it must wait. Ignoring waiting cost makes early arrival artificially attractive and distorts the optimization objective.

### Options considered

- Ignore waiting entirely  
- Treat waiting as zero cost  
- Explicitly model waiting cost  

### Chosen approach

Waiting is modeled as remaining in place and paying that cell’s traversal cost per timestep. The total cost of interception is: movement cost + waiting cost. This preserves the intended objective and ensures early arrival is only chosen when it is genuinely cost-effective.


## Challenge 3: Efficiency and Stability

### Why it was difficult

Large maps make repeated full replanning expensive. Additionally, naive replanning every timestep can create unstable or jittery behavior.

### Options considered

- Replan from scratch every timestep  
- Cache full search structures  
- Cache only the final interception plan  

### Chosen approach

After computing an interception path, I cache the sequence of robot moves and reuse it if still valid and aligned with the current timestep. If invalid, I discard it and replan. The search is also bounded by the remaining target horizon and skips stale states to maintain efficiency.

## Fallback Strategy

If no feasible interception exists within the remaining time window, the planner attempts to move toward the target’s current position using explored states. If that fails, it performs a safe greedy move that reduces distance while remaining valid. This guarantees the planner always outputs a legal action.

## Reflection

The most important shift in this design was reframing the problem from “chasing a target” to “intercepting under time and cost constraints.” Once I treated interception as a bounded multi-goal search with explicit waiting cost, the planner became both more principled and more predictable.