# ESP32 SLAM and Motion Planning Robot

An ESP32-based robot that performs real-time 2D SLAM (Simultaneous Localization and Mapping) to build a floor plan of an indoor environment, and autonomously navigates between a sequence of goal points using A* path planning on the resulting occupancy map.

## Overview

The robot uses a VL53L4CD time-of-flight distance sensor mounted on a servo motor to perform 180° range scans at each position. Successive scans are matched using a tangent-based scan matching algorithm (based on Lu & Milios, 1997) to estimate the robot's motion and build a global occupancy map. Motion estimates from scan matching are blended with dead-reckoning estimates derived from commanded motor movements. The robot uses A* search over a downscaled ("coarse") version of the occupancy map to plan a path toward a sequence of goal points representing edges of the stored map, cycling through goals as it reaches each one (or the nearest reachable point to it). The map is served live over WiFi as a webpage viewable from a browser on the local network.

## Hardware

- **Microcontroller**: ESP32-C6-DevKitC-1
- **Motor driver**: TB6612FNG dual H-bridge
- **Distance sensor**: VL53L4CD time-of-flight, I2C, ~18° sensor ray FOV, 1200mm ideal max range
- **Servo**: Standard PWM servo with 180° rotation range
- **Motors**: Two DC gear motors (differential drive) and one free-rolling caster wheel
- **Drive**: Differential drive, robot pivots in place about the midpoint of the rear wheel axle; the sensor is offset forward from this pivot point

## Features

- 180° range scan using servo-mounted ToF sensor, outlier rejection for distance discontinuities/spikes
- Scan-to-scan matching via a coarse-to-fine rotation search (multi-hypothesis sampling followed by golden section search refinement) and point-to-point ICP translation solving
- Hybrid rigid transformation approach: motor-timing-derived rotation and translation estimates are blended with scan-matching estimates, each with configurable weighting
- Sensor-offset-aware kinematics: since the sensor is offset from the pivot point, the expected sensor displacement during a rotation is computed from the pivot geometry (position taken the sensor's point of view travels along an arc centered around the pivot point during rotations)
- Point-normal-based correspondence weighting to reduce mismatches from noisy geometry and from the "parallel wall" degeneracy (see Design Priorities below)
- Occupancy-grid map served over WiFi at `192.168.4.1`. Connect to the `carlsjr` network and open a browser to view
- Display of map changes as well as prospective path is updated live and color coded.
- Path planning based on A* over a downscaled occupancy grid facilitates autonomously filling in the whole map by cycling through a list of goal points
- Manual driving option (turn left/right, drive forward) on the same webpage, useful for isolating and testing the SLAM algorithm independently of autonomous motion planning

## Algorithm

### SLAM (scan matching)

Each `SLAM_iteration` performs the following steps:

1. Collect a 180° range scan in polar form, discarding points that are spikes/discontinuities relative to their neighbors
2. Convert the scan to Cartesian coordinates in the robot's local frame
3. Compute a per-point surface normal for both the new scan and the previous scan, from local tangent lines
4. Run a coarse multi-hypothesis search over candidate rotations, centered on the dead-reckoned expected rotation (the projected rotation from the last motor command), to find an approximate best rotation
5. Refine the rotation with a golden section search in a window around the coarse result
6. Compare the ICP-refined rotation's matching cost against the matching cost of trusting the dead-reckoned rotation completely. If ICP does not produce a meaningfully better fit, fall back to the dead-reckoned rotation. This protects against ICP occasionally converging to a spurious local minimum on ambiguous or sparse geometry, and is important because the ICP-derived rotation value must be near perfect (see Problem Constraints)
7. Given the computed rotation value, compute correspondence pairs between the new and previous scan by interpolating in the previous scan at each new point's polar angle (offset by the rotation and estimated sensor displacement, described below)
8. Reject correspondence pair outliers based on point-normal similarity and distance
9. Solve for translation via point-to-point ICP (mean of correspondence points minus mean of rotated new points), weighted per-point (see below)
10. Blend the ICP translation estimate with the dead-reckoned translation estimate according to a configurable weight
11. Update the accumulated global position/rotation and transform the new scan into global space to update the occupancy map

**Point-to-point instead of point-to-plane ICP:** Point-to-plane ICP (used in the original Lu & Milios formulation) assumes the correspondence normal accurately represents the true local surface direction. Because the VL53L4CD's sensor ray has a non-trivial field of view (~18°), individual readings are smoothed samples of whatever surface geometry falls within that cone. This means surfaces that are not separated by an actual depth discontinuity are frequently read back as smooth curves rather than sharp corners, which breaks the flat-surface assumption point-to-plane ICP depends on. Point-to-point ICP has no such assumption and was found to be considerably more robust regarding to this smoothing effect in testing.

**Correspondence weighting:** The total matching distance used to compute the ICP translation weights each correspondence pair by:
- Incidence angle of the point's surface normal to the robot's current forward heading (weighted heavily) 
    - This suppresses points on surfaces running parallel to the direction of travel, which otherwise contribute little to no genuine motion signal and are highly prone to being mismatched (see the parallel wall section in Problem Constraints)
- Incidence angle of the point's surface normal to the sensor ray that produced it (weighted lightly) 
    - Points read nearly edge-on to the sensor are read less accurately than points read face-on, so they are valued less, but this effect is treated as a secondary correction relative to the parallel-wall correction above

The total matching distance used to score candidate rotations, however, weights each correspondence pair uniformly. This is done because due to sensor noise, while a small number of accurate point pairs are sufficient to find translation, finding rotation requires more points to give an accurate value, making it better not to try to avoid mismatched/misread points and just use all the points to get the best rotation match.

**Accounting for the sensor's offset from the pivot point:** Because the sensor is mounted forward of the robot's actual rotational pivot, a rotation is never a pure rotation from the sensor's frame of reference, as the sensor also sweeps through a corresponding arc and translates as a result. This is accounted for by computing an estimated sensor displacement from the pivot-to-sensor offset distance and the candidate/estimated rotation angle, and folding that displacement estimate into the correspondence search as an angular offset. Without this correction, correspondence matching (which otherwise assumes zero translation between scans) becomes systematically inaccurate for any rotation of significant magnitude, since it would implicitly assume the two scans were taken from the same physical point.

**Matching against the previous iteration only:** The SLAM algorithm finds an optimal rotation and translation value by matching points taken in the current iteration to points from the previous iteration only. Consequently, errors in the translation or rotation values fonud compound at an unmitigated rate because error in the previous iteration is carried over directly into this iteration. To address the compounding error, it was also considered to instead match the current iteration's points against:
- The union of the past several iterations 
   - The issue with this is that in the case that the transformation of points in the previous two iterations resulted in the points being in different locations (if there was significant matching error in at least one iteration), points in the new iteration will likely all be matched with one of the two previous iterations because this minimizes total matching distance among pairs, which means the presence of the points from the other iteration that was not matched with was altogether useless, therefore not improving the compounding error issue. In the case that there was no significant matching error in either of the previous two iterations, then there would be no improvement with matching this iteration's points to either of them. So, this approach would not systematically improve error accumulation.
- The union of the points created by interpolating between pairs in the previous two iterations unioned with the points in the previous two iterations that were not paired
    - The idea with this is that by interpolating between the pairs, we can make sure the points in both previous iterations are represented (as long as they were paired) so rigid transformation error is roughly cut in half, and we don't lose the information from the points that were not paired by including those too. While this improves on the issue of only one previous iteration being used to match (improves, but doesn't outright fix because the unpaired points would still have the same issue), the issue with this idea is that the points in the new iteration are now primarily being matched with points that didn't actually exist in either previous iteration, and this could result in greater error, specifically in corners (see Design Constraints section), and this is particularly bad because this is where most of the error is already seen in testing.

Ultimately, matching to strictly only the previous iteration's points is an imperfect option because it does not reduce error accumulation in any way, but it is the best choice regardless because it avoids the issues with alternatives that do reduce error accumulation.  

### Path planning (using A*)

1. The occupancy grid is downscaled into a coarser grid (each coarse cell aggregating a square block of standard cells) to make graph search computationally practical on the ESP32 board.
2. A* searches the coarse grid from the robot's current coarse cell toward the coarse cell containing the current goal, using 8-directional movement with diagonal (√2) and cardinal (1) edge costs.
3. Only coarse cells within a certain inflated range of either a previously observed obstacle or the origin, are considered traversable by the search. This keeps the robot from planning a path into unexplored, unverified space where it could lose its point of reference for scan matching (see Problem Constraints).
4. If the goal cell cannot be reached, A* instead returns a path to whichever explored cell came closest to the goal by euclidean distance during the search.
5. A goal is considered "reached" once the robot is within a set radius of it in coarse-cell units, rather than requiring the exact cell. This is because the purpose of reaching a goal is primarily to have that region of space within sensor range, rather than to explicitly occupy that target cell. 
6. The robot moves toward the second node in the resulting path, rather than the immediate first node after the robot's current cell, which smooths the effective path the robot follows and allows the coarse grid to be scaled finer than would otherwise be practical (more explanation in Design Rationale).
7. If the current goal is reached (or its nearest reachable substitute), the robot cycles selection to the next goal in the list and re-runs A*.

## Design Rationale

**A\* over sampling-based planners (RRT):** Sampling-based planning, namely RRT, was considered for its efficient average-case compute time, but its random sampling makes it poorly suited to a setting where the obstacle map is being updated after every single step. Since the robot cannot rotate without also translating (due to the sensor's offset from the pivot), and since the robot only ever follows the first or second node of a freshly recomputed plan, the direction of that leading node in an RRT is only weakly and inconsistently biased toward the goal from one replanning cycle to the next, as the randomness in tree structure means the robot's chosen heading could vary significantly between two nearly-identical planning cycles. Instead, A* on a coarse grid gives a stable and deterministic path each time it is recomputed, which matters far more than raw planning speed in a system that replans after nearly every physical step.

**A\* over potential fields:** Potential fields were considered for their natural adaptability to a continuously updating environment, but since the overall system's execution time is bounded much more tightly by the time to physically move and perform a SLAM scan than by the time spent path planning, potential fields' computational speed advantage does not outweigh the disadvantage of the less-efficient paths it generates. A* on a coarse grid, by contrast, produces more direct paths and is far more robust against getting stuck in local minima, which is an addition risk for potential fields in a environments that are not curated to avoid it. 

**A\* over hybrid (A\* + potential fields):** A hybrid long-range/local-planning approach was considered, but the added implementation complexity did not seem justified given that the coarse-grid approach, afforded by the role of path planning in this system (see below section), already makes pure A* comparably computationally efficient to a lighter-weight local planner like potential fields, while retaining A*'s guarantee against local minima that potential fields is missing.

**Downscaled "coarse" occupancy grid for planning:** Obstacle information is already stored in a fine occupancy grid from SLAM. However, planning directly on that fine grid would be computationally impractical on the ESP32 board and would produce more path nodes than are meaningful given that the robot only needs to be close enough to a goal position to see all the obstacles that can be seen from the actual coordinates. Downscaling the grid for planning allows the system to use more costly but complete graph search algorithms for path planning like A* that avoid drawbacks of cheaper methods like instability or susceptibility to traps in local minima.

**Moving toward the second path node instead of the first:** Following only the immediate next node in an 8-directional A* path produces a jagged, frequently-turning path. Skipping ahead to the second node smooths this out considerably, and this smoothing effect is what allows the coarse grid resolution to be made finer than would otherwise be practical, since without it, a finer grid would mean more frequent turns. Since the SLAM algorithm performs best when rotations are limited to around 15 degrees per iteration, most of the time spent executing a step prescribed by path planning will be used to align rotation, not translation, so it is important to keep rotations as minimal as possible.

**Replanning after every rotation step:** Because the sensor's position is offset from the pivot point, the robot cannot rotate without also translating. Since a fresh SLAM reading is required after every physical motion step regardless, and since a rotation changes the robot's position, the planner returns to the planning state after every rotation rather than assume that a previously computed path remains valid.

**Goal-reached radius instead of exact-cell matching:** This is done for the same reason that path planning can be done on a coarse occupancy grid: the purpose of a goal point is to bring the area surrounding the goal point into sensor range so it can be mapped, not to move the robot to the exact position. Adjusting the robot's position by a small amount can be cumbersome and adds unnecessary complexity to local path planning if it requires the robot to reverse direction, since the sensor's offset from the rotational pivot point causes the robot to translate significantly when it rotates. Thus, since the purpose of moving the robot to a position accomodates generous precision for the reason mentioned above, it is best to avoid time wasted on exact position refinement altogether and speed up execution time. 

**Collision checking with inflation on occupancy grid:** Collision checking is frequently considered the most expensive part of a motion planning system. However, the aforementioned priorities that allow us to use a downscaled occupancy grid for path planning also allow us to use a remarkably cheap solution to this part of the problem. Firstly, we can inflate the coarse grid lightly to mark off a safe estimate of points where the path planning will not choose to traverse. Since we do not care that the robot can position itself exactly to the limits of the open space it observes, if we simply ensure that the inflation is more generous than the furthest point on the car from the sensor, we can safely avoid having to evaluate exact geometric overlaps, or expensive Minkowski sums or raymarching. Invalid cell marking with inflation can be done once after each scan, and then can be used for each cell considered in path planning without having to be recomputed.

**Restricting A\* to cells near known obstacles:** Since the ToF sensor used in this system has a limited range of 1200mm in ideal conditions, it cannot always take distance measurements of every point in its line of sight, and for obstacles to be observed in a given scan, they must be within sufficient range of the robot. Because the SLAM algorithm matches against only the previous iteration's scan, if any scan is completely bereft of observed obstacles then this iteration will not be able to pair any points or perform ICP at all, leaving the rigid transformation entirely up to dead-reckoning. This will result in catastrophic error (see Problem Constaints for explanation), so there must be a guarantee that the robot never makes a scan that includes no points. This guarantee is implemented using another inflated (moreso than the obstacle coarse map) version of the coarse map against which the path planner checks before exploring a cell. As long as this map is inflated by a distance that undershoots the furthest distance away the sensor can measure, the path planner will never direct the car to leave sensing range of a known obstacle. The origin is also included as a point to be inflated on this map and marked as valid to explore. This is to allow some lenience in the initial placement of the robot's proximity to an obstacle.

## Problem Constraints/Priorities

**Localization must be close to perfect due to compounding errors:** Since each new scan is matched against the previous scan transformed by the previously computed translation and rotation, any error in either value is carried forward and compounds into every subsequent iteration. In typical SLAM systems, scan matching is a secondary correction layered on top of a primary odometer-based dead-reckoning system, and is mostly responsible for correcting slippage or drift rather than doing the majority of the localization work. This system has no wheel odometer, and as such, the only source of information available on which to base dead-reckoning is the estimate of distance/rotation derived from motor timing, which is less reliable than true wheel odometry. Nevertheless, dead-reckoning, even if just based on motor timing estimations, must also be utilized to meet the strict accuracy requirements demanded due to  compounding error, although it may not be trusted to the same extent that SLAM systems typically do.

**Rotation matching in particular must be extremely accurate:** Since rotation and translation are computed interdependently, and minute deviances in rotation (even 1-2 degrees) can drastically distort the geometry of the map, it is vital that rotation be as accurate as possible so that computed translation values are not thrown off due to mismatching. Dead-reckoning based on motor timing cannot reliably compensate for bad rotation values achieved through scan matching because it has no awareness of any physical drag with a directional bias that may occur. In testing, this sort of drag was often seen as a result of less-than-perfect weight balancing and trailing cables. Therefore, to compute the rotation component of the rigid transformation between scans, the value arrived at by scan matching is trusted almost entirely if not outright in favor of dead-reckoning, and must be highly accurate. 

**Translation matching does not need to be as precise:** Because the robot's wheels have rubber tires and slippage during straight-line motion is minimal in practice, it is possible for dead-reckoning based purely on motor-timing estimates to be reliably accurate. For this reason, and the frequently-seen inherent issue with ICP translation matching in cases with parallel walls (see section below), dead-reckoning can be used at a higher weight to compute the translation value. This means that the translation value found through ICP does not have to be as accurate as the ICP-derived value for rotation does.

**Parallel walls cause ICP-based translation to undershoot:** This issue is best explained by examining the worst case: the robot drives parallel to a wall that extends beyond sensor range in both directions, with no other reference surface visible. As the robot moves forward, new points entering sensor range in front of it are matched at the same rate that old points behind it leave sensor range, so the robot perceives an essentially identical scan regardless of how far it has actually traveled along the wall. In this situation, ICP alone has no way to recover true forward displacement, since the visible geometry is locally invariant to translation along its own length. Situations that are less extreme but involve the robot travelling parallel to a wall in this way will see the same issue to varying degrees. To increase resistance to this source of error, correspondence pairs in matching used to derive the translation value are weighed heavily by incidence angle of the point's surface normal to the robot's current forward heading (see Correspondence weighting) with the intuition that points on surfaces that the car is facing head-on are more trustworthy as they are less likely to exhibit this type of mismatch. However, the most effective safeguard against parallel wall translation error is simply to weight dead-reckoning more heavily for deriving translation, since it is unaffected by this issue and is generally reliable for the reasons described in the section above. 

**Corners cause skew in ICP-based rotation:** The SLAM algorithm computes a correspondence pair point to each point in the new iteration by interpolating between the two points whose polar angles flank the polar angle of the new point in the old space. If the two flanking points were part of the same flat surface, this procedure is able to accurately place a new point inbetween on them on the same surface. However, at depth discontinuities (one surface is physically behind the other surface) or corners, this approach places the corresponding point on a fictious surface inbetween the two surfaces the points are actually on. Because the fictitious point is always placed on the interior of the angle between the surfaces that flank it, the error is biased in a consistent direction and points that are mismatched to it will lead to skewed rotation matching rather than random noise that would even out. The closer the robot is to the corner, the more tightly-spaced the points it measures will be, meaning that more of the new points will be flanked by the innermost points on the corner surfaces and will be mismatched with points on a fictitious plane, thus causing the resulting skew to be more pronounced. Luckily, discarding points due to a depth discontinuity can be done by adding a check for the maximum distance a point can be apart from its pair, and discarding pairs with fictitious points at corners can be done by adding a check that normals of points in a pair are within a range of degrees of each other. However, it is impossible to use these checks to completely eliminate fictitious corner matching without also breaking the matching algorithm under normal conditions because, at a certain level of strictness, pairs that are in error become indisinguishable from valid pairs merely subject to noise. Since corner skew cannot be completely avoided, it is necessary to adequately limit the proximity the robot can obtain to a corner so that the effect is less pronounced and can be corrected for by the aforementioned checks. 

**The robot must always remain within sensing range of some reference obstacle:** If the robot were to lose all reference points at any point along the its path, matching would have to rely purely on dead-reckoning. This would be catastrophic because scan matching must be near perfect to negate compounding error and dead-reckoning cannot correct for a directional drag bias (see earlier section about rotation matching accuracy). To ensure this does not happen, the robot's path must maintain proximity to known obstacles within range of its sensor at all times. Because the sensor can only be rotated 180 degrees, it is possible that the robot could remain in sensor range of an obstacle but rotate away so no references are in view, but this should not happen unless the car were doubling back on a path it had already taken, which is unlikely because the goal positions are cycled through in clockwise order and the car never gets close enough to an obstacle that it would otherwise be forced to reverse direction.

## Building and Flashing

Requires ESP-IDF v5.x or later.

```powershell
# From esp-idf directory
.\export.ps1

# From project directory
idf.py set-target esp32c6
idf.py build
idf.py -p COM4 flash monitor # replace COM4 with usb port connected to esp32 board
```

## Configuration

Configuration parameters are stored in `config.h`:

| Define | Description |
|--------|-------------|
| `MAP_SIZE` | Occupancy grid dimensions (cells per axis) |
| `MAP_RATIO` | Side length in mm of one map cell |
| `COARSE_RATIO` | Number of cells per side aggregated into one coarse cell |
| `SENSOR_FREQ` | Range scan samples per sweep |
| `SENSOR_PERIOD` | Range scan sweep duration in ms |
| `SPIKE_THRESHOLD` | Distance in mm between adjacent scan points to treat as a surface discontinuity/spike |
| `POINT_NEIGHBORHOOD_SIZE` | Count of points on either side used to calculate normal of a point|
| `MAX_DISTANCE_PER_ITERATION` | Correspondence pair distance outlier threshold |
| `CORRESP_NORMAL_SIMILARITY` | Correspondence pair normal-direction similarity threshold in degrees |
| `MAXIMUM_UNCERTAINTY_INVERVAL` | Golden section search convergence threshold in degrees |
| `DEADRECKON_T_WEIGHT` | Weight of dead-reckoned translation (1) vs. ICP translation (0) |
| `DEADRECKON_ROT_WEIGHT` | Weight of dead-reckoned rotation (1) vs. ICP rotation (0) |
| `SENSOR_OFFSET_FROM_PIVOT` | Distance in mm between the sensor and the robot's rotational pivot point |
| `PLANNING_ROTATION_TOLERANCE` | Maximum rotation error under which robot with drive straight rather than keep adjusting rotation |
| `MAX_ROT_PER_STEP` | Maximum degrees the robot will rotate in a single motion step |
| `MAX_DIST_PER_STEP` | Maximum mm the robot will drive in a single motion step |
| `MOVE_SPEED` | Motor PWM speed (0–99) |
| `OPEN_INFLATION_CONST` | Chebyshev proximity to an occupied coarse cell for a coarse cell to be considered for exploration |
| `REACHED_DIST` | Euclidean coarse cell proximity to a goal position for it to be considered reached |

## Viewing the Map

1. Connect your device to the WiFi network `carlsjr` (password: `password123`)
2. Open a browser and go to `192.168.4.1`

The map is updated live to show the points seen in each scan (the first four iterations are color coded red, blue, pink, and green, afterwards the points will be black), the current A* path (purple) including the next node the robot will be directed towards (aqua), robot's position (orange), and current goal (orange). There are also manual driving controls (turn left, turn right, go forward) that function when the robot is in manual mode, for isolated testing of the SLAM algorithm.

## Known Limitations / Future Work

- Coarse-map obstacle inflation (safety margin around known obstacles, separate from the larger "stay near a reference" inflation used for planning reachability) is temporarily disabled pending a fix to a collision-checking bug on the newly restructured coarse map
- No wheel odometry; all dead-reckoning is derived from distance estimates using motor timing, which is less reliable/powerful than true encoder-based odometry

## References

- Lu, F. & Milios, E. (1997). Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans. *Journal of Intelligent and Robotic Systems*, 18, 249–275.

## Credits

- VL53L4CD ULD driver by [LooUQ](https://github.com/LooUQ/st_vl53l4cd)