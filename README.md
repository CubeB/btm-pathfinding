# btm-pathfinding

Better pathfinding for Byte Tactics ([ozgb/btm](https://github.com/ozgb/btm)), as an **optional mod**. The default engine stays bit-identical to 3.1c. What this mod changes:

- a long route is searched over several ticks instead of being cut off;
- units start moving along the straight line while the route is worked out;
- unreachable goals are answered at once, from connected regions of the map;
- a goal on an occupied cell is moved to the nearest cell a unit can stand on;
- waypoints have a wider tolerance, and a goal that drifts a little keeps its route;
- attackers in a group each get their own stand-off point;
- an unreachable target ends at the closest point instead of repathing forever;
- units parked on a factory pad are told to move.

## Provenance

This is a clean-room implementation, under MIT. Its behaviour comes from a written spec (`SPEC.md`) of the changes RWE-B4 (GPLv3) made to pathfinding. A separate reviewer checked the spec for leakage before anyone implemented it. The code was written only from the spec, by someone who never read RWE's source.

## Using it

The engine is abstract. The host implements three traits, which together
make up `World`:

- **`Terrain`**: static map data. `cells()` gives the size in cells,
  `sample(x, y)` gives the corner height samples ((cols+1) x (rows+1)),
  `sea_level()` gives sea level (water depth = sea level - height), and
  `lava(x, y)` marks lava-excluded cells. Passability per movement class comes
  from these values plus the class rules. It is computed once by
  `Pathfinder::new` (SPEC 1.7), so the terrain must not change afterwards.
- **`Obstacles`**: live blockers. `cell_blocked(x, y, searcher)` is true for a
  building cell that is not an open yard cell, a blocking feature, or a cell
  occupied by any mobile unit other than `searcher`. Buildings being added and
  removed only change what this returns. Regions are terrain-only (SPEC 4.2),
  so the pathfinder does not need to be told about building changes. Searches
  read this live, slice by slice.
- **`Units`**: `unit(id)` returns a `UnitView`: position, `Mobility` (a
  registered `ClassId`, or `Inline(footprint)` for an unnamed class where only
  obstacles block), whether it is still moving, and the destination of its
  outstanding request (`None` turns that check off).

Movement classes (`MoveClass`: footprint, depth bounds, land and water slope
limits, bad slopes, floater, hover) are given to `Pathfinder::new` as a `Vec`.
A `ClassId` is an index into that `Vec`.

Each tick:

```rust
let out = pathfinder.tick(&world);          // spends at most budget_per_tick
for d in &out.deliveries { /* d.route: waypoints, unreachable, goal_relaxed */ }
for e in &out.events     { /* Dropped / Abandoned / Failed: ask again later */ }
pathfinder.request(unit, Goal::Point(p))?; // or Goal::Rect(cells)
```

`Nav` is the per-unit side (SPEC 3, 6, 8, 10), kept as plain state:
`set_goal` installs the provisional straight route or keeps a drifting one,
`poll_request` applies the 60-tick rate limit and tells you when to call
`request`, `on_delivery` and `on_request_lost` take the service output, and
`follow` retires waypoints (16 world units, or 8 for the last) and returns
the point to steer at. `order_arrived`, `climb_factor` and
`underwater_factor` cover arrival and speed. `standoff::StandoffCache` gives
attackers their stand-off points. `pad::sweep`, `pad::PadRetry` and
`pad::site_decision` cover factory pads and unreachable build sites.

`grid::GridWorld` (heights, buildings, features, unit occupancy) and
`sim::Sim` (a minimal mover) are a reference host for tests and for
`cargo run --release --example bench`.

All maths that can affect a route is integer: positions are fixed point
(`UNIT` = 256 steps per world unit, one cell = 16 world units) and step lengths
are 1.00000 and 1.41421 scaled by 100000. Heap ties are broken by estimate and
then by the order vertices were offered, and no hash-map iteration order is
used anywhere.

### Host choices (configurable)

| Choice | Where | Default |
|---|---|---|
| Region-answer charge (SPEC 2.2) | `Config::precheck_charge` | `max(1, first-pass steps)`. Any part of the charge beyond the budget left in the tick is carried as debt into the next ticks, so a tick never charges more than its budget |
| Queue capacity | `Config::max_queue` | 65536 (`Error::QueueFull` beyond it) |
| Feature switches for measurement | `Config::{first_pass, relax_blocked_goal, region_precheck}` | all on |
| Climb look-ahead and floor (SPEC 1.4) | `NavConfig::{climb_lookahead, climb_floor}` | 16 world units, 0.25 |
| Radii, drift, cooldown, refresh age | `NavConfig` | SPEC 12 values |
| What a lone unreachable order does at the closest point (SPEC 8) | host | `Sim` completes it |
