# Pathfinding mod: behavioural specification

Audience: the engineer implementing a pathfinding mod for Byte Tactics (MIT).
This document describes observable behaviour and the numbers that define it.
It deliberately gives no code structure. You are free to organise your
implementation however suits the host game, provided the behaviour and
constants below hold.

"The baseline" means the way the host game's units move and route without this
mod, judged only by what a player sees.

Throughout: the simulation runs at **30 ticks per second**; a map **cell is
16 world units** square; "octile" distances are counted in cells.

---

## 0. Vocabulary

- **Cell grid.** The height grid of the map, one cell = 16 x 16 world units.
- **Footprint.** A unit's size in cells (width x depth), taken from its
  movement class. A unit's position is the centre of its footprint.
- **Vertex.** A search position: the top-left cell of a footprint-sized block.
  A unit at world position `p` occupies the block whose top-left cell is the
  cell nearest to `p - (footprint * 8)` (half a footprint, in world units).
- **Movement class.** Per-unit-type terrain rules: footprint, minimum and
  maximum water depth, maximum slope on land, maximum slope under water, a
  "bad slope" on land and a "bad slope" under water (see 1.4), and whether the
  unit floats.
- **Standable.** A vertex is standable for a unit if (a) its whole footprint
  fits the movement class's terrain rules there and (b) no cell of the
  footprint is occupied by an obstacle (1.6).
- **Route.** A list of world-space waypoints a unit walks.
- **Provisional route.** The two-point straight route a unit walks while its
  real route is being computed (section 3).

---

## 1. The search

### 1.1 Grid and moves

- A search is an A* over vertices on the cell grid, **eight-connected**.
- The search starts at the unit's current vertex. The start vertex is **not**
  tested for standability (a unit pressed against others must still be able to
  search).
- A vertex that is off the map is never standable.
- **Corner squeezing is forbidden.** A diagonal step is allowed only if both
  orthogonal vertices flanking it (the two that share the step's x-move and
  the step's y-move with the current vertex) are also standable. The same
  rule must be used when labelling regions (section 4).
- **Determinism requirement.** The same world, goal and start must produce the
  same route every time, on every peer (the host may be lockstep). To get this,
  a vertex is expanded at most once; a vertex already waiting is replaced only
  by a strictly cheaper offer; and arrival at the goal is judged when a vertex
  is chosen for expansion, not when it is first offered. Any implementation
  that yields identical routes is acceptable.

### 1.2 Cost

The cost of a route is a pair: a length and a turn count.

- **Length.** Each step costs 1 (orthogonal) or sqrt(2) = 1.41421 (diagonal),
  multiplied by a terrain factor of the cell stepped *into*:
  - factor **1** for ordinary ground;
  - **+1** if that vertex is *rough* (1.4);
  - **+1** if the unit is a *wader* (below) and that vertex is *under water*
    (1.5).
  The two surcharges add, so a step is worth 1x, 2x or 3x its base length.
- **Turn count.** Each step adds the size of the heading change from the
  previous step, measured in 45-degree units around the eight compass
  directions, 0 to 4 (straight on = 0, 90-degree turn = 2, reversal = 4).
  The first step adds 0.
- **Comparison.** Costs compare by length first (as a real number), then by
  turn count. So turns are purely a tie-breaker between routes of equal
  length: among equal-length routes the smoother one wins. Beyond that no
  tie-break is mandated.
- A **wader** is a unit that is not a floater and whose movement class allows
  a minimum water depth of 0 (i.e. a land unit that may enter shallow water).
  Floaters (ships) and units that require water are never surcharged for
  water. Hover units are not surcharged for slope (see 1.4).

### 1.3 Heuristic

- Estimated remaining cost = octile distance to the goal, in cells: with
  dx, dy the absolute coordinate differences, `|dx - dy|` orthogonal steps
  plus `min(dx, dy)` diagonal steps (length = orthogonal + 1.41421 x
  diagonal). Ties are broken by an estimated turn count of **1** if the
  estimate contains both orthogonal and diagonal steps, otherwise **0**.
- The heuristic is admissible for ordinary ground (it ignores surcharges).
  When the goal has been given slack (2.x, 5, 6) it still aims at the exact
  goal, so it can slightly overestimate to the nearest accepted vertex; this
  is accepted.
- Priority of a waiting vertex = cost so far + estimate, compared as in 1.2.

### 1.4 Slope handling

- **Cell slope** = highest minus lowest of the four corner heights of the cell
  (2x2 block of height samples at that cell), in the map's height units.
- **Impassable.** A footprint is impassable terrain if any cell in it has a
  slope above the class's maximum (use the land limit for cells above water,
  the water limit for cells below). This is baked into the class's terrain
  grid (1.7).
- **Rough** (costs double, 1.2). A vertex is rough if any cell in its
  footprint has slope above the class's *bad slope* limit. Use the land bad
  slope for a cell whose lowest corner is at or above sea level, the water bad
  slope otherwise. Bad slope defaults to **half of maximum slope** (integer
  division), never above maximum slope; classes may set it explicitly, and
  hover classes set it equal to maximum slope so they pay nothing for ground a
  tank finds rough. A vertex whose footprint plus one cell of margin would
  fall off the height grid is not rough.
- **Climb speed penalty (movement, not search).** A moving ground unit's top
  speed is scaled by a slope factor each tick. Approximately: compare the
  terrain height at the unit's position with that at a point about 16 world
  units ahead along its facing (a tunable look-ahead); if the ahead point is
  higher, with rise = height difference, factor is about
  `1 - (rise / maxSlope) / 2`, never below a tunable floor of **0.25**. So top
  speed is roughly halved at rise = maxSlope and floored at a quarter. Level or downhill = factor 1. A
  class with maximum slope 0 or >= 255 is never slowed. Off-map ahead = factor 1.
  Separately, a ground unit whose origin is below sea level has top speed
  halved unless it is a floater or hover craft.
- The effect to preserve: because rough ground costs double, units take a
  flat detour round a patch of steep-but-passable ground whenever the detour
  is cheaper than crossing the patch at double cost.

### 1.5 Water

- A vertex is *under water* if any height sample in the (footprint + 1) x
  (footprint + 1) block starting at the vertex lies below sea level.
- Water depth bounds (min/max) and slope limits are part of the terrain grid
  (1.7), so a land unit simply cannot enter water deeper than its class
  allows. The wading surcharge applies only to waders (1.2) in water they
  *can* enter.
- Effect to preserve: an amphibious/wading unit routes over a dry detour
  rather than wade when the dry route is no more than about twice as long as
  the wet one over the same stretch (test case: straight route 8 steps with 4
  wet; dry detour about 9.7 step-equivalents; dry wins). A floating unit goes
  straight through water with no detour.

### 1.6 Buildings, features, units

- A footprint is blocked if any of its cells is occupied by: a **building**
  cell that is not passable (an open yard cell, e.g. a factory's open
  doorway, is passable); a **blocking feature** (e.g. a rock or wall, not
  every feature blocks); a **mobile unit other than the searcher**; or a
  lava-excluded cell.
- **Cells the searching unit itself currently occupies are treated as
  passable for that search.** A unit standing inside something that blocks
  (a wreck over its own cells, a feature it was told to reclaim) would
  otherwise have no successors and be told it had arrived. It must be able to
  step out.
- All other occupants block, moving or not. The provisional route of
  section 3 and the 60-tick refresh of section 10 are what keep traffic
  flowing.
- Obstacle state is read live during a search; searches are sliced across
  ticks (section 2), so the world can change under a search. That is
  accepted; no re-validation is done until the route is delivered and
  followed.
- If no precomputed terrain grid exists for a unit's movement class (a unit
  with an inline, unnamed class), terrain is not consulted at all and only
  obstacles block.

### 1.7 Terrain grid per movement class

- For each movement class, a precomputed standability lookup covering every
  position where the footprint fits on the map says whether a footprint with
  its top-left cell at (x, y) is terrain-standable: no lava exclusion in the
  footprint, every cell slope within the class's maximum (land or water limit
  as in 1.4, chosen by whether the (footprint+1)-sized block touches water),
  and every cell's water depth (sea level minus height, floored at 0) between
  the class's minimum and maximum depth inclusive.
- Built once when the map loads (the terrain is static) and never changed.

### 1.8 Goals

Two kinds of destination exist.

**Point goal.** The goal vertex is the footprint block that contains the
destination point. The search stops on the first vertex taken from the waiting
list whose octile score to the goal is within the *slack* (below), or equal to
the goal. Octile score = 10 x orthogonal steps + 14 x diagonal steps. Slack
starts at 0 (exact goal only) and is raised by the relaxations in sections 5
and 6 and by the first pass (2.3); when several raise it, the largest wins.

**Rectangle goal** (used for "get next to this building/area", e.g. build
sites and the pad-clearing move in section 9). Accept any vertex whose
footprint touches the rectangle's outside without overlapping it, including
diagonal touching at corners. Estimate = octile distance to the nearest such
touching vertex (zero on it; inside the rectangle, the distance to the nearest
edge). No slack, no first pass, no region precheck for rectangle goals.

### 1.9 Result and route

When the search ends:

- **Found.** The path from start to the goal-accepted vertex.
- **Exhausted** (waiting list empty, goal never accepted). The goal is
  unreachable by any route. The result is the path to the *closest* vertex
  reached: the one with the smallest estimate (compared as in 1.2). For
  determinism, among equally close vertices the one reached first wins. The
  route is flagged **unreachable** (see section 8).

A route built from a path must satisfy:

- It contains only the corners of the path (runs of same-direction steps are
  collapsed) plus the start vertex as the first waypoint.
- Waypoints are world-space **centres of the footprint block** at ground
  height.
- Only for a point goal found exactly (stopped on the goal vertex itself, not
  on a slack vertex, and goal not relaxed by section 5) is the last waypoint
  the exact destination point rather than a cell centre.
- A route always has at least **two** waypoints. A one-vertex path (already
  there, or as close as it gets) yields the unit's current position followed by
  either the exact destination (found, goal not relaxed, point goal) or the
  unit's current position again (unreachable, relaxed, or rectangle goal).
- When the route is flagged unreachable, its last waypoint is remembered on the
  unit as its **reachable destination** (section 8).

---

## 2. Sliced search across ticks

### 2.1 Budget

- All path searches share a **budget of 4000 node expansions per tick**. An
  expansion is one vertex taken from the waiting list (including the one that
  turns out to be the goal).
- There is **no per-search cap** and no truncation. A search ends only on
  finding the goal or emptying the waiting list. (A design that caps a search
  and returns a partial route for the unit to walk and re-request is not
  wanted.)
- A search that exhausts the tick's remaining budget is **suspended in
  place** (nothing is recomputed) and resumed first thing next tick before any
  other request is started. Cost of a worst-case search over a 512 x 512 grid
  is up to 262,144 expansions = about 66 ticks.
- **One search runs at a time.** Requests wait in a first-in-first-out queue;
  the head request's search is the active one across ticks. Each tick: while
  budget remains, continue the active search, or start the head of the queue
  with the leftover budget, and pop a request when its search finishes.
- Before resuming a suspended search, check the unit still exists, is still
  moving, and still has the same destination; otherwise throw the search away
  and move on (counted as abandoned).
- Requests whose unit is gone or no longer moving are dropped when reached.
- A unit that asks again while it is already queued **behind** the head moves
  to the **back** of the queue. A unit that asks again while at the head keeps
  the head: if its destination is unchanged the active search carries on; if
  it changed, the active search is thrown away and a fresh one begins for the
  same unit at the head with the new destination.

### 2.2 What is and is not charged to the budget

- Expansions are charged.
- The cheap first pass (2.3) and the blocked-goal ring scan (section 5) are
  **not** charged when an A* then runs.
- An answer produced without any A* (region precheck, section 4) is charged
  `max(1, first-pass steps)` instead. This is a tunable throttle; the
  observable requirement is that a queue full of impossible goals is not
  answered wholesale in a single tick however long the cheap pass took, while
  cheap answers stay cheap. Expected effect: when walks are short (about a
  dozen steps) over a hundred such requests still finish in one tick; when they
  are long (about 500 steps) only a handful finish per tick.

### 2.3 First pass (cheap pre-search)

Before each point-goal search starts, run a cheap bounded pass that estimates
how close the unit can get to the goal, **not** a full search:

- It is a greedy walk toward the goal that follows obstacle boundaries when
  blocked, using the live standability test (including units). Any equivalent
  cheap method is acceptable (e.g. a bounded wall-following walk); the
  requirement is the contract below.
- It is bounded to **2000 steps**; on hitting the bound it reports the closest
  cell reached so far.
- It reports (a) the closest cell reached by octile score to the goal,
  (b) whether it reached the goal, (c) steps taken.
- If it did **not** reach the goal, and its closest cell is strictly nearer
  (by octile score) than the unit's own start and not zero, **raise the
  search's slack to that score**. The search now accepts any vertex at least
  as near as the walk got. An unreachable target therefore ends at the nearest
  spot instead of exhausting the whole reachable map.
- The pass **never refuses a search**. It only widens what counts as arrival.
  A reachable goal that the pass reaches is searched exactly (slack unchanged).

---

## 3. Moving on a straight line while the route computes

- On a new move order (or a new goal the unit cannot keep its old route for,
  section 10), the unit **immediately** receives a two-waypoint provisional
  route: its current position, then the goal (for a rectangle goal, the
  rectangle's centre at ground height).
- It starts walking that line on the **same tick** the order arrives, ignoring
  obstacles (it will slide along walls until the real route lands).
- The provisional route is marked so the system knows it is not a real route.
  The unit keeps asking for a real one (section 10) and keeps walking the line
  until the real route is delivered, which replaces it.
- When the real route lands, it replaces the unit's route wholesale, clears
  the provisional mark, clears the request-in-flight mark, and sets or clears
  the unit's remembered reachable destination (section 8).

Effect to preserve: a unit ordered to move is moving on the tick of the order,
even with the pathfinding budget at zero; a group of 400 does not stand still
while the queue drains.

---

## 4. Connected regions for instant "unreachable" answers

### 4.1 What is labelled

- Per movement class (that has a terrain grid, 1.7), label the **terrain
  grid** into connected regions: cell id 0 = not standable; ids 1, 2, ... for
  each connected set of standable vertices.
- Connectivity is **eight-way with the same no-squeeze rule as the search**:
  a diagonal link exists only when the two orthogonal vertices flanking it
  are also standable. (Four-way labelling would wrongly separate vertices that
  a diagonal step joins; a precheck that says "unreachable" wrongly is worse
  than none, so the labelling must be at least as connected as the search.)
- Labelling is by flood fill, scanning rows top to bottom and cells left to
  right, assigning the next id to each unlabelled standable vertex.
- Computed lazily the first time a class is asked about and then kept.

### 4.2 Not kept up to date for buildings or units

**Regions are terrain only.** They are computed from the static terrain
grid. They are **not** updated when buildings are placed or destroyed, and
they ignore units, features, and gates. They never change after first use
(terrain does not change during a game). A goal blocked by buildings or units
rather than terrain therefore still goes through the normal search (and the
blocked-goal relaxation of section 5). No incremental update, merge or split
mechanism is required.

Design constraint for the Byte Tactics implementation: the region precheck
must never claim "unreachable" when a route exists. So if you want regions to
reflect buildings, you must keep them exactly consistent (removals can merge
regions, additions can split them, and a stale label gives wrong answers). The
simple, safe choice is to label terrain only.

### 4.3 Use

At the start of a point-goal search:

- if the first pass ran, did **not** reach the goal, and was unable to improve
  on the start (slack not raised by it, 2.3), and
- the class has a region grid, and
- the start vertex has a nonzero region (a start off the grid or on an
  unlabelled vertex means "no conclusion"), and
- the goal vertex is off the grid, or has region 0, or has a different region
  from the start,

then **finish at once, without any A***: the route is the start vertex, then
the first pass's closest cell (if different from the start), flagged
unreachable (section 8). Charge the budget per 2.2.

If the first pass could improve on the start, or the regions agree, run the
normal search.

### 4.4 Reference measurement

(Quoted from a reference implementation's benchmark harness, not a game.)

- Scenario "pressed against water": 100 units start hard against a channel
  they cannot cross and all are ordered across it, 600 ticks. Expansions
  **2,396,000 -> 0**; units that arrived (at their closest reachable point)
  **77 -> 100**.
- Goals blocked by buildings or units rather than terrain gain nothing from
  regions, because regions are terrain only; that remains the expensive case.

---

## 5. Occupied target cells relaxed to the nearest standable cell

**Why.** An attack order paths to the cell the target stands on, and a move
order onto a building or into a crowd has the same shape. On a large map most
path requests (about nine in ten, in a reference measurement) aim at an
occupied cell, so the goal can never be reached, and A* would prove that by
closing every reachable vertex. Those searches are a small share of searches
but most of all expansions.

**Behaviour.** For a point goal, before the search starts: if the goal vertex
is not standable (using the live obstacle test, i.e. including units), look
outward from the goal in square rings of radius 1, 2, ... **8** cells
(Chebyshev rings). At the first ring that contains any standable vertex, take
the **smallest octile score to the goal among that ring's standable
vertices** and set the search's slack to it. The search then accepts any
vertex at least that near the goal, and returns the genuine shortest route to
the first such vertex.

- If no vertex within 8 cells is standable, leave the goal alone (the search
  runs as normal and may end as unreachable).
- Radius 8 is about a 17-cell-wide crowd: enough for a ring of units around an
  attack target while costing only a few hundred footprint tests.
- The route is marked **goal relaxed**. When a route is built from a relaxed
  search, **do not** snap the last waypoint to the exact destination, and for
  a one-vertex path stand still at the unit's own position. Otherwise the unit
  would be sent walking into whatever stands on the destination.
- This is a relaxation, not a work limit: a route that is genuinely long
  (around a wall) is untouched. Two rejected alternatives: a per-search work
  limit gives wrong answers for long routes; a "no progress" limit failed on
  a 64 x 64 wall.
- Reference measurement (crowded case: 200 units all sent at one blocked
  point, 900 ticks): expansions per search **13,777 -> 3,520**. Cases where the
  goal is not blocked are identical with and without the relaxation.

---

## 6. Waypoint tolerance and the drifting goal

### 6.1 Waypoint tolerance: 16, not 5

- A unit retires the waypoint it is heading for when its horizontal distance
  to it is **at most 16 world units**; the **final** waypoint retires at
  **8 world units**.
- At most **one** waypoint is retired per tick, however fast the unit moves.
- When the final waypoint retires the unit brakes to a stop; whether the order
  is finished is decided by the order's own arrival test (for a plain move to
  a point, the unit standing on the goal's cell; for a rectangle goal, within
  8 world units of the nearest point of the rectangle, enlarged by half the
  unit's footprint).
- Why not the baseline's 5: a waypoint is a cell centre, and a unit two cells
  across cannot always get within 5 of a waypoint that another unit is
  standing on; it circles until a collision repath rescues it.

### 6.2 Drifting goals keep their route (64 world units)

When a unit has a **real** (non-provisional) route and its goal changes:

- if the new resolved destination and the one the route was built for are both
  **points** and are within **64 world units** (straight-line, horizontal), it
  is the same goal drifting (typically an attacker's stand-off point following
  a moving target): **keep walking the current route** and let the rate-limited
  refresh (section 10) fetch a fresher one;
- otherwise (further than 64, or a rectangle goal, or a mix, or the current
  route is only a provisional one), it is a new order: install the provisional
  straight route at once (section 3).
- 64 was chosen to sit outside the 8-unit drift of a stand-off point
  (section 7) and still be small enough to mean "the same place".

---

## 7. Group attacks spread to their own stand-off points

**Problem.** Sending every attacker at the target's centre makes every search
aim at one cell, so a group queues one at a time on the request queue; the
first few fire and the rest walk into the back of whoever arrived first.

**Behaviour.** A ground unit chasing a **unit** target that is out of weapon
range (horizontal distance greater than the weapon's maximum range) walks to a
**stand-off point of its own**, not the target's centre:

- direction = horizontal vector from the target to the attacker's own current
  position, normalised (if they coincide, use +x);
- stand-off distance = the larger of
  - *footprint stand-off*: the larger half-extent of the target's footprint (in
    world units) plus **8**, and
  - *range stand-off*: the weapon's maximum range minus **8** (0 if the range
    is 8 or less);
- stand-off point = target position + direction x stand-off distance, at the
  target's height. So a short-range weapon against a huge target stops outside
  the target rather than inside it, even if that is out of weapon range.
- Attackers coming from different bearings get different points, so a group
  fans out around the target.
- An attack on a **ground point** (no unit behind it) goes straight at the
  point as before.
- Once within range, the unit stops chasing and sets its weapon targets.

**Stability.** The stand-off point is cached per attacker per target and reused
as long as the distance from the cached point to the target's *current*
position is strictly between (stand-off distance - 8) and (stand-off distance
+ 8) (lower bound floored at 0). The tolerance is a **band around the stand-off
distance**, not a comparison of squared distances (which would shrink the
effective tolerance to about a sixth of a unit and discard the cache nearly
every tick). Only when the target has really moved is a new bearing computed
from the attacker's current position. A stationary target therefore gives one
approach point and one path request per attacker for the whole engagement.
Each attacker's cache is separate from the "where is the target" cache, so
other orders on the same unit are never given an approach point by mistake.

The point is a function of positions and definitions only, so every peer
computes the same value.

Interaction: if a stand-off point is itself occupied, section 5 relaxes it; if
the target moves by under 64 world units, section 6.2 keeps the route.

---

## 8. Unreachable targets end at the closest point

- If a point or rectangle goal has no route, the unit receives a route to the
  closest standable vertex found (1.9 for exhausted searches; 4.3 for region
  precheck) and is flagged with that **reachable destination** (the route's
  last waypoint).
- Arrival tests for that order treat the reachable destination as the goal, so
  the unit stops there rather than shuffling against the obstacle forever. A
  unit that stops short of a wall stops *next to* it, not inside or on it.
- A builder ordered to a construction site it cannot reach (route flagged
  unreachable) gives the order up with a "can't reach the site" message
  rather than building across the obstacle. This differs from a site that is
  merely occupied, which is waited out.
- What a lone unreachable move order does at the closest point (complete,
  retry later, advance a queue) is a host policy question and is not part of
  this spec. Keep the order alive or complete it as the baseline does.

---

## 9. Factory pads clear

**Problem.** In the baseline a unit left parked on a factory's spawn point
simply blocks production forever (a shipyard with a friendly hull on its pad
stops producing).

**Behaviour (a deliberate departure from the baseline).**

- **Trigger A.** Whenever a factory (or any builder placing a structure)
  finds its spawn/placement footprint blocked by a mobile unit, it sweeps that
  footprint: every mobile unit occupying any cell of it that currently has
  **no orders at all** is given a "clear the pad" order carrying the footprint
  rectangle. Units that already have orders are left alone.
- **Trigger B.** The same sweep runs, over the factory's own footprint,
  whenever the factory's own behaviour data asks for its pad to be cleared
  (the shipped factories do this as they open up to release a unit), so a
  parked unit is asked to leave before production is blocked. It is driven by
  that request, not by production starting as such.
- A factory whose pad is blocked **retries every 15 ticks** with no cap and no
  failure message.
- **The clear-the-pad order** is a rectangle-goal move (1.8): the unit
  must reach a point where it is clear of the pad rectangle expanded by
  `(3 x unit footprint) - 4` cells on each side in each axis (clamp at 0 if
  negative). Arrival uses the rectangle-goal arrival test of 6.1. The order
  is removed on arrival.

---

## 10. Repath timing and limits

- **Request rate limit.** A unit may have a route request queued **at most
  once every 60 ticks** (2 seconds). A request is issued when the unit wants a
  route, has none in flight, and either has never asked or its last request
  was at least 60 ticks ago. The timestamp is recorded at issue time and
  **survives a new order** (the new order's provisional route starts at once
  but its real request still waits for the 60 ticks to pass).
- **A unit still on a provisional route always wants a real one**, and gets
  one the moment the limit allows. The limit is a delay, not a lost request.
- **Do not interrupt a search in flight.** If the goal changes while the
  unit's request is in the queue or being searched, the new goal is noted but
  the search is let land; the unit re-enters the logic on the tick it
  completes. Requesting again with the same destination keeps queue position
  (2.1).
- **Goal drift** (6.2): keep the route, mark "wants a route", and let the
  rate limit gate the refresh.
- **Moving target resolution.** If the resolved destination of an unchanged
  goal changes (target walked), update the stored destination only when no
  request is in flight; mark "wants a route".
- **Other triggers** (all gated by the same 60-tick limit):
  - the unit has walked to the end of its route (and its order is not yet
    satisfied) and the route is at least **30 ticks** old, or it has no route;
  - the unit is in collision (blocked by another) with no request in flight
    and its route is at least **30 ticks** old, or it has no route.
- **When delivered**, a route replaces everything (section 3). A route
  request is dropped if the unit has died or is no longer moving.
- **Budget:** 4000 expansions per tick (section 2).
- **First-pass bound:** 2000 steps (2.3). **Blocked-goal ring radius:** 8 (section 5).
- Reference measurement: with the limit and drift tolerance, searches per tick
  fell by a third to a half, and expansions thrown away fell by **86-95%**.
  Scenarios that never repath were unchanged.
- Raising the budget was found not worthwhile: a modest shorter queue for a
  much longer worst-case tick.

---

## 11. What deliberately changes from the baseline, and what is kept

### Kept (matches the baseline as a player sees it)

- A unit ordered to move starts moving immediately in a straight line, and
  switches to the real route when it arrives.
- Long routes are never truncated or abandoned half way: they complete over
  several ticks.
- A cheap look-ahead relaxes what counts as arrival when the goal is out of
  reach, so impossible goals end at the nearest spot.
- Unreachable goals end at the closest reachable point rather than re-pathing
  forever.
- Route re-requests are rate-limited per unit to once every 60 ticks.
- Eight-way movement, no squeezing diagonally between touching obstacle
  corners, slope-aware routing and speed, wading penalties.

### Changed (deliberate departures)

- **Waypoints retire at 16 world units (8 for the last)** rather than 5, to stop
  crowds circling cell centres that others occupy (6.1).
- **Occupied goals are relaxed up front** to the nearest standable cell within 8
  cells, where the baseline keeps searching (section 5).
- **Drifting goals within 64 world units keep the route** (6.2).
- **Region precheck** answers cross-region goals without searching
  (section 4). Terrain only.
- **Per-attacker stand-off points** so a group fans out (section 7).
- **Pad clearing** units parked on a factory pad are told to move off
  (section 9).
- **Search budget is 4000 expansions per tick**, shared in a first-in-first-out
  queue, one search at a time; the cheap first pass is bounded at 2000 steps
  and the start vertex is not tested for standability; the first pass relaxes
  but never refuses a search.
- **Own cells are passable for the searching unit**, so a unit trapped inside a
  blocker can step out.
- **Climb penalty**: top speed falls to half at the slope limit, floor a quarter.

---

## 12. Constants (single table)

| Constant | Value |
|---|---|
| Tick rate | 30 / s |
| Cell size | 16 world units |
| Expansion budget | 4000 per tick, all searches combined |
| Per-search cap | none |
| First-pass step limit | 2000 |
| Blocked-goal look-out radius | 8 cells (square rings) |
| Octile score | 10 per orthogonal step, 14 per diagonal step |
| Search cost | 1 / 1.41421 per step; x2 rough; +1 more if wading in water (x3 both) |
| Turn tie-break | 45-degree units, 0-4 per step |
| Heuristic turn estimate | 1 if both orthogonal and diagonal steps remain, else 0 |
| Bad slope default | floor(max slope / 2) (hover: = max slope) |
| Climb speed factor | 1 - rise/maxSlope/2, floor 0.25 |
| Underwater speed | halved (not floaters/hover) |
| Waypoint retire radius | 16 (final: 8) world units |
| Retire rate | at most 1 per tick |
| Goal drift tolerance | 64 world units |
| Request cooldown | 60 ticks per unit |
| Route age before refresh on finish/collision | 30 ticks |
| Stand-off footprint margin | 8 world units |
| Stand-off range margin | 8 world units |
| Stand-off cache tolerance band | +/- 8 world units |
| Pad retry | every 15 ticks |
| Pad-clear expansion | 3 x footprint - 4 cells each side |
| Rectangle arrival radius | 8 world units |
| Terrain regions | terrain only, 8-way with no-squeeze rule, lazy |

---

## 13. Acceptance behaviours

1. Two obstacles touching diagonally never get squeezed through: a route
   around them has more than 2 vertices and contains no diagonal step between
   them.
2. A corrugated passable-but-steep band of slope 6 (limit 10, bad slope 5)
   across the map is avoided by a flat detour; a class whose bad slope equals
   its max slope crosses it freely.
3. A wading unit detours over dry land when the wet route is not much shorter;
   a floater goes straight through.
4. A goal blocked by one obstacle on an open 64 x 64 map: without slack the
   search is exhausted and closes more than 1000 vertices; with slack equal to
   one orthogonal step (score 10) it finishes complete beside the obstacle in
   under 1000 closed vertices and the last vertex is not the goal.
5. A goal in a sealed box ends at the closest reachable vertex, and which one
   is the earliest expanded among the equally close.
6. A search sliced into 1-expansion slices returns exactly the route of an
   uninterrupted search; a walled-off goal on a 64 x 64 map is declared
   unreachable after more than 4000 expansions (no 1000 cap).
7. A unit that asks again while its search is suspended at the queue head keeps
   its place.
8. With the budget set to 0 a unit ordered to move still walks toward the
   goal and still has an outstanding request.
9. A reachable goal is reached, not approximated: the first pass never leaves
   a unit short of a reachable destination.
10. A goal across a channel too deep for the unit is answered at once, with no
    expansions, and the unit ends at the bank; the budget is charged for the
    first-pass steps taken.
11. A goal that moves 32 world units while a real route is held keeps the
    route and issues no new request until 60 ticks after the last request; a
    goal 800 units away takes the provisional route at once.
12. A unit standing inside a blocking feature can walk out of it.
13. A builder ordered onto an unreachable site gives up rather than building
    across the obstacle.
14. Climbing slows a unit toward half speed at the slope limit and never below a
    quarter; flat, downhill or no-limit classes are unaffected.

---

## 14. Diagnostics worth keeping (optional, never feed back into behaviour)

Counters per window: searches begun; expansions; searches suspended across a
tick; searches abandoned; searches ended exhausted; searches answered by
regions; searches relaxed by the first pass; first-pass steps; requests
queued at end of tick; ticks with a non-empty queue; deepest queue; most
expansions in a single search; searches begun with a blocked goal and how many
got a standable cell; searches where the first pass could not improve on the
start. These help show which fixes are worth having.

---

## 15. Deliberately out of scope

- Any alternative search algorithm or movement model from the baseline's own
  scheduler.
- Per-tick unit steering (aim point, braking rules) beyond the waypoint
  radii and the climb factor above; the host's existing follower is used.
- Treatment of recently moved units as passable; treat all other units as
  walls.
- Any tight-clearance (ring) classification beyond the slope-based "rough"
  test in 1.4.
