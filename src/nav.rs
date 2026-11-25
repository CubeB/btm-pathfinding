//! Per-unit navigation state: the provisional straight route, route requests
//! and their rate limit, the drifting goal, waypoint retiring, arrival tests and
//! the climb speed factor (SPEC 3, 6, 8, 10, 1.4).
//!
//! [`Nav`] is pure state driven by the host each tick:
//! 1. [`Nav::set_goal`] whenever the order's resolved destination may have changed;
//! 2. [`Nav::on_delivery`] / [`Nav::on_request_lost`] with the service's output;
//! 3. [`Nav::follow`] for the waypoint to steer at;
//! 4. [`Nav::poll_request`], forwarding any goal it returns to
//!    [`crate::service::Pathfinder::request`].

use crate::geom::{isqrt_i64, vertex_of, CellRect, Footprint, WorldPos, CELL, UNIT};
use crate::route::{ground_height, waypoint, Waypoint};
use crate::service::Delivery;
use crate::terrain::{MoveClass, Terrain};
use crate::world::Goal;

/// Fixed-point 1.0 for speed factors.
pub const FACTOR_ONE: u32 = 1024;

/// Navigation constants (SPEC 12). Lengths are fixed point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NavConfig {
    /// Waypoint retire radius (16 world units).
    pub retire_radius: i32,
    /// Final waypoint retire radius (8 world units).
    pub final_radius: i32,
    /// Goal drift tolerance (64 world units).
    pub drift_tolerance: i32,
    /// Ticks between requests of one unit (60).
    pub request_cooldown: u64,
    /// Route age before a refresh on finishing or colliding (30 ticks).
    pub refresh_age: u64,
    /// Rectangle arrival radius (8 world units).
    pub rect_arrival_radius: i32,
    /// Host choice (SPEC 1.4, tunable): climb look-ahead (16 world units).
    pub climb_lookahead: i32,
    /// Host choice (SPEC 1.4, tunable): climb factor floor (0.25 = 256).
    pub climb_floor: u32,
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            retire_radius: 16 * UNIT,
            final_radius: 8 * UNIT,
            drift_tolerance: 64 * UNIT,
            request_cooldown: 60,
            refresh_age: 30,
            rect_arrival_radius: 8 * UNIT,
            climb_lookahead: 16 * UNIT,
            climb_floor: FACTOR_ONE / 4,
        }
    }
}

/// Where a goal is aimed for the provisional route: the point, or a
/// rectangle's centre.
pub fn goal_point(g: Goal) -> WorldPos {
    match g {
        Goal::Point(p) => p,
        Goal::Rect(r) => r.centre(),
    }
}

fn drifting(cfg: &NavConfig, built_for: Option<Goal>, new: Goal) -> bool {
    match (built_for, new) {
        (Some(Goal::Point(a)), Goal::Point(b)) => {
            let t = i64::from(cfg.drift_tolerance);
            a.dist2(b) <= t * t
        }
        _ => false,
    }
}

/// One unit's navigation state.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Nav {
    goal: Option<Goal>,
    route: Vec<Waypoint>,
    next: usize,
    provisional: bool,
    route_goal: Option<Goal>,
    route_tick: u64,
    unreachable: bool,
    relaxed: bool,
    reachable: Option<Waypoint>,
    in_flight: Option<Goal>,
    last_request: Option<u64>,
    wants: bool,
}

impl Nav {
    /// No order, no route.
    pub fn new() -> Self {
        Self::default()
    }

    /// The current resolved destination. Call on every order and whenever the
    /// destination may have moved. A change within the drift tolerance of a real
    /// route's destination keeps the route (SPEC 6.2); anything else installs the
    /// provisional straight route at once (SPEC 3). An in-flight request is never
    /// interrupted (SPEC 10). Returns whether a provisional route was installed.
    pub fn set_goal<T: Terrain + ?Sized>(
        &mut self,
        cfg: &NavConfig,
        terrain: &T,
        goal: Goal,
        pos: WorldPos,
        tick: u64,
    ) -> bool {
        if self.goal == Some(goal) {
            return false;
        }
        let keep =
            !self.provisional && !self.route.is_empty() && drifting(cfg, self.route_goal, goal);
        self.goal = Some(goal);
        self.wants = true;
        if keep {
            return false;
        }
        self.route = vec![waypoint(terrain, pos), waypoint(terrain, goal_point(goal))];
        self.next = 0;
        self.provisional = true;
        self.route_goal = Some(goal);
        self.route_tick = tick;
        self.unreachable = false;
        self.relaxed = false;
        self.reachable = None;
        true
    }

    /// The order ended: forget goal and route. The request timestamp survives.
    pub fn clear(&mut self) {
        let last = self.last_request;
        let in_flight = self.in_flight;
        *self = Self::default();
        self.last_request = last;
        self.in_flight = in_flight;
    }

    /// Whether to queue a route request now (SPEC 10), and for which goal. Pass
    /// whether the unit is blocked by another and whether its order is already
    /// satisfied.
    pub fn poll_request(
        &mut self,
        cfg: &NavConfig,
        tick: u64,
        collided: bool,
        satisfied: bool,
    ) -> Option<Goal> {
        let goal = self.goal?;
        if self.in_flight.is_some() {
            return None;
        }
        let old = self.route.is_empty() || tick.saturating_sub(self.route_tick) >= cfg.refresh_age;
        if self.provisional || (old && ((self.at_end() && !satisfied) || collided)) {
            self.wants = true;
        }
        if !self.wants {
            return None;
        }
        if let Some(last) = self.last_request {
            if tick < last.saturating_add(cfg.request_cooldown) {
                return None;
            }
        }
        self.last_request = Some(tick);
        self.in_flight = Some(goal);
        self.wants = false;
        Some(goal)
    }

    /// A route landed. It replaces the route wholesale unless the destination
    /// has since changed by more than the drift tolerance, in which case it is
    /// discarded and a new one wanted. Returns whether it was installed.
    pub fn on_delivery(&mut self, cfg: &NavConfig, d: &Delivery, tick: u64) -> bool {
        self.in_flight = None;
        let Some(goal) = self.goal else {
            return false;
        };
        let fits = goal == d.goal || drifting(cfg, Some(d.goal), goal);
        if !fits {
            self.wants = true;
            return false;
        }
        self.route.clone_from(&d.route.waypoints);
        self.next = 0;
        self.provisional = false;
        self.route_goal = Some(d.goal);
        self.route_tick = tick;
        self.unreachable = d.route.unreachable;
        self.relaxed = d.route.goal_relaxed;
        self.reachable = d.route.reachable_destination();
        if goal != d.goal {
            self.wants = true;
        }
        true
    }

    /// The request was dropped, abandoned or failed: want a route again.
    pub fn on_request_lost(&mut self) {
        self.in_flight = None;
        if self.goal.is_some() {
            self.wants = true;
        }
    }

    /// Retire at most one waypoint (16 world units, 8 for the last) and return
    /// the one to steer at, or `None` at the end of the route (SPEC 6.1).
    pub fn follow(&mut self, cfg: &NavConfig, pos: WorldPos) -> Option<WorldPos> {
        if let Some(wp) = self.route.get(self.next) {
            let r = i64::from(if self.next + 1 == self.route.len() {
                cfg.final_radius
            } else {
                cfg.retire_radius
            });
            if pos.dist2(wp.pos) <= r * r {
                self.next += 1;
            }
        }
        self.route.get(self.next).map(|w| w.pos)
    }

    /// The current destination.
    pub fn goal(&self) -> Option<Goal> {
        self.goal
    }
    /// The route being walked.
    pub fn route(&self) -> &[Waypoint] {
        &self.route
    }
    /// Index of the waypoint being steered at.
    pub fn next_waypoint(&self) -> usize {
        self.next
    }
    /// Whether every waypoint is retired.
    pub fn at_end(&self) -> bool {
        self.next >= self.route.len()
    }
    /// Whether the route is the provisional straight line.
    pub fn is_provisional(&self) -> bool {
        self.provisional
    }
    /// The goal of the request in flight, if any. Hosts report this as
    /// [`crate::world::UnitView::destination`].
    pub fn in_flight(&self) -> Option<Goal> {
        self.in_flight
    }
    /// Whether a route is wanted but not yet requested.
    pub fn wants_route(&self) -> bool {
        self.wants
    }
    /// Tick of the last request.
    pub fn last_request(&self) -> Option<u64> {
        self.last_request
    }
    /// The reachable destination of an unreachable route (SPEC 8).
    pub fn reachable_destination(&self) -> Option<Waypoint> {
        self.reachable
    }
    /// Whether the route is flagged unreachable.
    pub fn is_unreachable(&self) -> bool {
        self.unreachable
    }
    /// Whether the route ends at a relaxed goal (SPEC 5).
    pub fn is_relaxed(&self) -> bool {
        self.relaxed
    }
}

/// A plain move's arrival test: the unit stands on the goal's cell, read as
/// "its search vertex is the goal's vertex".
pub fn point_arrived(pos: WorldPos, fp: Footprint, dest: WorldPos) -> bool {
    vertex_of(pos, fp) == vertex_of(dest, fp)
}

/// The rectangle arrival test (SPEC 6.1): within `radius` of the boundary of
/// the rectangle enlarged by half the footprint. A point deeper inside than
/// `radius` has not arrived (so a pad-clearing unit still on the pad keeps going).
pub fn rect_arrived(pos: WorldPos, fp: Footprint, rect: CellRect, radius: i32) -> bool {
    let c = i64::from(CELL);
    let (hw, hh) = (i64::from(fp.w) * c / 2, i64::from(fp.h) * c / 2);
    let x0 = i64::from(rect.x0) * c - hw;
    let x1 = i64::from(rect.x1) * c + hw;
    let y0 = i64::from(rect.y0) * c - hh;
    let y1 = i64::from(rect.y1) * c + hh;
    let (px, py) = (i64::from(pos.x), i64::from(pos.y));
    let r = i64::from(radius);
    if px > x0 && px < x1 && py > y0 && py < y1 {
        (px - x0).min(x1 - px).min(py - y0).min(y1 - py) <= r
    } else {
        let dx = (x0 - px).max(px - x1).max(0);
        let dy = (y0 - py).max(py - y1).max(0);
        dx * dx + dy * dy <= r * r
    }
}

/// The order's arrival test, with the reachable destination standing in for
/// the goal once a route was flagged unreachable (SPEC 8).
pub fn order_arrived(nav: &Nav, cfg: &NavConfig, goal: Goal, pos: WorldPos, fp: Footprint) -> bool {
    if let Some(r) = nav.reachable_destination() {
        return point_arrived(pos, fp, r.pos);
    }
    match goal {
        Goal::Point(p) => point_arrived(pos, fp, p),
        Goal::Rect(r) => rect_arrived(pos, fp, r, cfg.rect_arrival_radius),
    }
}

/// The point `lookahead` ahead of `pos` towards `toward` (`pos` if equal).
pub fn ahead_point(pos: WorldPos, toward: WorldPos, lookahead: i32) -> WorldPos {
    let dx = i64::from(toward.x) - i64::from(pos.x);
    let dy = i64::from(toward.y) - i64::from(pos.y);
    let d = isqrt_i64(dx.saturating_mul(dx).saturating_add(dy.saturating_mul(dy)));
    if d == 0 {
        return pos;
    }
    let l = i64::from(lookahead);
    WorldPos::new(
        crate::geom::clamp_i32(i64::from(pos.x) + dx * l / d),
        crate::geom::clamp_i32(i64::from(pos.y) + dy * l / d),
    )
}

/// Climb speed factor (SPEC 1.4) in 1/1024: `1 - (rise / maxSlope) / 2`,
/// floored at `floor`; 1 when level, downhill, off-map ahead, or the class has
/// max slope 0 or >= 255.
pub fn climb_factor<T: Terrain + ?Sized>(
    terrain: &T,
    class: &MoveClass,
    pos: WorldPos,
    ahead: WorldPos,
    floor: u32,
) -> u32 {
    if class.max_slope <= 0 || class.max_slope >= 255 {
        return FACTOR_ONE;
    }
    let (cols, rows) = terrain.cells();
    let (mx, my) = (
        i64::from(cols) * i64::from(CELL),
        i64::from(rows) * i64::from(CELL),
    );
    if ahead.x < 0 || ahead.y < 0 || i64::from(ahead.x) > mx || i64::from(ahead.y) > my {
        return FACTOR_ONE;
    }
    let rise = i64::from(ground_height(terrain, ahead)) - i64::from(ground_height(terrain, pos));
    if rise <= 0 {
        return FACTOR_ONE;
    }
    let one = i64::from(FACTOR_ONE);
    let f = one - rise * one / (2 * i64::from(class.max_slope) * i64::from(UNIT));
    f.clamp(i64::from(floor), one) as u32
}

/// Underwater speed factor (SPEC 1.4): halved for a ground unit whose origin is
/// below sea level, unless it floats or hovers.
pub fn underwater_factor<T: Terrain + ?Sized>(
    terrain: &T,
    class: &MoveClass,
    pos: WorldPos,
) -> u32 {
    if class.floater || class.hover {
        return FACTOR_ONE;
    }
    let sea = i64::from(terrain.sea_level()) * i64::from(UNIT);
    if i64::from(ground_height(terrain, pos)) < sea {
        FACTOR_ONE / 2
    } else {
        FACTOR_ONE
    }
}
