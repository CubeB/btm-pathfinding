//! The pathfinding service: a FIFO of requests, one sliced search at a time, a
//! shared per-tick expansion budget, routes and events out (SPEC 2, 4, 5).

use std::collections::VecDeque;

use crate::classes::ClassTable;
use crate::error::Error;
use crate::firstpass::{walk, FirstPass};
use crate::geom::{octile_score, vertex_of, Cell, CellRect, Footprint, WorldPos};
use crate::probe::Probe;
use crate::route::{build_route, Route};
use crate::search::{Scratch, Search, SearchGoal, SearchSpec, SearchStatus};
use crate::terrain::{ClassGrid, MoveClass, Terrain};
use crate::world::{ClassId, Goal, UnitId, World};

/// What a region-precheck answer (an answer with no A*) is charged (SPEC 2.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrecheckCharge {
    /// `max(1, first-pass steps)`, as SPEC suggests.
    FirstPassSteps,
    /// A fixed charge per answer.
    Fixed(u32),
}

/// Service configuration. Defaults are SPEC's constants; the switches exist
/// for measurement (and the SPEC 13.4 style experiments) and default to on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Expansions per tick, all searches combined (4000).
    pub budget_per_tick: u32,
    /// First-pass step bound (2000).
    pub first_pass_steps: u32,
    /// Blocked-goal look-out radius in cells (8).
    pub ring_radius: u32,
    /// Run the first pass before point-goal searches.
    pub first_pass: bool,
    /// Relax occupied point goals (SPEC 5).
    pub relax_blocked_goal: bool,
    /// Answer cross-region goals without A* (SPEC 4).
    pub region_precheck: bool,
    /// Host choice: what a region answer costs.
    pub precheck_charge: PrecheckCharge,
    /// Host choice: maximum queued requests (bounds memory).
    pub max_queue: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            budget_per_tick: 4000,
            first_pass_steps: 2000,
            ring_radius: 8,
            first_pass: true,
            relax_blocked_goal: true,
            region_precheck: true,
            precheck_charge: PrecheckCharge::FirstPassSteps,
            max_queue: 1 << 16,
        }
    }
}

/// How a delivered route was decided.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Resolution {
    /// The search accepted a vertex.
    Found,
    /// The search emptied its waiting list (unreachable).
    Exhausted,
    /// The region precheck answered without A* (unreachable).
    Regions,
}

/// A finished request.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Delivery {
    /// The unit.
    pub unit: UnitId,
    /// The goal the route was computed for.
    pub goal: Goal,
    /// The route.
    pub route: Route,
    /// How it was decided.
    pub resolution: Resolution,
    /// Expansions the search used.
    pub expansions: u64,
    /// First-pass steps taken (0 if it did not run).
    pub first_pass_steps: u32,
}

/// Requests that ended without a route.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Event {
    /// The unit was gone or no longer moving when its request was reached.
    Dropped(UnitId),
    /// A started search was thrown away: unit gone, stopped, or new destination.
    Abandoned(UnitId),
    /// The request could not be searched.
    Failed(UnitId, Error),
}

impl Event {
    /// The unit concerned.
    pub fn unit(&self) -> UnitId {
        match *self {
            Self::Dropped(u) | Self::Abandoned(u) | Self::Failed(u, _) => u,
        }
    }
}

/// What one tick produced.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TickOutput {
    /// Routes, in completion order.
    pub deliveries: Vec<Delivery>,
    /// Requests that ended without a route.
    pub events: Vec<Event>,
    /// Expansions this tick.
    pub expansions: u32,
    /// Budget used this tick: expansions plus region-answer charges. Never more
    /// than the budget.
    pub charged: u32,
}

/// Diagnostic counters (SPEC 14). They never feed back into behaviour.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Requests started (A* or precheck).
    pub searches_begun: u64,
    /// Expansions.
    pub expansions: u64,
    /// Ticks that ended with a search suspended.
    pub suspended: u64,
    /// Searches thrown away.
    pub abandoned: u64,
    /// Requests dropped unstarted.
    pub dropped: u64,
    /// Searches that exhausted their waiting list.
    pub exhausted: u64,
    /// Searches that found their goal.
    pub found: u64,
    /// Requests answered by regions.
    pub answered_by_regions: u64,
    /// Searches whose slack the first pass raised.
    pub relaxed_by_first_pass: u64,
    /// First-pass steps.
    pub first_pass_steps: u64,
    /// Searches where the first pass could not improve on the start.
    pub first_pass_no_improvement: u64,
    /// Requests queued at the end of the last tick.
    pub queued_at_end_of_tick: u64,
    /// Ticks with a non-empty queue.
    pub ticks_with_queue: u64,
    /// Deepest queue seen.
    pub deepest_queue: u64,
    /// Most expansions in a single search.
    pub most_expansions_in_search: u64,
    /// Searches begun with a blocked goal.
    pub blocked_goals: u64,
    /// Of those, how many got a standable ring vertex.
    pub blocked_goals_relaxed: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Request {
    unit: UnitId,
    goal: Goal,
}

#[derive(Debug)]
struct Active {
    req: Request,
    search: Search,
    fp: Footprint,
    exact: Option<(Cell, WorldPos)>,
    relaxed: bool,
    fp_steps: u32,
}

enum Begin {
    Dropped,
    Failed(Error),
    Answered(Box<Delivery>, u32),
    Started(Box<Active>),
}

/// The pathfinding service.
#[derive(Debug)]
pub struct Pathfinder {
    cfg: Config,
    table: ClassTable,
    scratch: Scratch,
    queue: VecDeque<Request>,
    active: Option<Box<Active>>,
    debt: u32,
    stats: Stats,
}

impl Pathfinder {
    /// Build the service for a map: every class's terrain grid is computed now
    /// (SPEC 1.7). `terrain` must be the terrain later passed to [`Self::tick`].
    pub fn new<T: Terrain + ?Sized>(
        cfg: Config,
        terrain: &T,
        classes: Vec<MoveClass>,
    ) -> Result<Self, Error> {
        let table = ClassTable::build(terrain, classes)?;
        let (cols, rows) = table.map_size();
        Ok(Self {
            cfg,
            table,
            scratch: Scratch::new(cols, rows),
            queue: VecDeque::new(),
            active: None,
            debt: 0,
            stats: Stats::default(),
        })
    }

    /// The configuration.
    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// Change the configuration (takes effect from the next request or tick).
    pub fn set_config(&mut self, cfg: Config) {
        self.cfg = cfg;
    }

    /// The movement classes.
    pub fn classes(&self) -> &ClassTable {
        &self.table
    }

    /// A class's terrain grid.
    pub fn class_grid(&self, id: ClassId) -> Option<&ClassGrid> {
        self.table.grid(id)
    }

    /// Counters.
    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    /// Requests waiting, including the active one.
    pub fn queue_len(&self) -> usize {
        self.queue.len()
    }

    /// Position of the unit's request in the queue (0 = head).
    pub fn queue_position(&self, unit: UnitId) -> Option<usize> {
        self.queue.iter().position(|r| r.unit == unit)
    }

    /// Whether the unit's search is started and suspended at the head.
    pub fn is_searching(&self, unit: UnitId) -> bool {
        self.active.as_ref().is_some_and(|a| a.req.unit == unit)
    }

    /// Budget still owed by region answers that cost more than was left.
    pub fn debt(&self) -> u32 {
        self.debt
    }

    /// Ask for a route (SPEC 2.1). A unit behind the head moves to the back; a
    /// unit at the head keeps it, and a new destination restarts its search.
    pub fn request(&mut self, unit: UnitId, goal: Goal) -> Result<(), Error> {
        if let Goal::Rect(r) = goal {
            if r.is_empty() {
                return Err(Error::EmptyRect);
            }
        }
        if let Some(front) = self.queue.front_mut() {
            if front.unit == unit {
                if front.goal != goal {
                    front.goal = goal;
                    if self.active.take().is_some() {
                        self.stats.abandoned += 1;
                    }
                }
                return Ok(());
            }
        }
        if let Some(pos) = self.queue_position(unit) {
            self.queue.remove(pos);
        } else if self.queue.len() >= self.cfg.max_queue {
            return Err(Error::QueueFull);
        }
        self.queue.push_back(Request { unit, goal });
        Ok(())
    }

    /// Withdraw the unit's request, abandoning its search if active.
    pub fn cancel(&mut self, unit: UnitId) -> bool {
        let Some(pos) = self.queue_position(unit) else {
            return false;
        };
        if pos == 0 && self.active.take().is_some() {
            self.stats.abandoned += 1;
        }
        self.queue.remove(pos);
        true
    }

    /// Run one tick of searching with the per-tick budget.
    pub fn tick<W: World + ?Sized>(&mut self, world: &W) -> TickOutput {
        let mut out = TickOutput::default();
        if !self.queue.is_empty() {
            self.stats.ticks_with_queue += 1;
        }
        self.stats.deepest_queue = self.stats.deepest_queue.max(self.queue.len() as u64);
        let budget = self.cfg.budget_per_tick;
        let pay = self.debt.min(budget);
        self.debt -= pay;
        out.charged = pay;
        let mut remaining = budget - pay;
        while remaining > 0 {
            let Some(active) = self.active.as_mut() else {
                let Some(req) = self.queue.front().copied() else {
                    break;
                };
                match self.begin(world, req) {
                    Begin::Dropped => {
                        self.queue.pop_front();
                        self.stats.dropped += 1;
                        out.events.push(Event::Dropped(req.unit));
                    }
                    Begin::Failed(e) => {
                        self.queue.pop_front();
                        out.events.push(Event::Failed(req.unit, e));
                    }
                    Begin::Answered(d, charge) => {
                        self.queue.pop_front();
                        let now = charge.min(remaining);
                        remaining -= now;
                        out.charged += now;
                        self.debt = self.debt.saturating_add(charge - now);
                        out.deliveries.push(*d);
                    }
                    Begin::Started(a) => self.active = Some(a),
                }
                continue;
            };
            let req = active.req;
            let view = world.unit(req.unit);
            let still =
                view.is_some_and(|v| v.moving && v.destination.is_none_or(|d| d == req.goal));
            if !still {
                self.active = None;
                self.queue.pop_front();
                self.stats.abandoned += 1;
                out.events.push(Event::Abandoned(req.unit));
                continue;
            }
            let res = active
                .search
                .run(world, &self.table, &mut self.scratch, remaining);
            match res {
                Err(e) => {
                    self.active = None;
                    self.queue.pop_front();
                    out.events.push(Event::Failed(req.unit, e));
                }
                Ok((spent, status)) => {
                    remaining = remaining.saturating_sub(spent);
                    out.expansions += spent;
                    out.charged += spent;
                    self.stats.expansions += u64::from(spent);
                    match status {
                        SearchStatus::Suspended => {
                            self.stats.suspended += 1;
                            break;
                        }
                        SearchStatus::Finished(o) => {
                            let a = &**active;
                            let exact = match a.exact {
                                Some((gv, dest)) if o.found && o.end == gv && !a.relaxed => {
                                    Some(dest)
                                }
                                _ => None,
                            };
                            let pos = view.map_or_else(WorldPos::default, |v| v.pos);
                            let route =
                                build_route(world, &o.path, a.fp, pos, exact, !o.found, a.relaxed);
                            if o.found {
                                self.stats.found += 1;
                            } else {
                                self.stats.exhausted += 1;
                            }
                            self.stats.most_expansions_in_search =
                                self.stats.most_expansions_in_search.max(o.expansions);
                            out.deliveries.push(Delivery {
                                unit: req.unit,
                                goal: req.goal,
                                route,
                                resolution: if o.found {
                                    Resolution::Found
                                } else {
                                    Resolution::Exhausted
                                },
                                expansions: o.expansions,
                                first_pass_steps: a.fp_steps,
                            });
                            self.active = None;
                            self.queue.pop_front();
                        }
                    }
                }
            }
        }
        self.stats.queued_at_end_of_tick = self.queue.len() as u64;
        out
    }

    fn begin<W: World + ?Sized>(&mut self, world: &W, req: Request) -> Begin {
        let Some(view) = world.unit(req.unit) else {
            return Begin::Dropped;
        };
        if !view.moving {
            return Begin::Dropped;
        }
        let (fp, class) = match self.table.resolve(view.mobility) {
            Ok(x) => x,
            Err(e) => return Begin::Failed(e),
        };
        let (vw, vh) = self.table.vertex_extent(fp);
        if vw <= 0 || vh <= 0 {
            return Begin::Failed(Error::FootprintExceedsMap);
        }
        let raw = vertex_of(view.pos, fp);
        let start = Cell::new(raw.x.clamp(0, vw - 1), raw.y.clamp(0, vh - 1));
        let own = CellRect::block(start, fp);
        self.stats.searches_begun += 1;
        let mut slack = 0u32;
        let mut relaxed = false;
        let mut fp_steps = 0u32;
        let (goal, exact) = match req.goal {
            Goal::Rect(r) => (SearchGoal::Rect(r), None),
            Goal::Point(p) => {
                let gv = vertex_of(p, fp);
                let pass = match Probe::new(world, &self.table, req.unit, view.mobility, own) {
                    Ok(probe) => {
                        if self.cfg.relax_blocked_goal && !probe.standable(gv) {
                            self.stats.blocked_goals += 1;
                            if let Some(s) = ring_scan(&probe, gv, self.cfg.ring_radius) {
                                self.stats.blocked_goals_relaxed += 1;
                                slack = slack.max(s);
                                relaxed = true;
                            }
                        }
                        if self.cfg.first_pass {
                            Some(walk(&probe, start, gv, self.cfg.first_pass_steps))
                        } else {
                            None
                        }
                    }
                    Err(e) => return Begin::Failed(e),
                };
                if let Some(fpr) = pass {
                    fp_steps = fpr.steps;
                    self.stats.first_pass_steps += u64::from(fpr.steps);
                    let improved = !fpr.reached
                        && fpr.closest_score < octile_score(start, gv)
                        && fpr.closest_score != 0;
                    if improved {
                        slack = slack.max(fpr.closest_score);
                        self.stats.relaxed_by_first_pass += 1;
                    } else if !fpr.reached {
                        self.stats.first_pass_no_improvement += 1;
                        if let Some(d) =
                            self.precheck(world, req, class, start, gv, &fpr, fp, view.pos, relaxed)
                        {
                            return d;
                        }
                    }
                }
                (SearchGoal::Vertex(gv), Some((gv, p)))
            }
        };
        let spec = SearchSpec {
            unit: req.unit,
            mobility: view.mobility,
            start,
            own,
            goal,
            slack,
            record_expansions: false,
        };
        match Search::new(spec, &self.table, &mut self.scratch) {
            Ok(search) => Begin::Started(Box::new(Active {
                req,
                search,
                fp,
                exact,
                relaxed,
                fp_steps,
            })),
            Err(e) => Begin::Failed(e),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn precheck<W: World + ?Sized>(
        &mut self,
        world: &W,
        req: Request,
        class: Option<ClassId>,
        start: Cell,
        gv: Cell,
        fpr: &FirstPass,
        fp: Footprint,
        pos: WorldPos,
        relaxed: bool,
    ) -> Option<Begin> {
        if !self.cfg.region_precheck {
            return None;
        }
        let class = class?;
        self.table.ensure_regions(class);
        let regions = self.table.regions(class)?;
        let rs = regions.region(start).unwrap_or(0);
        if rs == 0 {
            return None;
        }
        let differs = match regions.region(gv) {
            None | Some(0) => true,
            Some(rg) => rg != rs,
        };
        if !differs {
            return None;
        }
        let mut path = vec![start];
        if fpr.closest != start {
            path.push(fpr.closest);
        }
        let route = build_route(world, &path, fp, pos, None, true, relaxed);
        self.stats.answered_by_regions += 1;
        let charge = match self.cfg.precheck_charge {
            PrecheckCharge::FirstPassSteps => fpr.steps.max(1),
            PrecheckCharge::Fixed(n) => n,
        };
        Some(Begin::Answered(
            Box::new(Delivery {
                unit: req.unit,
                goal: req.goal,
                route,
                resolution: Resolution::Regions,
                expansions: 0,
                first_pass_steps: fpr.steps,
            }),
            charge,
        ))
    }
}

/// SPEC 5: the smallest octile score among standable vertices of the first
/// Chebyshev ring around `g` (radius 1 ..= `radius`) that has any.
fn ring_scan<W: World + ?Sized>(probe: &Probe<'_, W>, g: Cell, radius: u32) -> Option<u32> {
    let radius = radius.min(1024) as i32;
    for r in 1..=radius {
        let mut best: Option<u32> = None;
        let mut consider = |c: Cell| {
            if probe.standable(c) {
                let s = octile_score(c, g);
                if best.is_none_or(|b| s < b) {
                    best = Some(s);
                }
            }
        };
        for dx in -r..=r {
            consider(Cell::new(g.x + dx, g.y - r));
            consider(Cell::new(g.x + dx, g.y + r));
        }
        for dy in (-r + 1)..r {
            consider(Cell::new(g.x - r, g.y + dy));
            consider(Cell::new(g.x + r, g.y + dy));
        }
        if best.is_some() {
            return best;
        }
    }
    None
}
