//! A minimal deterministic simulation tying [`GridWorld`], [`Pathfinder`] and
//! [`Nav`] together, for tests and benchmarks. It stands in for the host's
//! movement: units steer straight at their waypoint at a fixed speed scaled by
//! the climb and underwater factors, slide along an axis when blocked, and are
//! flagged in collision when they cannot move at all.
//!
//! Host policies chosen here (SPEC 8 leaves them to the host): an order is
//! complete when its arrival test passes, or when a relaxed route's last
//! waypoint has been retired (the unit is as close as the crowd allows).

use crate::error::Error;
use crate::geom::{isqrt_i64, vertex_of, Cell, CellRect, Footprint, WorldPos};
use crate::grid::GridWorld;
use crate::nav::{
    ahead_point, climb_factor, order_arrived, underwater_factor, Nav, NavConfig, FACTOR_ONE,
};
use crate::service::{Config, Event, Pathfinder, TickOutput};
use crate::terrain::MoveClass;
use crate::world::{ClassId, Goal, Mobility, Obstacles, UnitId};

/// A simulated unit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SimUnit {
    /// Its id in the world.
    pub id: UnitId,
    /// Movement class.
    pub class: ClassId,
    /// Footprint.
    pub footprint: Footprint,
    /// Position.
    pub pos: WorldPos,
    /// Top speed, fixed point per tick.
    pub speed: i32,
    /// Navigation state.
    pub nav: Nav,
    /// Current order.
    pub order: Option<Goal>,
    /// Blocked last tick.
    pub collided: bool,
    /// Orders completed.
    pub completed: u32,
}

/// Totals of one simulated tick.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SimTick {
    /// The service's output.
    pub output: TickOutput,
    /// Orders completed this tick.
    pub arrivals: u32,
}

/// The simulation.
#[derive(Debug)]
pub struct Sim {
    /// The world.
    pub world: GridWorld,
    /// The pathfinding service.
    pub pf: Pathfinder,
    /// Units, indexed by id.
    pub units: Vec<SimUnit>,
    /// Navigation constants.
    pub nav_cfg: NavConfig,
    /// Ticks run.
    pub tick: u64,
    /// Orders completed in total.
    pub arrivals: u64,
}

impl Sim {
    /// A simulation over `world` with the given classes and service config.
    pub fn new(world: GridWorld, classes: Vec<MoveClass>, cfg: Config) -> Result<Self, Error> {
        let pf = Pathfinder::new(cfg, &world, classes)?;
        Ok(Self {
            world,
            pf,
            units: Vec::new(),
            nav_cfg: NavConfig::default(),
            tick: 0,
            arrivals: 0,
        })
    }

    /// Add a unit of `class` at `pos` with top speed `speed` (fixed point per tick).
    pub fn add_unit(&mut self, class: ClassId, pos: WorldPos, speed: i32) -> Result<UnitId, Error> {
        let fp = self
            .pf
            .classes()
            .class(class)
            .map(|c| c.footprint)
            .ok_or(Error::UnknownClass(class))?;
        let id = self.world.add_unit(pos, Mobility::Class(class), fp)?;
        if id.0 as usize != self.units.len() {
            return Err(Error::Capacity);
        }
        self.units.push(SimUnit {
            id,
            class,
            footprint: fp,
            pos,
            speed,
            nav: Nav::new(),
            order: None,
            collided: false,
            completed: 0,
        });
        Ok(id)
    }

    /// Give a unit a move order (or change its destination).
    pub fn order(&mut self, id: UnitId, goal: Goal) -> Result<(), Error> {
        let u = self
            .units
            .get_mut(id.0 as usize)
            .ok_or(Error::UnknownUnit(id))?;
        u.order = Some(goal);
        u.nav
            .set_goal(&self.nav_cfg, &self.world, goal, u.pos, self.tick);
        self.world.set_unit_moving(id, true)
    }

    /// A unit.
    pub fn unit(&self, id: UnitId) -> Option<&SimUnit> {
        self.units.get(id.0 as usize)
    }

    /// Run one tick: search, deliver, move, test arrival, request.
    pub fn step(&mut self) -> SimTick {
        let output = self.pf.tick(&self.world);
        for d in &output.deliveries {
            if let Some(u) = self.units.get_mut(d.unit.0 as usize) {
                u.nav.on_delivery(&self.nav_cfg, d, self.tick);
            }
        }
        for e in &output.events {
            if let Some(u) = self.units.get_mut(e.unit().0 as usize) {
                u.nav.on_request_lost();
            }
            if let Event::Failed(..) = e {
                // Nothing else to do: the unit keeps its provisional route.
            }
        }
        let mut arrivals = 0;
        for i in 0..self.units.len() {
            let Some(goal) = self.units[i].order else {
                continue;
            };
            self.move_unit(i);
            let cfg = self.nav_cfg;
            let tick = self.tick;
            let u = &mut self.units[i];
            let done = order_arrived(&u.nav, &cfg, goal, u.pos, u.footprint)
                || (u.nav.at_end() && !u.nav.is_provisional() && u.nav.is_relaxed());
            if done {
                u.order = None;
                u.nav.clear();
                u.completed += 1;
                arrivals += 1;
                let id = u.id;
                let _ = self.world.set_unit_moving(id, false);
                let _ = self.world.set_unit_destination(id, None);
                continue;
            }
            if let Some(g) = u.nav.poll_request(&cfg, tick, u.collided, false) {
                let id = u.id;
                let _ = self.world.set_unit_destination(id, Some(g));
                if self.pf.request(id, g).is_err() {
                    self.units[i].nav.on_request_lost();
                }
            }
        }
        self.arrivals += u64::from(arrivals);
        self.tick += 1;
        SimTick { output, arrivals }
    }

    fn free(&self, u: &SimUnit, p: WorldPos) -> bool {
        let cur = vertex_of(u.pos, u.footprint);
        let v = vertex_of(p, u.footprint);
        if v == cur {
            return true;
        }
        let Some(grid) = self.pf.class_grid(u.class) else {
            return false;
        };
        if !grid.standable(v) {
            return false;
        }
        let own = CellRect::block(cur, u.footprint);
        for y in v.y..v.y + u.footprint.h {
            for x in v.x..v.x + u.footprint.w {
                if !own.contains(Cell::new(x, y)) && self.world.cell_blocked(x, y, u.id) {
                    return false;
                }
            }
        }
        true
    }

    fn move_unit(&mut self, i: usize) {
        let cfg = self.nav_cfg;
        let pos = self.units[i].pos;
        let Some(target) = self.units[i].nav.follow(&cfg, pos) else {
            self.units[i].collided = false;
            return;
        };
        let dx = i64::from(target.x) - i64::from(pos.x);
        let dy = i64::from(target.y) - i64::from(pos.y);
        let dist = isqrt_i64(dx * dx + dy * dy);
        if dist == 0 {
            return;
        }
        let u = &self.units[i];
        let mut factor = u64::from(FACTOR_ONE);
        if let Some(class) = self.pf.classes().class(u.class) {
            let ahead = ahead_point(pos, target, cfg.climb_lookahead);
            factor = factor
                * u64::from(climb_factor(
                    &self.world,
                    class,
                    pos,
                    ahead,
                    cfg.climb_floor,
                ))
                / u64::from(FACTOR_ONE);
            factor = factor * u64::from(underwater_factor(&self.world, class, pos))
                / u64::from(FACTOR_ONE);
        }
        let speed = (i64::from(u.speed) * factor as i64 / i64::from(FACTOR_ONE)).max(1);
        let step = speed.min(dist);
        let nx = crate::geom::clamp_i32(i64::from(pos.x) + dx * step / dist);
        let ny = crate::geom::clamp_i32(i64::from(pos.y) + dy * step / dist);
        let candidates = [
            WorldPos::new(nx, ny),
            WorldPos::new(nx, pos.y),
            WorldPos::new(pos.x, ny),
        ];
        let next = candidates
            .into_iter()
            .find(|&p| p != pos && self.free(u, p));
        let id = u.id;
        match next {
            Some(p) => {
                self.units[i].pos = p;
                self.units[i].collided = false;
                let _ = self.world.set_unit_pos(id, p);
            }
            None => self.units[i].collided = true,
        }
    }
}
