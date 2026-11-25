//! Reproduces SPEC's reference scenarios on synthetic maps and prints
//! expansions, milliseconds per tick and arrivals. Timing is informative only.
//!
//! Run with `cargo run --release --example bench`.

use std::time::Instant;

use btm_pathfinding::geom::{block_centre, UNIT};
use btm_pathfinding::grid::GridWorld;
use btm_pathfinding::rng::XorShift64;
use btm_pathfinding::sim::Sim;
use btm_pathfinding::{Cell, CellRect, ClassId, Config, Error, Footprint, Goal, MoveClass};

const ONE: Footprint = Footprint::square(1);

struct Report {
    expansions: u64,
    searches: u64,
    ms_total: f64,
    ms_max: f64,
    ticks: u64,
    arrivals: u64,
    units: usize,
}

fn run(mut sim: Sim, ticks: u64) -> Report {
    let mut ms_max = 0f64;
    let t0 = Instant::now();
    for _ in 0..ticks {
        let t = Instant::now();
        sim.step();
        ms_max = ms_max.max(t.elapsed().as_secs_f64() * 1000.0);
    }
    let stats = *sim.pf.stats();
    Report {
        expansions: stats.expansions,
        searches: stats.searches_begun,
        ms_total: t0.elapsed().as_secs_f64() * 1000.0,
        ms_max,
        ticks,
        arrivals: sim.arrivals,
        units: sim.units.len(),
    }
}

fn print(scenario: &str, variant: &str, r: &Report) {
    let per = r.expansions.checked_div(r.searches).unwrap_or(0);
    println!(
        "{scenario:<24} {variant:<18} expansions {:>9}  searches {:>5}  exp/search {:>6}  ms/tick {:>6.3} (max {:>6.2})  arrivals {:>3}/{}",
        r.expansions,
        r.searches,
        per,
        r.ms_total / r.ticks as f64,
        r.ms_max,
        r.arrivals,
        r.units
    );
}

/// SPEC 4.4: 100 units pressed against a channel they cannot cross, all
/// ordered across it, 600 ticks.
fn pressed_against_water(cfg: Config) -> Result<Report, Error> {
    let mut w = GridWorld::flat(256, 128, 10)?;
    w.set_sea_level(0);
    w.set_heights(|x, _| if (200..=206).contains(&x) { -50 } else { 10 });
    let mut sim = Sim::new(w, vec![MoveClass::tank(ONE, 20, 0)], cfg)?;
    let mut ids = Vec::new();
    for y in 14..114 {
        ids.push((
            sim.add_unit(ClassId(0), block_centre(Cell::new(198, y), ONE), 2 * UNIT)?,
            y,
        ));
    }
    for (id, y) in ids {
        sim.order(id, Goal::Point(block_centre(Cell::new(220, y), ONE)))?;
    }
    Ok(run(sim, 600))
}

/// SPEC 5: 200 units all sent at one blocked point (a building), 900 ticks.
fn crowded_blocked_point(cfg: Config) -> Result<Report, Error> {
    let mut w = GridWorld::flat(128, 128, 0)?;
    w.add_building(CellRect::new(62, 62, 66, 66), &[])?;
    let mut sim = Sim::new(w, vec![MoveClass::tank(ONE, 20, 0)], cfg)?;
    let mut rng = XorShift64::new(2024);
    let mut ids = Vec::new();
    while ids.len() < 200 {
        let c = Cell::new(rng.range(0, 128), rng.range(0, 128));
        if (c.x - 64).abs() < 30 && (c.y - 64).abs() < 30 {
            continue;
        }
        if sim.world.occupancy(c) > 0 {
            continue;
        }
        ids.push(sim.add_unit(ClassId(0), block_centre(c, ONE), 2 * UNIT)?);
    }
    for id in ids {
        sim.order(id, Goal::Point(block_centre(Cell::new(64, 64), ONE)))?;
    }
    Ok(run(sim, 900))
}

/// Long moves across a rocky 256 x 256 field: 400 units, 1200 ticks.
fn open_field(cfg: Config) -> Result<Report, Error> {
    let mut w = GridWorld::flat(256, 256, 0)?;
    let mut rng = XorShift64::new(77);
    for _ in 0..300 {
        let (x, y) = (rng.range(0, 250), rng.range(0, 250));
        let r = if rng.chance(1, 2) {
            CellRect::from_size(x, y, rng.range(2, 20), 1)
        } else {
            CellRect::from_size(x, y, 1, rng.range(2, 20))
        };
        w.fill_features(r, true);
    }
    let mut sim = Sim::new(w, vec![MoveClass::tank(ONE, 20, 0)], cfg)?;
    let mut ids = Vec::new();
    while ids.len() < 400 {
        let c = Cell::new(rng.range(0, 60), rng.range(0, 256));
        if sim.world.static_blocked(c) || sim.world.occupancy(c) > 0 {
            continue;
        }
        ids.push(sim.add_unit(ClassId(0), block_centre(c, ONE), 3 * UNIT)?);
    }
    for id in ids {
        let g = Cell::new(rng.range(120, 180), rng.range(0, 256));
        sim.order(id, Goal::Point(block_centre(g, ONE)))?;
    }
    Ok(run(sim, 1200))
}

fn main() -> Result<(), Error> {
    let on = Config::default();
    let no_regions = Config {
        region_precheck: false,
        ..on
    };
    let no_relax = Config {
        relax_blocked_goal: false,
        ..on
    };
    let bare = Config {
        relax_blocked_goal: false,
        first_pass: false,
        region_precheck: false,
        ..on
    };
    print(
        "pressed against water",
        "no regions",
        &pressed_against_water(no_regions)?,
    );
    print(
        "pressed against water",
        "regions",
        &pressed_against_water(on)?,
    );
    print(
        "crowded blocked point",
        "no relaxation",
        &crowded_blocked_point(no_relax)?,
    );
    print(
        "crowded blocked point",
        "A* only",
        &crowded_blocked_point(bare)?,
    );
    print(
        "crowded blocked point",
        "relaxation",
        &crowded_blocked_point(on)?,
    );
    print("open field 400 units", "default", &open_field(on)?);
    Ok(())
}
