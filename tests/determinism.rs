//! Identical inputs give identical outputs, run after run and slice after slice.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use btm_pathfinding::geom::UNIT;
use btm_pathfinding::grid::GridWorld;
use btm_pathfinding::rng::XorShift64;
use btm_pathfinding::sim::Sim;
use btm_pathfinding::{
    Cell, CellRect, ClassId, Config, Delivery, Footprint, Goal, Mobility, MoveClass, Pathfinder,
    WorldPos,
};
use common::{centre, drain};

const ONE: Footprint = Footprint::square(1);

fn map(seed: u64) -> GridWorld {
    let mut rng = XorShift64::new(seed);
    let mut w = GridWorld::flat(64, 64, 10).unwrap();
    w.set_sea_level(0);
    for _ in 0..20 {
        let x = rng.range(0, 60);
        let y = rng.range(0, 60);
        let r = CellRect::from_size(x, y, rng.range(1, 12), rng.range(1, 3));
        let r = if rng.chance(1, 2) {
            r
        } else {
            CellRect::from_size(x, y, r.y1 - r.y0, r.x1 - r.x0)
        };
        w.fill_features(r, true);
    }
    for _ in 0..3 {
        let (cx, cy) = (rng.range(5, 59) as u32, rng.range(5, 59) as u32);
        for y in cy - 3..=cy + 3 {
            for x in cx - 3..=cx + 3 {
                w.set_sample(x, y, -5);
            }
        }
    }
    w
}

fn classes() -> Vec<MoveClass> {
    vec![
        MoveClass::tank(ONE, 20, 0),
        MoveClass::tank(Footprint::square(2), 20, 8),
    ]
}

fn sim_run(seed: u64) -> (Vec<Delivery>, Vec<WorldPos>, u64) {
    let mut sim = Sim::new(map(seed), classes(), Config::default()).unwrap();
    let mut rng = XorShift64::new(seed ^ 0xABCD);
    let mut ids = Vec::new();
    for i in 0..60 {
        let class = ClassId((i % 2) as u16);
        let fp = if i % 2 == 0 {
            ONE
        } else {
            Footprint::square(2)
        };
        let (x, y) = (rng.range(0, 62), rng.range(0, 62));
        let grid = sim.pf.class_grid(class).unwrap();
        if !grid.standable(Cell::new(x, y)) || sim.world.static_blocked(Cell::new(x, y)) {
            continue;
        }
        ids.push(sim.add_unit(class, centre(x, y, fp), 3 * UNIT).unwrap());
    }
    let mut deliveries = Vec::new();
    for t in 0..400 {
        if t % 50 == 0 {
            for &id in &ids {
                if rng.chance(1, 2) {
                    let g = centre(rng.range(0, 62), rng.range(0, 62), ONE);
                    sim.order(id, Goal::Point(g)).unwrap();
                }
            }
        }
        let st = sim.step();
        deliveries.extend(st.output.deliveries);
    }
    let pos = sim.units.iter().map(|u| u.pos).collect();
    (deliveries, pos, sim.arrivals)
}

#[test]
fn repeated_simulations_identical() {
    for seed in [1, 7, 99] {
        let a = sim_run(seed);
        let b = sim_run(seed);
        assert!(!a.0.is_empty());
        assert_eq!(a, b, "seed {seed}");
    }
}

#[test]
fn routes_independent_of_budget_slicing() {
    let w0 = map(5);
    let mut rng = XorShift64::new(55);
    let mut w = w0.clone();
    let mut reqs = Vec::new();
    for _ in 0..40 {
        let (x, y) = (rng.range(0, 64), rng.range(0, 64));
        let id = w
            .add_unit(centre(x, y, ONE), Mobility::Class(ClassId(0)), ONE)
            .unwrap();
        w.set_unit_moving(id, true).unwrap();
        reqs.push((
            id,
            Goal::Point(centre(rng.range(0, 64), rng.range(0, 64), ONE)),
        ));
    }
    let mut results = Vec::new();
    for budget in [4000, 333, 7, 1] {
        let mut pf = Pathfinder::new(common::cfg_budget(budget), &w, classes()).unwrap();
        for &(id, g) in &reqs {
            pf.request(id, g).unwrap();
        }
        let (mut ds, es, outs) = drain(&mut pf, &w, 1_000_000);
        assert!(es.is_empty());
        assert!(outs.iter().all(|o| o.charged <= budget));
        ds.sort_by_key(|d| d.unit);
        results.push(ds);
    }
    for r in &results[1..] {
        assert_eq!(r, &results[0]);
    }
}
