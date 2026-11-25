//! Property tests over random maps, driven by the in-crate xorshift generator.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use btm_pathfinding::geom::vertex_of;
use btm_pathfinding::grid::GridWorld;
use btm_pathfinding::rng::XorShift64;
use btm_pathfinding::{
    Cell, CellRect, ClassId, Config, Footprint, Goal, Mobility, MoveClass, Pathfinder,
    PrecheckCharge, Resolution, UnitId,
};
use common::{centre, check_path, drain, reachable, ref_standable};

fn random_map(rng: &mut XorShift64, cols: u32, rows: u32) -> GridWorld {
    let mut w = GridWorld::flat(cols, rows, 10).unwrap();
    w.set_sea_level(0);
    // Lakes: impassable terrain for the land classes, so regions matter.
    for _ in 0..rng.below(4) {
        let (cx, cy) = (rng.range(0, cols as i32), rng.range(0, rows as i32));
        let r = rng.range(1, 6);
        for y in (cy - r).max(0)..=(cy + r).min(rows as i32) {
            for x in (cx - r).max(0)..=(cx + r).min(cols as i32) {
                w.set_sample(x as u32, y as u32, -20);
            }
        }
    }
    // A wall across the map with at most one gap.
    if rng.chance(1, 3) {
        let x = rng.range(5, cols as i32 - 5);
        w.fill_features(CellRect::new(x, 0, x + 1, rows as i32), true);
        if rng.chance(1, 2) {
            let gy = rng.range(0, rows as i32 - 2);
            w.fill_features(CellRect::new(x, gy, x + 1, gy + 2), false);
        }
    }
    // Scattered rocks and short walls.
    let density = rng.below(25);
    for y in 0..rows as i32 {
        for x in 0..cols as i32 {
            if rng.chance(density, 100) {
                w.set_feature(Cell::new(x, y), true);
            }
        }
    }
    for _ in 0..rng.below(6) {
        let (x, y) = (rng.range(0, cols as i32), rng.range(0, rows as i32));
        let r = if rng.chance(1, 2) {
            CellRect::from_size(x, y, rng.range(2, 15), 1)
        } else {
            CellRect::from_size(x, y, 1, rng.range(2, 15))
        };
        w.fill_features(r, true);
    }
    w
}

fn classes() -> Vec<MoveClass> {
    vec![
        MoveClass::tank(Footprint::square(1), 30, 0),
        MoveClass::tank(Footprint::square(2), 30, 0),
    ]
}

struct Case {
    w: GridWorld,
    reqs: Vec<(UnitId, ClassId, Footprint, Goal)>,
}

fn random_case(seed: u64, n: usize) -> Case {
    let mut rng = XorShift64::new(seed);
    let (cols, rows) = (rng.range(12, 48) as u32, rng.range(12, 48) as u32);
    let mut w = random_map(&mut rng, cols, rows);
    let mut reqs = Vec::new();
    for _ in 0..n {
        let k = rng.below(2) as u16;
        let fp = Footprint::square(i32::from(k) + 1);
        let (x, y) = (
            rng.range(0, cols as i32 - fp.w + 1),
            rng.range(0, rows as i32 - fp.h + 1),
        );
        // In this world the units are obstacles to each other (budget test);
        // `solo` rebuilds the map with only the searcher for route checks.
        let id = w
            .add_unit(centre(x, y, fp), Mobility::Class(ClassId(k)), fp)
            .unwrap();
        w.set_unit_moving(id, true).unwrap();
        let goal = if rng.chance(1, 5) {
            let (gx, gy) = (rng.range(0, cols as i32 - 2), rng.range(0, rows as i32 - 2));
            Goal::Rect(CellRect::from_size(
                gx,
                gy,
                rng.range(1, 4),
                rng.range(1, 4),
            ))
        } else {
            Goal::Point(centre(
                rng.range(0, cols as i32),
                rng.range(0, rows as i32),
                Footprint::square(1),
            ))
        };
        reqs.push((id, ClassId(k), fp, goal));
    }
    Case { w, reqs }
}

/// Run each request alone in a world where only the searcher exists.
fn solo(
    seed: u64,
    n: usize,
    cfg: Config,
) -> Vec<(
    GridWorld,
    ClassId,
    Footprint,
    Goal,
    btm_pathfinding::Delivery,
)> {
    let case = random_case(seed, n);
    let mut out = Vec::new();
    for &(id, class, fp, goal) in &case.reqs {
        let rec = case.w.record(id).unwrap();
        let mut rng = XorShift64::new(seed);
        let (cols, rows) = (rng.range(12, 48) as u32, rng.range(12, 48) as u32);
        let mut w = random_map(&mut rng, cols, rows);
        let me = w.add_unit(rec.pos, rec.mobility, rec.footprint).unwrap();
        w.set_unit_moving(me, true).unwrap();
        let mut pf = Pathfinder::new(cfg, &w, classes()).unwrap();
        pf.request(me, goal).unwrap();
        let (ds, es, outs) = drain(&mut pf, &w, 1_000_000);
        assert!(es.is_empty(), "{es:?}");
        assert_eq!(ds.len(), 1);
        assert!(outs.iter().all(|o| o.charged <= cfg.budget_per_tick));
        out.push((w, class, fp, goal, ds.into_iter().next().unwrap()));
    }
    out
}

#[test]
fn routes_never_cross_impassable_cells() {
    for seed in 1..=60u64 {
        for (w, class, fp, _goal, d) in solo(seed, 6, Config::default()) {
            let pf = Pathfinder::new(Config::default(), &w, classes()).unwrap();
            let grid = pf.class_grid(class).unwrap();
            assert!(d.route.waypoints.len() >= 2);
            check_path(&w, grid, fp, &d.route.path);
        }
    }
}

#[test]
fn unreachable_agrees_with_full_search() {
    let mut unreachable_seen = 0;
    let mut regions_seen = 0;
    for seed in 1..=80u64 {
        for (w, class, fp, goal, d) in solo(seed, 6, Config::default()) {
            let pf = Pathfinder::new(Config::default(), &w, classes()).unwrap();
            let grid = pf.class_grid(class).unwrap();
            let start = d.route.path[0];
            let seen = reachable(&w, grid, fp, start);
            let (vw, vh) = grid.extent();
            let own = CellRect::block(start, fp);
            let goal_reachable = match goal {
                Goal::Point(p) => {
                    let g = vertex_of(p, fp);
                    g.x >= 0 && g.y >= 0 && g.x < vw && g.y < vh && seen[(g.y * vw + g.x) as usize]
                }
                Goal::Rect(r) => (0..vh).any(|y| {
                    (0..vw).any(|x| {
                        let c = Cell::new(x, y);
                        seen[(y * vw + x) as usize] && btm_pathfinding::search::touches(c, fp, r)
                    })
                }),
            };
            if d.route.unreachable {
                unreachable_seen += 1;
                assert!(
                    !goal_reachable,
                    "seed {seed}: flagged unreachable but a route exists ({:?})",
                    d.resolution
                );
            }
            if d.resolution == Resolution::Regions {
                regions_seen += 1;
            }
            if goal_reachable {
                assert!(!d.route.unreachable);
                if let Goal::Point(p) = goal {
                    let g = vertex_of(p, fp);
                    if ref_standable(&w, grid, fp, own, g) {
                        assert!(
                            d.route.exact,
                            "seed {seed}: stopped short of a reachable goal {g:?}"
                        );
                    }
                }
            }
            // Everything on the path is reachable in the full search.
            for c in &d.route.path {
                assert!(seen[(c.y * vw + c.x) as usize]);
            }
        }
    }
    assert!(unreachable_seen > 10, "{unreachable_seen}");
    let _ = regions_seen;
}

/// Starts placed at the vertex of their reachable area nearest the goal, so
/// the first pass cannot improve and the region precheck gets its chance.
#[test]
fn region_precheck_agrees_with_full_search() {
    let mut regions_seen = 0;
    let mut checked = 0;
    for seed in 1..=150u64 {
        let mut rng = XorShift64::new(seed * 7919);
        let (cols, rows) = (rng.range(12, 40) as u32, rng.range(12, 40) as u32);
        let mut w = random_map(&mut rng, cols, rows);
        let k = rng.below(2) as u16;
        let fp = Footprint::square(i32::from(k) + 1);
        let pf0 = Pathfinder::new(Config::default(), &w, classes()).unwrap();
        let grid = pf0.class_grid(ClassId(k)).unwrap().clone();
        let (vw, vh) = grid.extent();
        let s0 = Cell::new(rng.range(0, vw), rng.range(0, vh));
        if !ref_standable(&w, &grid, fp, CellRect::default(), s0) {
            continue;
        }
        let g = Cell::new(rng.range(0, vw), rng.range(0, vh));
        let seen = reachable(&w, &grid, fp, s0);
        let mut start = s0;
        for y in 0..vh {
            for x in 0..vw {
                let c = Cell::new(x, y);
                if seen[(y * vw + x) as usize]
                    && btm_pathfinding::geom::octile_score(c, g)
                        < btm_pathfinding::geom::octile_score(start, g)
                {
                    start = c;
                }
            }
        }
        let id = w
            .add_unit(
                btm_pathfinding::geom::block_centre(start, fp),
                Mobility::Class(ClassId(k)),
                fp,
            )
            .unwrap();
        w.set_unit_moving(id, true).unwrap();
        let goal = Goal::Point(btm_pathfinding::geom::block_centre(g, fp));
        let mut pf = Pathfinder::new(Config::default(), &w, classes()).unwrap();
        pf.request(id, goal).unwrap();
        let (ds, _, _) = drain(&mut pf, &w, 1_000_000);
        let d = &ds[0];
        let seen = reachable(&w, &grid, fp, start);
        let goal_reachable = seen[(g.y * vw + g.x) as usize];
        checked += 1;
        if d.resolution == Resolution::Regions {
            regions_seen += 1;
            assert_eq!(d.expansions, 0);
        }
        if d.route.unreachable {
            assert!(!goal_reachable, "seed {seed}");
        }
        if goal_reachable {
            assert!(d.route.exact, "seed {seed}");
        }
    }
    assert!(checked > 50, "{checked}");
    assert!(regions_seen > 5, "{regions_seen}");
}

#[test]
fn per_tick_budget_never_exceeded() {
    for seed in 1..=40u64 {
        let mut rng = XorShift64::new(seed * 31);
        let case = random_case(seed, 30);
        let budget = rng.range(1, 200) as u32;
        let cfg = Config {
            budget_per_tick: budget,
            precheck_charge: if rng.chance(1, 2) {
                PrecheckCharge::FirstPassSteps
            } else {
                PrecheckCharge::Fixed(rng.range(0, 500) as u32)
            },
            ..Config::default()
        };
        let mut pf = Pathfinder::new(cfg, &case.w, classes()).unwrap();
        let mut done = 0;
        for t in 0..20_000u32 {
            // Requests arrive over time, and some units ask again.
            if t < 200 {
                let i = rng.below(case.reqs.len() as u64) as usize;
                let (id, _, _, g) = case.reqs[i];
                pf.request(id, g).unwrap();
            }
            let o = pf.tick(&case.w);
            assert!(
                o.charged <= budget,
                "seed {seed} tick {t}: {} > {budget}",
                o.charged
            );
            assert!(o.expansions <= o.charged);
            done += o.deliveries.len();
            if t >= 200 && pf.queue_len() == 0 && pf.debt() == 0 {
                break;
            }
        }
        assert_eq!(pf.queue_len(), 0, "seed {seed}: queue drained");
        assert!(done > 0);
    }
}
