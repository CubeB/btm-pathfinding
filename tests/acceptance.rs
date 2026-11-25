//! SPEC section 13 acceptance behaviours, one test (or more) each.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use btm_pathfinding::cost::Cost;
use btm_pathfinding::geom::{vertex_of, UNIT};
use btm_pathfinding::grid::GridWorld;
use btm_pathfinding::nav::{climb_factor, Nav, NavConfig, FACTOR_ONE};
use btm_pathfinding::pad::{site_decision, SiteDecision};
use btm_pathfinding::search::heuristic;
use btm_pathfinding::sim::Sim;
use btm_pathfinding::{
    Cell, CellRect, ClassId, ClassTable, Config, Delivery, Footprint, Goal, Mobility, MoveClass,
    Pathfinder, Resolution, Scratch, Search, SearchGoal, SearchSpec, SearchStatus, UnitId,
    WorldPos,
};
use common::{centre, check_path, drain};

const ONE: Footprint = Footprint::square(1);
const TANK: ClassId = ClassId(0);

fn tank1() -> MoveClass {
    MoveClass::tank(ONE, 10, 0)
}

fn spawn(w: &mut GridWorld, x: i32, y: i32, class: ClassId, fp: Footprint) -> UnitId {
    let id = w
        .add_unit(centre(x, y, fp), Mobility::Class(class), fp)
        .unwrap();
    w.set_unit_moving(id, true).unwrap();
    id
}

fn route_once(
    w: &GridWorld,
    classes: Vec<MoveClass>,
    cfg: Config,
    unit: UnitId,
    goal: Goal,
) -> Delivery {
    let mut pf = Pathfinder::new(cfg, w, classes).unwrap();
    pf.request(unit, goal).unwrap();
    let (ds, es, _) = drain(&mut pf, w, 10_000);
    assert!(es.is_empty(), "{es:?}");
    assert_eq!(ds.len(), 1);
    ds.into_iter().next().unwrap()
}

fn run_search(
    w: &GridWorld,
    table: &ClassTable,
    unit: UnitId,
    start: Cell,
    goal: SearchGoal,
    slack: u32,
    slice: u32,
) -> (btm_pathfinding::SearchOutcome, Vec<Cell>) {
    let (c, r) = table.map_size();
    let mut scratch = Scratch::new(c, r);
    let spec = SearchSpec {
        unit,
        mobility: Mobility::Class(TANK),
        start,
        own: CellRect::block(start, ONE),
        goal,
        slack,
        record_expansions: true,
    };
    let mut s = Search::new(spec, table, &mut scratch).unwrap();
    loop {
        let (spent, st) = s.run(w, table, &mut scratch, slice).unwrap();
        assert!(spent <= slice);
        if let SearchStatus::Finished(o) = st {
            return (o, s.expanded().to_vec());
        }
    }
}

// 1. No squeezing between diagonally touching obstacles.
#[test]
fn a01_no_corner_squeeze() {
    let mut w = GridWorld::flat(32, 32, 0).unwrap();
    w.set_feature(Cell::new(10, 10), true);
    w.set_feature(Cell::new(11, 11), true);
    let u = spawn(&mut w, 10, 11, TANK, ONE);
    let d = route_once(
        &w,
        vec![tank1()],
        Config::default(),
        u,
        Goal::Point(centre(11, 10, ONE)),
    );
    assert!(!d.route.unreachable && d.route.exact);
    assert!(d.route.waypoints.len() > 2, "{:?}", d.route.waypoints);
    assert!(d.route.path.len() > 2);
    let pf = Pathfinder::new(Config::default(), &w, vec![tank1()]).unwrap();
    check_path(&w, pf.class_grid(TANK).unwrap(), ONE, &d.route.path);

    // A whole diagonal wall of corner-touching rocks is walked round, not through.
    let mut w = GridWorld::flat(32, 32, 0).unwrap();
    for i in 4..=16 {
        w.set_feature(Cell::new(i, 20 - i), true);
    }
    let u = spawn(&mut w, 6, 6, TANK, ONE);
    let d = route_once(
        &w,
        vec![tank1()],
        Config::default(),
        u,
        Goal::Point(centre(14, 14, ONE)),
    );
    assert!(d.route.exact);
    assert!(d.route.waypoints.len() > 2);
    check_path(&w, pf.class_grid(TANK).unwrap(), ONE, &d.route.path);
    assert!(d
        .route
        .path
        .iter()
        .any(|c| c.x < 4 || c.y < 4 || c.x > 16 || c.y > 16));
}

fn corrugated_band() -> GridWorld {
    let mut w = GridWorld::flat(64, 64, 0).unwrap();
    w.set_heights(|x, y| {
        if x <= 52 && (16..=48).contains(&y) && y % 2 == 1 {
            6
        } else {
            0
        }
    });
    w
}

// 2. A steep-but-passable band is avoided by a flat detour; hover crosses it.
#[test]
fn a02_rough_band_detour() {
    let mut w = corrugated_band();
    let mut tank = MoveClass::tank(ONE, 10, 0);
    tank.bad_slope = Some(5);
    let hover = MoveClass::hover(ONE, 10);
    let u = spawn(&mut w, 40, 8, TANK, ONE);
    let goal = Goal::Point(centre(40, 56, ONE));
    let d = route_once(&w, vec![tank, hover], Config::default(), u, goal);
    let pf = Pathfinder::new(Config::default(), &w, vec![tank, hover]).unwrap();
    let grid = pf.class_grid(TANK).unwrap();
    assert!(grid.standable(Cell::new(40, 30)) && grid.rough(Cell::new(40, 30)));
    assert!(d.route.exact);
    assert!(
        d.route.path.iter().all(|&c| !grid.rough(c)),
        "tank crossed the band"
    );
    assert!(d.route.path.iter().any(|c| c.x >= 53));

    let mut w = corrugated_band();
    let h = w
        .add_unit(centre(40, 8, ONE), Mobility::Class(ClassId(1)), ONE)
        .unwrap();
    w.set_unit_moving(h, true).unwrap();
    let d = route_once(&w, vec![tank, hover], Config::default(), h, goal);
    assert!(d.route.exact);
    assert_eq!(d.route.waypoints.len(), 2, "hover goes straight");
    assert!(d.route.path.iter().all(|c| c.x == 40));
}

fn pond() -> GridWorld {
    let mut w = GridWorld::flat(20, 12, 2).unwrap();
    w.set_sea_level(0);
    for sx in 5..=7 {
        for sy in 5..=6 {
            w.set_sample(sx, sy, -2);
        }
    }
    w
}

// 3. A wader detours over dry land; a floater goes straight through.
#[test]
fn a03_wading_detour() {
    let wader = MoveClass::tank(ONE, 10, 5);
    let mut floater = MoveClass::tank(ONE, 10, 5);
    floater.floater = true;
    let mut w = pond();
    let u = spawn(&mut w, 2, 5, TANK, ONE);
    let goal = Goal::Point(centre(10, 5, ONE));
    let pf = Pathfinder::new(Config::default(), &w, vec![wader, floater]).unwrap();
    let grid = pf.class_grid(TANK).unwrap();
    let wet: Vec<i32> = (2..=10).filter(|&x| grid.wet(Cell::new(x, 5))).collect();
    assert_eq!(wet, vec![4, 5, 6, 7], "straight route has 4 wet vertices");
    let d = route_once(&w, vec![wader, floater], Config::default(), u, goal);
    assert!(d.route.exact);
    assert!(
        d.route.path.iter().all(|&c| !grid.wet(c)),
        "wader waded: {:?}",
        d.route.path
    );

    let mut w = pond();
    let f = w
        .add_unit(centre(2, 5, ONE), Mobility::Class(ClassId(1)), ONE)
        .unwrap();
    w.set_unit_moving(f, true).unwrap();
    let d = route_once(&w, vec![wader, floater], Config::default(), f, goal);
    assert_eq!(d.route.waypoints.len(), 2);
    assert_eq!(d.route.path.len(), 9);
}

// 4. Slack lets a blocked goal finish beside the obstacle cheaply.
#[test]
fn a04_slack_on_blocked_goal() {
    let mut w = GridWorld::flat(64, 64, 0).unwrap();
    let goal = Cell::new(40, 32);
    w.set_feature(goal, true);
    let u = spawn(&mut w, 10, 32, TANK, ONE);
    let table = ClassTable::build(&w, vec![tank1()]).unwrap();
    let (o, _) = run_search(
        &w,
        &table,
        u,
        Cell::new(10, 32),
        SearchGoal::Vertex(goal),
        0,
        u32::MAX,
    );
    assert!(!o.found);
    assert!(o.expansions > 1000, "{}", o.expansions);
    let (o, _) = run_search(
        &w,
        &table,
        u,
        Cell::new(10, 32),
        SearchGoal::Vertex(goal),
        10,
        u32::MAX,
    );
    assert!(o.found);
    assert!(o.expansions < 1000, "{}", o.expansions);
    assert_ne!(o.end, goal);
    assert_eq!(btm_pathfinding::geom::octile_score(o.end, goal), 10);

    // And through the service: the ring scan relaxes the goal (SPEC 5).
    let d = route_once(
        &w,
        vec![tank1()],
        Config::default(),
        u,
        Goal::Point(centre(40, 32, ONE)),
    );
    assert!(d.route.goal_relaxed && !d.route.unreachable && !d.route.exact);
    assert!(d.expansions < 1000);
    assert_eq!(*d.route.path.last().unwrap(), Cell::new(39, 32));
}

// 5. A sealed goal ends at the closest vertex, the earliest expanded of ties.
#[test]
fn a05_sealed_box_closest() {
    let mut w = GridWorld::flat(64, 64, 0).unwrap();
    let g = Cell::new(40, 32);
    for d in -2..=2 {
        for c in [
            Cell::new(g.x + d, g.y - 2),
            Cell::new(g.x + d, g.y + 2),
            Cell::new(g.x - 2, g.y + d),
            Cell::new(g.x + 2, g.y + d),
        ] {
            w.set_feature(c, true);
        }
    }
    let u = spawn(&mut w, 10, 32, TANK, ONE);
    let table = ClassTable::build(&w, vec![tank1()]).unwrap();
    let (o, expanded) = run_search(
        &w,
        &table,
        u,
        Cell::new(10, 32),
        SearchGoal::Vertex(g),
        0,
        u32::MAX,
    );
    assert!(!o.found);
    let h = |c: Cell| heuristic(SearchGoal::Vertex(g), ONE, c);
    let best: Cost = expanded.iter().map(|&c| h(c)).min().unwrap();
    let first = expanded.iter().copied().find(|&c| h(c) == best).unwrap();
    assert_eq!(o.end, first);
    assert_eq!(o.end, Cell::new(37, 32));
    let ties = expanded.iter().filter(|&&c| h(c) == best).count();
    assert_eq!(ties, 4, "four equally close vertices, one per side");

    // Through the service with only A*: flagged unreachable, ends there.
    let cfg = Config {
        first_pass: false,
        ..Config::default()
    };
    let d = route_once(&w, vec![tank1()], cfg, u, Goal::Point(centre(40, 32, ONE)));
    assert_eq!(d.resolution, Resolution::Exhausted);
    assert!(d.route.unreachable);
    assert_eq!(
        d.route.reachable_destination().unwrap().pos,
        centre(37, 32, ONE)
    );
}

// 6. One-expansion slices give the same route; no 1000 cap.
#[test]
fn a06_slicing_and_no_cap() {
    let mut w = GridWorld::flat(64, 64, 0).unwrap();
    for y in 5..60 {
        w.set_feature(Cell::new(30, y), true);
    }
    for x in 10..30 {
        w.set_feature(Cell::new(x, 20), true);
    }
    let u = spawn(&mut w, 5, 30, TANK, ONE);
    let table = ClassTable::build(&w, vec![tank1()]).unwrap();
    let goal = SearchGoal::Vertex(Cell::new(50, 30));
    let (whole, ew) = run_search(&w, &table, u, Cell::new(5, 30), goal, 0, u32::MAX);
    let (sliced, es) = run_search(&w, &table, u, Cell::new(5, 30), goal, 0, 1);
    assert_eq!(whole, sliced);
    assert_eq!(ew, es);
    assert!(whole.found);

    let a = route_once(
        &w,
        vec![tank1()],
        Config::default(),
        u,
        Goal::Point(centre(50, 30, ONE)),
    );
    let b = route_once(
        &w,
        vec![tank1()],
        common::cfg_budget(1),
        u,
        Goal::Point(centre(50, 30, ONE)),
    );
    assert_eq!(a, b);

    // A walled-off goal is declared unreachable only after > 4000 expansions
    // (5120 vertices on the near side of the wall).
    let mut w = GridWorld::flat(96, 64, 0).unwrap();
    w.fill_features(CellRect::new(80, 0, 82, 64), true);
    let u = spawn(&mut w, 5, 30, TANK, ONE);
    let cfg = Config {
        first_pass: false,
        ..Config::default()
    };
    let mut pf = Pathfinder::new(cfg, &w, vec![tank1()]).unwrap();
    pf.request(u, Goal::Point(centre(90, 30, ONE))).unwrap();
    let (ds, _, outs) = drain(&mut pf, &w, 100);
    assert!(outs.len() > 1, "took more than one tick");
    assert_eq!(ds[0].resolution, Resolution::Exhausted);
    assert!(ds[0].route.unreachable);
    assert!(ds[0].expansions > 4000, "{}", ds[0].expansions);
    assert!(outs.iter().all(|o| o.expansions <= 4000));
}

// 7. Asking again while suspended at the head keeps the place.
#[test]
fn a07_head_keeps_place() {
    let mut w = GridWorld::flat(64, 64, 0).unwrap();
    w.fill_features(CellRect::new(30, 5, 31, 64), true);
    let a = spawn(&mut w, 5, 30, TANK, ONE);
    let b = spawn(&mut w, 5, 40, TANK, ONE);
    let c = spawn(&mut w, 5, 50, TANK, ONE);
    let goal_a = Goal::Point(centre(50, 30, ONE));
    let goal_b = Goal::Point(centre(50, 40, ONE));
    let mut pf = Pathfinder::new(common::cfg_budget(10), &w, vec![tank1()]).unwrap();
    pf.request(a, goal_a).unwrap();
    pf.request(b, goal_b).unwrap();
    pf.request(c, goal_b).unwrap();
    pf.tick(&w);
    assert!(pf.is_searching(a));
    pf.request(a, goal_a).unwrap();
    assert_eq!(pf.queue_position(a), Some(0));
    assert!(pf.is_searching(a), "search carried on");
    // Behind the head: moves to the back.
    pf.request(b, goal_b).unwrap();
    assert_eq!(pf.queue_position(b), Some(2));
    let (ds, _, _) = drain(&mut pf, &w, 100_000);
    assert_eq!(ds[0].unit, a);
    let whole = route_once(&w, vec![tank1()], Config::default(), a, goal_a);
    assert_eq!(ds[0], whole, "resumed search equals an uninterrupted one");
    // A new destination at the head restarts in place.
    let mut pf = Pathfinder::new(common::cfg_budget(10), &w, vec![tank1()]).unwrap();
    pf.request(a, goal_a).unwrap();
    pf.request(b, goal_b).unwrap();
    pf.tick(&w);
    pf.request(a, goal_b).unwrap();
    assert_eq!(pf.queue_position(a), Some(0));
    assert!(!pf.is_searching(a));
    let (ds, _, _) = drain(&mut pf, &w, 100_000);
    assert_eq!((ds[0].unit, ds[0].goal), (a, goal_b));
}

// 8. Budget 0: the unit still walks toward the goal and has a request out.
#[test]
fn a08_moves_with_zero_budget() {
    let w = GridWorld::flat(64, 64, 0).unwrap();
    let mut sim = Sim::new(w, vec![tank1()], common::cfg_budget(0)).unwrap();
    let u = sim.add_unit(TANK, centre(5, 5, ONE), 2 * UNIT).unwrap();
    let goal = centre(50, 5, ONE);
    sim.order(u, Goal::Point(goal)).unwrap();
    let start = sim.unit(u).unwrap().pos;
    sim.step();
    let after = sim.unit(u).unwrap().clone();
    assert!(after.pos.x > start.x, "moving on the tick of the order");
    assert!(after.nav.is_provisional());
    assert_eq!(sim.pf.queue_position(u), Some(0));
    assert_eq!(after.nav.in_flight(), Some(Goal::Point(goal)));
    for _ in 0..100 {
        sim.step();
    }
    let later = sim.unit(u).unwrap();
    assert!(later.pos.x > after.pos.x + 100 * UNIT);
    assert_eq!(sim.pf.queue_position(u), Some(0));
}

// 9. A reachable goal is reached exactly, even behind walls and in a U trap.
#[test]
fn a09_reachable_goal_reached() {
    let shapes: Vec<Vec<CellRect>> = vec![
        vec![CellRect::new(30, 10, 31, 54)],
        vec![
            CellRect::new(30, 20, 31, 44),
            CellRect::new(20, 20, 31, 21),
            CellRect::new(20, 43, 31, 44),
        ],
        vec![CellRect::new(0, 30, 60, 31)],
        vec![
            CellRect::new(20, 10, 21, 50),
            CellRect::new(40, 14, 41, 64),
            CellRect::new(20, 10, 41, 11),
        ],
    ];
    for walls in shapes {
        let mut w = GridWorld::flat(64, 64, 0).unwrap();
        for r in &walls {
            w.fill_features(*r, true);
        }
        let u = spawn(&mut w, 25, 32, TANK, ONE);
        for goal in [centre(50, 32, ONE), centre(25, 60, ONE), centre(5, 2, ONE)] {
            let d = route_once(&w, vec![tank1()], Config::default(), u, Goal::Point(goal));
            assert!(!d.route.unreachable, "{walls:?} {goal:?}");
            assert!(d.route.exact, "stopped short: {walls:?} {goal:?}");
            assert_eq!(d.route.waypoints.last().unwrap().pos, goal);
        }
    }
}

fn channel() -> GridWorld {
    let mut w = GridWorld::flat(64, 32, 10).unwrap();
    w.set_sea_level(0);
    w.set_heights(|x, _| if (30..=34).contains(&x) { -50 } else { 10 });
    w
}

// 10. A goal across a too-deep channel is answered at once from regions.
#[test]
fn a10_channel_regions() {
    let class = MoveClass::tank(ONE, 255, 5);
    let mut w = channel();
    let u = spawn(&mut w, 28, 16, TANK, ONE);
    let mut pf = Pathfinder::new(Config::default(), &w, vec![class]).unwrap();
    assert!(!pf.class_grid(TANK).unwrap().standable(Cell::new(29, 16)));
    pf.request(u, Goal::Point(centre(40, 16, ONE))).unwrap();
    let out = pf.tick(&w);
    assert_eq!(out.deliveries.len(), 1);
    let d = &out.deliveries[0];
    assert_eq!(d.resolution, Resolution::Regions);
    assert_eq!(d.expansions, 0);
    assert_eq!(out.expansions, 0);
    assert!(d.first_pass_steps > 0);
    assert_eq!(out.charged, d.first_pass_steps.max(1));
    assert!(d.route.unreachable);
    let rd = d.route.reachable_destination().unwrap();
    assert_eq!(
        vertex_of(rd.pos, ONE),
        Cell::new(28, 16),
        "ends at the bank"
    );

    // Simulated: the unit stops at the bank and its order completes.
    let mut sim = Sim::new(channel(), vec![class], Config::default()).unwrap();
    let u = sim.add_unit(TANK, centre(28, 16, ONE), 2 * UNIT).unwrap();
    sim.order(u, Goal::Point(centre(40, 16, ONE))).unwrap();
    for _ in 0..5 {
        sim.step();
    }
    assert_eq!(sim.unit(u).unwrap().completed, 1);
    assert_eq!(vertex_of(sim.unit(u).unwrap().pos, ONE).x, 28);
    assert_eq!(sim.pf.stats().expansions, 0);
}

// 11. A drifting goal keeps its route and waits for the rate limit; a far one
//     takes the provisional route at once.
#[test]
fn a11_drift_and_rate_limit() {
    let w = GridWorld::flat(128, 64, 0).unwrap();
    let cfg = NavConfig::default();
    let mut nav = Nav::new();
    let pos = centre(5, 5, ONE);
    let g0 = Goal::Point(centre(40, 5, ONE));
    assert!(nav.set_goal(&cfg, &w, g0, pos, 0));
    assert_eq!(nav.poll_request(&cfg, 0, false, false), Some(g0));
    let mut w2 = w.clone();
    let u = w2.add_unit(pos, Mobility::Class(TANK), ONE).unwrap();
    w2.set_unit_moving(u, true).unwrap();
    let d = route_once(&w2, vec![tank1()], Config::default(), u, g0);
    assert!(nav.on_delivery(&cfg, &d, 5));
    assert!(!nav.is_provisional());
    let real = nav.route().to_vec();

    let moved = Goal::Point(WorldPos::new(
        centre(40, 5, ONE).x + 32 * UNIT,
        centre(40, 5, ONE).y,
    ));
    assert!(!nav.set_goal(&cfg, &w, moved, pos, 10));
    assert_eq!(nav.route(), &real[..], "route kept");
    assert!(nav.wants_route());
    for t in 10..60 {
        assert_eq!(nav.poll_request(&cfg, t, false, false), None, "tick {t}");
    }
    assert_eq!(nav.poll_request(&cfg, 60, false, false), Some(moved));
    nav.on_delivery(&cfg, &d, 61); // the old-goal route is within drift: installed

    let far = Goal::Point(WorldPos::new(
        centre(40, 5, ONE).x + 800 * UNIT,
        centre(40, 5, ONE).y,
    ));
    assert!(nav.set_goal(&cfg, &w, far, pos, 70));
    assert!(nav.is_provisional());
    assert_eq!(nav.route().len(), 2);
    assert_eq!(
        nav.route()[1].pos,
        WorldPos::new(centre(40, 5, ONE).x + 800 * UNIT, centre(40, 5, ONE).y)
    );
    // The cooldown survives the new order.
    assert_eq!(nav.poll_request(&cfg, 70, false, false), None);
    assert_eq!(nav.poll_request(&cfg, 120, false, false), Some(far));
}

// 12. A unit standing inside a blocking feature walks out of it.
#[test]
fn a12_walk_out_of_feature() {
    let two = Footprint::square(2);
    let class = MoveClass::tank(two, 10, 0);
    let mut w = GridWorld::flat(32, 32, 0).unwrap();
    w.fill_features(CellRect::new(10, 10, 12, 12), true);
    let mut sim = Sim::new(w, vec![class], Config::default()).unwrap();
    let u = sim.add_unit(TANK, centre(10, 10, two), 2 * UNIT).unwrap();
    let goal = centre(25, 25, two);
    sim.order(u, Goal::Point(goal)).unwrap();
    for _ in 0..400 {
        sim.step();
    }
    let unit = sim.unit(u).unwrap();
    assert_eq!(unit.completed, 1, "arrived");
    assert_eq!(vertex_of(unit.pos, two), Cell::new(25, 25));
}

// 13. A builder ordered onto an unreachable site gives up.
#[test]
fn a13_builder_gives_up() {
    let mut w = GridWorld::flat(64, 64, 0).unwrap();
    let site = CellRect::new(38, 30, 42, 34);
    let ring = site.expand(3, 3);
    for x in ring.x0..ring.x1 {
        w.set_feature(Cell::new(x, ring.y0), true);
        w.set_feature(Cell::new(x, ring.y1 - 1), true);
    }
    for y in ring.y0..ring.y1 {
        w.set_feature(Cell::new(ring.x0, y), true);
        w.set_feature(Cell::new(ring.x1 - 1, y), true);
    }
    let u = spawn(&mut w, 5, 5, TANK, ONE);
    let d = route_once(&w, vec![tank1()], Config::default(), u, Goal::Rect(site));
    assert!(d.route.unreachable);
    assert_eq!(site_decision(&d.route), SiteDecision::GiveUp);
    // An open site is reached and built.
    let w2 = {
        let mut w2 = GridWorld::flat(64, 64, 0).unwrap();
        let u2 = spawn(&mut w2, 5, 5, TANK, ONE);
        assert_eq!(u2, u);
        w2
    };
    let d = route_once(&w2, vec![tank1()], Config::default(), u, Goal::Rect(site));
    assert!(!d.route.unreachable);
    assert_eq!(site_decision(&d.route), SiteDecision::Proceed);
    let end = *d.route.path.last().unwrap();
    assert!(btm_pathfinding::search::touches(end, ONE, site));
}

// 14. Climbing slows toward half at the slope limit, never below a quarter.
#[test]
fn a14_climb_factor() {
    let class = MoveClass::tank(ONE, 10, 0);
    let floor = NavConfig::default().climb_floor;
    let look = NavConfig::default().climb_lookahead;
    let at = |k: i32, dir: i32, class: &MoveClass| {
        let mut w = GridWorld::flat(32, 32, 0).unwrap();
        w.set_heights(|x, _| x as i32 * k);
        let pos = centre(16, 16, ONE);
        let ahead = WorldPos::new(pos.x + dir * look, pos.y);
        climb_factor(&w, class, pos, ahead, floor)
    };
    assert_eq!(at(0, 1, &class), FACTOR_ONE, "flat");
    assert_eq!(at(10, 1, &class), FACTOR_ONE / 2, "rise = max slope halves");
    assert_eq!(at(5, 1, &class), FACTOR_ONE * 3 / 4);
    assert_eq!(at(40, 1, &class), FACTOR_ONE / 4, "floor");
    assert_eq!(at(10, -1, &class), FACTOR_ONE, "downhill");
    let mut free = class;
    free.max_slope = 255;
    assert_eq!(at(10, 1, &free), FACTOR_ONE);
    free.max_slope = 0;
    assert_eq!(at(10, 1, &free), FACTOR_ONE);
    // Off-map ahead.
    let w = GridWorld::flat(32, 32, 0).unwrap();
    assert_eq!(
        climb_factor(
            &w,
            &class,
            centre(0, 0, ONE),
            WorldPos::new(-20 * UNIT, 0),
            floor
        ),
        FACTOR_ONE
    );
    // And a simulated climb is slower than flat ground.
    let ramp = |k: i32| {
        let mut w = GridWorld::flat(64, 8, 0).unwrap();
        w.set_heights(|x, _| x as i32 * k);
        let mut sim = Sim::new(w, vec![class], Config::default()).unwrap();
        let u = sim.add_unit(TANK, centre(2, 4, ONE), 2 * UNIT).unwrap();
        sim.order(u, Goal::Point(centre(60, 4, ONE))).unwrap();
        for _ in 0..60 {
            sim.step();
        }
        sim.unit(u).unwrap().pos.x
    };
    let flat = ramp(0);
    let steep = ramp(8);
    assert!(
        steep < flat * 3 / 4 && steep > flat / 4,
        "flat {flat} steep {steep}"
    );
}
