//! Stand-off points (SPEC 7), pad clearing (SPEC 9) and rectangle arrival (6.1).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use btm_pathfinding::geom::{vertex_of, UNIT};
use btm_pathfinding::grid::GridWorld;
use btm_pathfinding::nav::rect_arrived;
use btm_pathfinding::pad::{pad_clear_rect, sweep, PadOccupant, PadRetry};
use btm_pathfinding::sim::Sim;
use btm_pathfinding::standoff::{standoff_distance, Chase, StandoffCache, Target};
use btm_pathfinding::{CellRect, ClassId, Config, Footprint, Goal, MoveClass, UnitId, WorldPos};
use common::centre;

#[test]
fn standoff_distance_and_fan_out() {
    // Short weapon against a big target: outside the footprint.
    assert_eq!(standoff_distance(40 * UNIT, 10 * UNIT), 48 * UNIT);
    // Long weapon: just inside range.
    assert_eq!(standoff_distance(8 * UNIT, 200 * UNIT), 192 * UNIT);
    assert_eq!(standoff_distance(0, 5 * UNIT), 8 * UNIT);

    let target = Target {
        id: UnitId(9),
        pos: WorldPos::from_units(500, 500),
        z: 7,
        half_extent: 16 * UNIT,
    };
    let range = 100 * UNIT;
    let mut points = Vec::new();
    for a in [
        WorldPos::from_units(900, 500),
        WorldPos::from_units(100, 500),
        WorldPos::from_units(500, 900),
    ] {
        let mut cache = StandoffCache::default();
        match cache.chase(a, &target, range) {
            Chase::Approach { point, z } => {
                assert_eq!(z, 7);
                assert!((point.dist(target.pos) - i64::from(92 * UNIT)).abs() <= 1);
                points.push(point);
            }
            Chase::InRange => panic!("out of range"),
        }
    }
    assert_eq!(points[0], WorldPos::from_units(592, 500));
    assert_ne!(points[0], points[1]);
    assert_ne!(points[1], points[2]);
    // Coincident attacker: +x.
    assert_eq!(
        btm_pathfinding::standoff::standoff_point(target.pos, target.pos, 24 * UNIT),
        WorldPos::from_units(524, 500)
    );
    // In range: stop chasing.
    let mut cache = StandoffCache::default();
    assert_eq!(
        cache.chase(WorldPos::from_units(550, 500), &target, range),
        Chase::InRange
    );
}

#[test]
fn standoff_cache_band() {
    let mut t = Target {
        id: UnitId(1),
        pos: WorldPos::from_units(500, 500),
        z: 0,
        half_extent: 8 * UNIT,
    };
    let range = 50 * UNIT; // stand-off 42
    let mut cache = StandoffCache::default();
    let Chase::Approach { point: p0, .. } = cache.chase(WorldPos::from_units(800, 500), &t, range)
    else {
        panic!()
    };
    // The attacker moves round; a stationary target keeps the same point.
    let Chase::Approach { point: p1, .. } = cache.chase(WorldPos::from_units(500, 800), &t, range)
    else {
        panic!()
    };
    assert_eq!(p0, p1);
    // Target drifts 5 units: still inside the +/- 8 band.
    t.pos = WorldPos::from_units(505, 500);
    let Chase::Approach { point: p2, .. } = cache.chase(WorldPos::from_units(500, 800), &t, range)
    else {
        panic!()
    };
    assert_eq!(p2, p0);
    // Target moves 20 units: new bearing from the attacker's current position.
    t.pos = WorldPos::from_units(520, 500);
    let Chase::Approach { point: p3, .. } = cache.chase(WorldPos::from_units(520, 800), &t, range)
    else {
        panic!()
    };
    assert_eq!(p3, WorldPos::from_units(520, 542));
    // A different target never reuses the cache.
    let other = Target { id: UnitId(2), ..t };
    let Chase::Approach { point: p4, .. } =
        cache.chase(WorldPos::from_units(900, 500), &other, range)
    else {
        panic!()
    };
    assert_eq!(p4, WorldPos::from_units(562, 500));
}

#[test]
fn pad_rect_and_sweep() {
    let pad = CellRect::new(10, 10, 14, 14);
    assert_eq!(pad_clear_rect(pad, Footprint::square(1)), pad);
    assert_eq!(pad_clear_rect(pad, Footprint::square(2)), pad.expand(2, 2));
    assert_eq!(pad_clear_rect(pad, Footprint::new(3, 1)), pad.expand(5, 0));
    let occ = [
        PadOccupant {
            unit: UnitId(1),
            footprint: Footprint::square(1),
            cells: CellRect::from_size(11, 11, 1, 1),
            idle: true,
        },
        PadOccupant {
            unit: UnitId(2),
            footprint: Footprint::square(1),
            cells: CellRect::from_size(12, 12, 1, 1),
            idle: false,
        },
        PadOccupant {
            unit: UnitId(3),
            footprint: Footprint::square(2),
            cells: CellRect::from_size(13, 9, 2, 2),
            idle: true,
        },
        PadOccupant {
            unit: UnitId(4),
            footprint: Footprint::square(1),
            cells: CellRect::from_size(20, 20, 1, 1),
            idle: true,
        },
    ];
    let orders = sweep(pad, occ);
    assert_eq!(
        orders,
        vec![
            (UnitId(1), Goal::Rect(pad)),
            (UnitId(3), Goal::Rect(pad.expand(2, 2)))
        ]
    );
    let mut retry = PadRetry::default();
    assert!(retry.due(0));
    retry.blocked(0);
    assert!(!retry.due(14));
    assert!(retry.due(15));
}

#[test]
fn parked_unit_clears_the_pad() {
    let two = Footprint::square(2);
    let w = GridWorld::flat(48, 48, 0).unwrap();
    let mut sim = Sim::new(w, vec![MoveClass::tank(two, 10, 0)], Config::default()).unwrap();
    let pad = CellRect::new(20, 20, 24, 24);
    let u = sim
        .add_unit(ClassId(0), centre(21, 21, two), 2 * UNIT)
        .unwrap();
    let occ = PadOccupant {
        unit: u,
        footprint: two,
        cells: CellRect::block(vertex_of(sim.unit(u).unwrap().pos, two), two),
        idle: sim.unit(u).unwrap().order.is_none(),
    };
    let orders = sweep(pad, [occ]);
    assert_eq!(orders.len(), 1);
    sim.order(orders[0].0, orders[0].1).unwrap();
    for _ in 0..300 {
        sim.step();
    }
    let unit = sim.unit(u).unwrap();
    assert_eq!(unit.completed, 1);
    assert!(unit.order.is_none(), "order removed on arrival");
    let cells = CellRect::block(vertex_of(unit.pos, two), two);
    assert!(
        !cells.overlaps(pad_clear_rect(pad, two)),
        "clear of the expanded pad"
    );
}

#[test]
fn rectangle_arrival_band() {
    let one = Footprint::square(1);
    let r = CellRect::new(10, 10, 12, 12);
    // Enlarged rect spans x 152 .. 200 world units.
    assert!(rect_arrived(
        WorldPos::from_units(150, 176),
        one,
        r,
        8 * UNIT
    ));
    assert!(rect_arrived(
        WorldPos::from_units(145, 176),
        one,
        r,
        8 * UNIT
    ));
    assert!(!rect_arrived(
        WorldPos::from_units(140, 176),
        one,
        r,
        8 * UNIT
    ));
    assert!(
        !rect_arrived(WorldPos::from_units(176, 176), one, r, 8 * UNIT),
        "deep inside"
    );
}
