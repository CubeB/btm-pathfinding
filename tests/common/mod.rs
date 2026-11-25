#![allow(dead_code, clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use btm_pathfinding::geom::{allowed_moves, block_centre};
use btm_pathfinding::grid::GridWorld;
use btm_pathfinding::{
    Cell, CellRect, ClassGrid, Config, Delivery, Event, Footprint, Pathfinder, TickOutput, WorldPos,
};

pub fn centre(x: i32, y: i32, fp: Footprint) -> WorldPos {
    block_centre(Cell::new(x, y), fp)
}

/// Tick until the queue is empty; collect everything.
pub fn drain(
    pf: &mut Pathfinder,
    w: &GridWorld,
    max_ticks: u32,
) -> (Vec<Delivery>, Vec<Event>, Vec<TickOutput>) {
    let mut ds = Vec::new();
    let mut es = Vec::new();
    let mut outs = Vec::new();
    for _ in 0..max_ticks {
        if pf.queue_len() == 0 && pf.debt() == 0 {
            break;
        }
        let o = pf.tick(w);
        ds.extend(o.deliveries.iter().cloned());
        es.extend(o.events.iter().copied());
        outs.push(o);
    }
    (ds, es, outs)
}

pub fn cfg_budget(b: u32) -> Config {
    Config {
        budget_per_tick: b,
        ..Config::default()
    }
}

/// Standability as the reference sees it in a static world without other
/// units: class terrain, no features or buildings, own cells exempt.
pub fn ref_standable(
    w: &GridWorld,
    grid: &ClassGrid,
    fp: Footprint,
    own: CellRect,
    v: Cell,
) -> bool {
    if !grid.standable(v) {
        return false;
    }
    for y in v.y..v.y + fp.h {
        for x in v.x..v.x + fp.w {
            let c = Cell::new(x, y);
            if !own.contains(c) && w.static_blocked(c) {
                return false;
            }
        }
    }
    true
}

/// Full breadth-first reachability from `start` with the no-squeeze rule.
pub fn reachable(w: &GridWorld, grid: &ClassGrid, fp: Footprint, start: Cell) -> Vec<bool> {
    let (vw, vh) = grid.extent();
    let own = CellRect::block(start, fp);
    let mut seen = vec![false; (vw * vh) as usize];
    let idx = |c: Cell| (c.y * vw + c.x) as usize;
    let mut q = std::collections::VecDeque::new();
    seen[idx(start)] = true;
    q.push_back(start);
    while let Some(v) = q.pop_front() {
        let m = allowed_moves(v, |c| ref_standable(w, grid, fp, own, c));
        for d in 0..8u8 {
            if m[d as usize] {
                let n = v.step(d);
                if !seen[idx(n)] {
                    seen[idx(n)] = true;
                    q.push_back(n);
                }
            }
        }
    }
    seen
}

/// Check that a path only steps between adjacent standable vertices without
/// squeezing corners.
pub fn check_path(w: &GridWorld, grid: &ClassGrid, fp: Footprint, path: &[Cell]) {
    let own = CellRect::block(path[0], fp);
    for pair in path.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        assert!(
            dx.abs() <= 1 && dy.abs() <= 1 && (dx, dy) != (0, 0),
            "non-adjacent step {a:?}->{b:?}"
        );
        assert!(
            ref_standable(w, grid, fp, own, b),
            "step into unstandable {b:?}"
        );
        if dx != 0 && dy != 0 {
            assert!(
                ref_standable(w, grid, fp, own, Cell::new(a.x + dx, a.y)),
                "squeeze at {a:?}->{b:?}"
            );
            assert!(
                ref_standable(w, grid, fp, own, Cell::new(a.x, a.y + dy)),
                "squeeze at {a:?}->{b:?}"
            );
        }
    }
}
