//! Routes built from search paths (SPEC 1.9).

use crate::geom::{block_centre, clamp_i32, dir_between, Cell, Footprint, WorldPos, CELL, UNIT};
use crate::terrain::Terrain;

/// A world-space waypoint at ground height.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Waypoint {
    /// Horizontal position.
    pub pos: WorldPos,
    /// Ground height in fixed point ([`UNIT`] steps per height unit).
    pub z: i32,
}

/// A route for a unit to walk.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Route {
    /// At least two waypoints: the start, the corners, the end.
    pub waypoints: Vec<Waypoint>,
    /// Every vertex of the path, start first (one entry for a one-vertex path).
    pub path: Vec<Cell>,
    /// The goal could not be reached; the route ends at the closest vertex.
    pub unreachable: bool,
    /// The goal was occupied and relaxed to the nearest standable ring (SPEC 5).
    pub goal_relaxed: bool,
    /// The last waypoint is the exact destination point.
    pub exact: bool,
}

impl Route {
    /// The reachable destination of an unreachable route: its last waypoint.
    pub fn reachable_destination(&self) -> Option<Waypoint> {
        if self.unreachable {
            self.waypoints.last().copied()
        } else {
            None
        }
    }
}

/// Ground height at `pos` in fixed point, bilinear over the corner samples,
/// clamped to the map.
pub fn ground_height<T: Terrain + ?Sized>(t: &T, pos: WorldPos) -> i32 {
    let (cols, rows) = t.cells();
    if cols == 0 || rows == 0 {
        return 0;
    }
    let c = i128::from(CELL);
    let px = i128::from(pos.x).clamp(0, i128::from(cols) * c);
    let py = i128::from(pos.y).clamp(0, i128::from(rows) * c);
    let cx = (px / c).min(i128::from(cols) - 1);
    let cy = (py / c).min(i128::from(rows) - 1);
    let fx = px - cx * c;
    let fy = py - cy * c;
    let (x, y) = (cx as u32, cy as u32);
    let h00 = i128::from(t.sample(x, y));
    let h10 = i128::from(t.sample(x + 1, y));
    let h01 = i128::from(t.sample(x, y + 1));
    let h11 = i128::from(t.sample(x + 1, y + 1));
    let sum = h00 * (c - fx) * (c - fy) + h10 * fx * (c - fy) + h01 * (c - fx) * fy + h11 * fx * fy;
    let v = sum * i128::from(UNIT) / (c * c);
    clamp_i32(v.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64)
}

pub(crate) fn waypoint<T: Terrain + ?Sized>(t: &T, pos: WorldPos) -> Waypoint {
    Waypoint {
        pos,
        z: ground_height(t, pos),
    }
}

/// Build a route from a path (SPEC 1.9): the start vertex, the corners, and the
/// end, as footprint-block centres; `exact` replaces the end by that point. A
/// one-vertex path gives the unit's position then `exact` or its position again.
pub(crate) fn build_route<T: Terrain + ?Sized>(
    t: &T,
    path: &[Cell],
    fp: Footprint,
    unit_pos: WorldPos,
    exact: Option<WorldPos>,
    unreachable: bool,
    goal_relaxed: bool,
) -> Route {
    let mut waypoints = Vec::new();
    if path.len() <= 1 {
        waypoints.push(waypoint(t, unit_pos));
        waypoints.push(waypoint(t, exact.unwrap_or(unit_pos)));
    } else {
        waypoints.push(waypoint(t, block_centre(path[0], fp)));
        for i in 1..path.len() - 1 {
            if dir_between(path[i - 1], path[i]) != dir_between(path[i], path[i + 1]) {
                waypoints.push(waypoint(t, block_centre(path[i], fp)));
            }
        }
        let last = path[path.len() - 1];
        waypoints.push(waypoint(t, exact.unwrap_or_else(|| block_centre(last, fp))));
    }
    Route {
        waypoints,
        path: path.to_vec(),
        unreachable,
        goal_relaxed,
        exact: exact.is_some(),
    }
}
