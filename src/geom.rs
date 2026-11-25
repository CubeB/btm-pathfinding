//! Fixed-point geometry shared by every module.
//!
//! World positions are fixed point: one world unit is [`UNIT`] (256) steps and a
//! map cell is [`CELL_UNITS`] (16) world units, so [`CELL`] steps. Everything that
//! can affect a route is integer maths, so every peer computes the same answer.

/// Fixed-point steps per world unit.
pub const UNIT: i32 = 256;
/// World units per map cell (SPEC section 0).
pub const CELL_UNITS: i32 = 16;
/// One map cell in fixed-point steps.
pub const CELL: i32 = UNIT * CELL_UNITS;
/// Simulation rate (SPEC preamble).
pub const TICKS_PER_SECOND: u32 = 30;

pub(crate) fn clamp_i32(v: i64) -> i32 {
    v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// Integer square root of a non-negative value (0 for negatives).
pub fn isqrt_i64(v: i64) -> i64 {
    if v <= 0 {
        0
    } else {
        (v as u64).isqrt() as i64
    }
}

/// A horizontal world position in fixed point ([`UNIT`] steps per world unit).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct WorldPos {
    /// East-west coordinate.
    pub x: i32,
    /// North-south coordinate (grows towards higher rows).
    pub y: i32,
}

impl WorldPos {
    /// A position from raw fixed-point coordinates.
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// A position from whole world units.
    pub const fn from_units(x: i32, y: i32) -> Self {
        Self {
            x: x.saturating_mul(UNIT),
            y: y.saturating_mul(UNIT),
        }
    }

    /// Squared horizontal distance in fixed-point steps (saturating).
    pub fn dist2(self, other: Self) -> i64 {
        let dx = i64::from(self.x) - i64::from(other.x);
        let dy = i64::from(self.y) - i64::from(other.y);
        dx.saturating_mul(dx).saturating_add(dy.saturating_mul(dy))
    }

    /// Horizontal distance in fixed-point steps, rounded down.
    pub fn dist(self, other: Self) -> i64 {
        isqrt_i64(self.dist2(other))
    }
}

/// A cell (or search vertex) on the map grid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Cell {
    /// Column.
    pub x: i32,
    /// Row.
    pub y: i32,
}

impl Cell {
    /// A cell from its column and row.
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// The neighbouring cell in compass direction `dir` (see [`DIRS`]).
    pub fn step(self, dir: u8) -> Self {
        let (dx, dy) = DIRS[usize::from(dir & 7)];
        Self::new(self.x.saturating_add(dx), self.y.saturating_add(dy))
    }
}

/// A unit's size in cells (width x depth).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Footprint {
    /// Width in cells.
    pub w: i32,
    /// Depth in cells.
    pub h: i32,
}

impl Footprint {
    /// Largest supported footprint side, in cells.
    pub const MAX_SIDE: i32 = 64;

    /// A `w` x `h` footprint.
    pub const fn new(w: i32, h: i32) -> Self {
        Self { w, h }
    }

    /// An `n` x `n` footprint.
    pub const fn square(n: i32) -> Self {
        Self { w: n, h: n }
    }

    /// Whether both sides are between 1 and [`Footprint::MAX_SIDE`].
    pub fn is_valid(self) -> bool {
        (1..=Self::MAX_SIDE).contains(&self.w) && (1..=Self::MAX_SIDE).contains(&self.h)
    }
}

/// A half-open rectangle of cells: `x0 <= x < x1`, `y0 <= y < y1`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CellRect {
    /// First column.
    pub x0: i32,
    /// First row.
    pub y0: i32,
    /// One past the last column.
    pub x1: i32,
    /// One past the last row.
    pub y1: i32,
}

impl CellRect {
    /// A rectangle from its corners (half open).
    pub const fn new(x0: i32, y0: i32, x1: i32, y1: i32) -> Self {
        Self { x0, y0, x1, y1 }
    }

    /// A rectangle from its top-left cell and size.
    pub const fn from_size(x: i32, y: i32, w: i32, h: i32) -> Self {
        Self {
            x0: x,
            y0: y,
            x1: x.saturating_add(w),
            y1: y.saturating_add(h),
        }
    }

    /// The footprint block whose top-left cell is `v`.
    pub fn block(v: Cell, fp: Footprint) -> Self {
        Self::from_size(v.x, v.y, fp.w, fp.h)
    }

    /// Whether the rectangle has no cells.
    pub fn is_empty(self) -> bool {
        self.x1 <= self.x0 || self.y1 <= self.y0
    }

    /// Whether `c` lies inside.
    pub fn contains(self, c: Cell) -> bool {
        c.x >= self.x0 && c.x < self.x1 && c.y >= self.y0 && c.y < self.y1
    }

    /// Whether the two rectangles share at least one cell.
    pub fn overlaps(self, o: Self) -> bool {
        self.x0 < o.x1 && o.x0 < self.x1 && self.y0 < o.y1 && o.y0 < self.y1
    }

    /// The rectangle grown by `dx` cells left and right and `dy` cells up and down.
    pub fn expand(self, dx: i32, dy: i32) -> Self {
        Self {
            x0: self.x0.saturating_sub(dx),
            y0: self.y0.saturating_sub(dy),
            x1: self.x1.saturating_add(dx),
            y1: self.y1.saturating_add(dy),
        }
    }

    /// The world-space centre of the rectangle.
    pub fn centre(self) -> WorldPos {
        let cx = (i64::from(self.x0) + i64::from(self.x1)) * i64::from(CELL) / 2;
        let cy = (i64::from(self.y0) + i64::from(self.y1)) * i64::from(CELL) / 2;
        WorldPos::new(clamp_i32(cx), clamp_i32(cy))
    }
}

/// The search vertex of a unit at `pos`: the top-left cell of its footprint
/// block, i.e. the cell nearest to `pos - footprint * 8 world units` (SPEC 0).
/// Halves round up.
pub fn vertex_of(pos: WorldPos, fp: Footprint) -> Cell {
    let half = i64::from(CELL / 2);
    let c = i64::from(CELL);
    let x = i64::from(pos.x) - i64::from(fp.w) * half;
    let y = i64::from(pos.y) - i64::from(fp.h) * half;
    Cell::new(
        clamp_i32((x + half).div_euclid(c)),
        clamp_i32((y + half).div_euclid(c)),
    )
}

/// The world-space centre of the footprint block whose top-left cell is `v`.
pub fn block_centre(v: Cell, fp: Footprint) -> WorldPos {
    let c = i64::from(CELL);
    let x = i64::from(v.x) * c + i64::from(fp.w) * c / 2;
    let y = i64::from(v.y) * c + i64::from(fp.h) * c / 2;
    WorldPos::new(clamp_i32(x), clamp_i32(y))
}

/// The eight compass steps, clockwise from east on a y-down grid:
/// E, SE, S, SW, W, NW, N, NE. Odd indices are diagonal.
pub const DIRS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

/// Whether direction `d` is diagonal.
pub fn is_diagonal(d: u8) -> bool {
    d & 1 == 1
}

/// The size of a heading change in 45-degree units, 0 to 4 (SPEC 1.2).
pub fn turn_size(a: u8, b: u8) -> u32 {
    let d = (i32::from(a) - i32::from(b)).rem_euclid(8) as u32;
    d.min(8 - d)
}

/// The compass direction from `a` towards `b` (by sign), if they differ.
pub fn dir_between(a: Cell, b: Cell) -> Option<u8> {
    let s = ((b.x - a.x).signum(), (b.y - a.y).signum());
    DIRS.iter().position(|&d| d == s).map(|i| i as u8)
}

/// Octile score: 10 per orthogonal step plus 14 per diagonal step (SPEC 1.8).
pub fn octile_score(a: Cell, b: Cell) -> u32 {
    let dx = (i64::from(a.x) - i64::from(b.x)).unsigned_abs();
    let dy = (i64::from(a.y) - i64::from(b.y)).unsigned_abs();
    let (mx, mn) = if dx > dy { (dx, dy) } else { (dy, dx) };
    u32::try_from(10 * (mx - mn) + 14 * mn).unwrap_or(u32::MAX)
}

/// The eight moves out of `v` allowed by `stand`, with corner squeezing
/// forbidden: a diagonal needs both flanking orthogonal vertices (SPEC 1.1).
pub fn allowed_moves(v: Cell, mut stand: impl FnMut(Cell) -> bool) -> [bool; 8] {
    let mut m = [false; 8];
    for d in [0u8, 2, 4, 6] {
        m[usize::from(d)] = stand(v.step(d));
    }
    for d in [1u8, 3, 5, 7] {
        let a = m[usize::from(d - 1)];
        let b = m[usize::from((d + 1) & 7)];
        m[usize::from(d)] = a && b && stand(v.step(d));
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_rounding() {
        let p = WorldPos::new(3 * CELL + CELL / 2, 5 * CELL + CELL / 2);
        assert_eq!(vertex_of(p, Footprint::square(1)), Cell::new(3, 5));
        let p = WorldPos::new(4 * CELL, 4 * CELL);
        assert_eq!(vertex_of(p, Footprint::square(2)), Cell::new(3, 3));
        assert_eq!(block_centre(Cell::new(3, 3), Footprint::square(2)), p);
    }

    #[test]
    fn turns_and_scores() {
        assert_eq!(turn_size(0, 0), 0);
        assert_eq!(turn_size(0, 2), 2);
        assert_eq!(turn_size(0, 4), 4);
        assert_eq!(turn_size(7, 1), 2);
        assert_eq!(octile_score(Cell::new(0, 0), Cell::new(3, 1)), 34);
    }
}
