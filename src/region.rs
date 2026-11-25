//! Connected terrain regions for instant "unreachable" answers (SPEC 4).

use crate::geom::{allowed_moves, Cell};
use crate::terrain::ClassGrid;

/// Region labels of one class's terrain grid: 0 = not standable, 1, 2, ... for
/// each connected set, eight-way with the no-squeeze rule. Terrain only.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegionGrid {
    vw: i32,
    vh: i32,
    labels: Vec<u32>,
    count: u32,
}

impl RegionGrid {
    /// Label `grid` by flood fill, rows top to bottom and cells left to right.
    pub fn label(grid: &ClassGrid) -> Self {
        let (vw, vh) = grid.extent();
        let mut labels = vec![0u32; (vw.max(0) as usize) * (vh.max(0) as usize)];
        let mut count = 0u32;
        let mut stack: Vec<Cell> = Vec::new();
        let idx = |c: Cell| c.y as usize * vw as usize + c.x as usize;
        for y in 0..vh {
            for x in 0..vw {
                let c = Cell::new(x, y);
                if !grid.standable(c) || labels[idx(c)] != 0 {
                    continue;
                }
                count += 1;
                labels[idx(c)] = count;
                stack.push(c);
                while let Some(v) = stack.pop() {
                    let moves = allowed_moves(v, |n| grid.standable(n));
                    for d in 0..8u8 {
                        if !moves[usize::from(d)] {
                            continue;
                        }
                        let n = v.step(d);
                        let i = idx(n);
                        if labels[i] == 0 {
                            labels[i] = count;
                            stack.push(n);
                        }
                    }
                }
            }
        }
        Self {
            vw,
            vh,
            labels,
            count,
        }
    }

    /// The region of `v`, or `None` off the grid.
    pub fn region(&self, v: Cell) -> Option<u32> {
        if v.x < 0 || v.y < 0 || v.x >= self.vw || v.y >= self.vh {
            return None;
        }
        self.labels
            .get(v.y as usize * self.vw as usize + v.x as usize)
            .copied()
    }

    /// Number of regions.
    pub fn count(&self) -> u32 {
        self.count
    }
}
