//! Route cost: a fixed-point length and a turn count, compared in that order
//! (SPEC 1.2, 1.3).

/// Length of an orthogonal step (1.0 in fixed point, 5 decimal places).
pub const ORTHO: u64 = 100_000;
/// Length of a diagonal step (1.41421).
pub const DIAG: u64 = 141_421;

/// A route cost. The derived ordering compares length first, then turns.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Cost {
    /// Length in units of [`ORTHO`] per cell.
    pub len: u64,
    /// Sum of heading changes, in 45-degree units.
    pub turns: u32,
}

impl Cost {
    /// No cost.
    pub const ZERO: Self = Self { len: 0, turns: 0 };
    /// Larger than any real cost.
    pub const MAX: Self = Self {
        len: u64::MAX,
        turns: u32::MAX,
    };

    /// Saturating sum.
    pub fn plus(self, o: Self) -> Self {
        Self {
            len: self.len.saturating_add(o.len),
            turns: self.turns.saturating_add(o.turns),
        }
    }
}

/// Octile estimate for coordinate differences `dx`, `dy` (SPEC 1.3): length of
/// `|dx - dy|` orthogonal plus `min(dx, dy)` diagonal steps, and an estimated
/// turn count of 1 when both kinds of step remain.
pub fn octile_estimate(dx: u64, dy: u64) -> Cost {
    let (mx, mn) = if dx > dy { (dx, dy) } else { (dy, dx) };
    let orth = mx - mn;
    Cost {
        len: ORTHO
            .saturating_mul(orth)
            .saturating_add(DIAG.saturating_mul(mn)),
        turns: u32::from(orth > 0 && mn > 0),
    }
}
