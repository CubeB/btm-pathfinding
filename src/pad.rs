//! Factory pads clear (SPEC 9) and builders giving up on unreachable sites (SPEC 8).

use crate::geom::{CellRect, Footprint};
use crate::route::Route;
use crate::world::{Goal, UnitId};

/// Ticks between attempts of a factory whose pad is blocked.
pub const PAD_RETRY_TICKS: u64 = 15;

/// The rectangle a pad-clearing unit must get clear of: the pad expanded by
/// `3 x footprint - 4` cells (at least 0) on each side, per axis.
pub fn pad_clear_rect(pad: CellRect, fp: Footprint) -> CellRect {
    pad.expand((3 * fp.w - 4).max(0), (3 * fp.h - 4).max(0))
}

/// A mobile unit near a pad.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadOccupant {
    /// The unit.
    pub unit: UnitId,
    /// Its footprint size.
    pub footprint: Footprint,
    /// The cells it occupies.
    pub cells: CellRect,
    /// Whether it has no orders at all.
    pub idle: bool,
}

/// The sweep (triggers A and B): every idle unit occupying any cell of `pad`
/// gets a clear-the-pad order, a rectangle goal. Units with orders are left alone.
pub fn sweep<I: IntoIterator<Item = PadOccupant>>(
    pad: CellRect,
    occupants: I,
) -> Vec<(UnitId, Goal)> {
    occupants
        .into_iter()
        .filter(|o| o.idle && o.cells.overlaps(pad))
        .map(|o| (o.unit, Goal::Rect(pad_clear_rect(pad, o.footprint))))
        .collect()
}

/// A blocked factory's retry timer: every 15 ticks, no cap, no message.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PadRetry {
    next: Option<u64>,
}

impl PadRetry {
    /// Whether to try placing now.
    pub fn due(&self, tick: u64) -> bool {
        self.next.is_none_or(|n| tick >= n)
    }
    /// The attempt at `tick` found the pad blocked.
    pub fn blocked(&mut self, tick: u64) {
        self.next = Some(tick.saturating_add(PAD_RETRY_TICKS));
    }
    /// The pad was free.
    pub fn clear(&mut self) {
        self.next = None;
    }
}

/// What a builder does with the route to its site.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SiteDecision {
    /// Walk there and build.
    Proceed,
    /// Give the order up: "can't reach the site".
    GiveUp,
}

/// A builder ordered to an unreachable site gives up rather than build across
/// the obstacle (SPEC 8). A site that is merely occupied is waited out by the host.
pub fn site_decision(route: &Route) -> SiteDecision {
    if route.unreachable {
        SiteDecision::GiveUp
    } else {
        SiteDecision::Proceed
    }
}
