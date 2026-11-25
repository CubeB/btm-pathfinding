//! What the pathfinder needs from the host: obstacles and units.
//!
//! Terrain is the [`crate::terrain::Terrain`] trait. A host world implements all
//! three traits and so gets [`World`] for free.

use crate::geom::{CellRect, Footprint, WorldPos};
use crate::terrain::Terrain;

/// A unit, as the host identifies it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct UnitId(pub u32);

/// A registered movement class (index into the class list given to the pathfinder).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ClassId(pub u16);

/// How a unit moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mobility {
    /// A registered movement class: footprint and terrain rules come from it.
    Class(ClassId),
    /// An inline, unnamed class: only the footprint is known, terrain is not
    /// consulted and only obstacles block (SPEC 1.6).
    Inline(Footprint),
}

/// A destination (SPEC 1.8).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Goal {
    /// Go to this point.
    Point(WorldPos),
    /// Get next to this rectangle (touching its outside, corners included).
    Rect(CellRect),
}

/// What the pathfinder reads about a unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UnitView {
    /// Current position (the centre of its footprint).
    pub pos: WorldPos,
    /// Movement class or inline footprint.
    pub mobility: Mobility,
    /// Whether the unit is still moving (has a move order). Requests of units
    /// that are not moving are dropped and their searches abandoned (SPEC 2.1).
    pub moving: bool,
    /// The destination the unit's outstanding request is for. `Some(d)` with `d`
    /// different from the request's goal abandons a suspended search; `None`
    /// means the host does not track it and no check is made.
    pub destination: Option<Goal>,
}

/// Live obstacle state, read during searches (SPEC 1.6).
pub trait Obstacles {
    /// Whether cell (`x`, `y`) is blocked for `searcher`: by a building cell that
    /// is not an open yard cell, a blocking feature, or a mobile unit other than
    /// `searcher`. Off-map cells may return anything (they are never standable).
    /// Lava is read from [`Terrain::lava`] and need not be repeated here.
    fn cell_blocked(&self, x: i32, y: i32, searcher: UnitId) -> bool;
}

/// Units, by id.
pub trait Units {
    /// The unit, or `None` if it no longer exists.
    fn unit(&self, id: UnitId) -> Option<UnitView>;
}

/// Everything a search reads.
pub trait World: Terrain + Obstacles + Units {}

impl<T: Terrain + Obstacles + Units + ?Sized> World for T {}
