//! Typed errors.

use std::fmt;

use crate::world::{ClassId, UnitId};

/// Everything that can go wrong in this crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Error {
    /// The map has no cells.
    EmptyMap,
    /// The map is larger than [`crate::terrain::MAX_MAP_SIDE`] cells on a side.
    MapTooLarge {
        /// Columns given.
        cols: u32,
        /// Rows given.
        rows: u32,
    },
    /// A footprint side is outside 1 ..= [`crate::geom::Footprint::MAX_SIDE`].
    BadFootprint,
    /// The footprint does not fit on the map at all.
    FootprintExceedsMap,
    /// No movement class with this id was registered.
    UnknownClass(ClassId),
    /// The host does not know this unit.
    UnknownUnit(UnitId),
    /// A rectangle goal with no cells.
    EmptyRect,
    /// The request queue is at its configured capacity.
    QueueFull,
    /// A fixed-capacity table (units, buildings) is full.
    Capacity,
    /// A search was resumed with scratch space that another search has since used,
    /// or with scratch space of a different map size.
    StaleSearch,
    /// A search was resumed after it finished.
    SearchFinished,
    /// A start vertex outside the map.
    OutOfBounds,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyMap => write!(f, "the map has no cells"),
            Self::MapTooLarge { cols, rows } => {
                write!(f, "map of {cols}x{rows} cells is too large")
            }
            Self::BadFootprint => write!(f, "footprint side out of range"),
            Self::FootprintExceedsMap => write!(f, "footprint does not fit on the map"),
            Self::UnknownClass(c) => write!(f, "unknown movement class {}", c.0),
            Self::UnknownUnit(u) => write!(f, "unknown unit {}", u.0),
            Self::EmptyRect => write!(f, "rectangle goal has no cells"),
            Self::QueueFull => write!(f, "path request queue is full"),
            Self::Capacity => write!(f, "fixed-capacity table is full"),
            Self::StaleSearch => write!(f, "search scratch space was reused"),
            Self::SearchFinished => write!(f, "search already finished"),
            Self::OutOfBounds => write!(f, "vertex outside the map"),
        }
    }
}

impl std::error::Error for Error {}
