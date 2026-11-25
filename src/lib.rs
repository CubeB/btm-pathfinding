//! Better pathfinding for Byte Tactics. The behaviour is specified in `SPEC.md`.
//!
//! The engine is abstract: a host implements [`Terrain`], [`Obstacles`] and
//! [`Units`] (together [`World`]), registers its [`MoveClass`]es, and drives a
//! [`Pathfinder`] once per tick. [`Nav`] holds per-unit navigation state.
//! [`grid::GridWorld`] and [`sim::Sim`] are a reference host for tests and
//! benchmarks.

#![forbid(unsafe_code)]

pub mod classes;
pub mod cost;
pub mod error;
pub mod firstpass;
pub mod geom;
mod probe;
pub mod region;
pub mod route;
pub mod search;
pub mod service;
pub mod terrain;
pub mod world;

pub use classes::ClassTable;
pub use cost::Cost;
pub use error::Error;
pub use geom::{Cell, CellRect, Footprint, WorldPos, CELL, UNIT};
pub use route::{Route, Waypoint};
pub use search::{Scratch, Search, SearchGoal, SearchOutcome, SearchSpec, SearchStatus};
pub use service::{
    Config, Delivery, Event, Pathfinder, PrecheckCharge, Resolution, Stats, TickOutput,
};
pub use terrain::{ClassGrid, MoveClass, Terrain};
pub use world::{ClassId, Goal, Mobility, Obstacles, UnitId, UnitView, Units, World};
