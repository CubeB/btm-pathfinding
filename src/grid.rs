//! A simple in-crate world for tests, benchmarks and as a reference host:
//! a height grid, buildings added and removed, features, and unit records with
//! a cell occupancy count. Implements [`crate::world::World`].

use crate::error::Error;
use crate::geom::{vertex_of, Cell, CellRect, Footprint, WorldPos};
use crate::terrain::{check_map, Terrain};
use crate::world::{Goal, Mobility, Obstacles, UnitId, UnitView, Units};

/// A placed building.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BuildingId(pub u32);

#[derive(Clone, Debug, PartialEq, Eq)]
struct Building {
    rect: CellRect,
    yard: Vec<Cell>,
}

/// What the world records about a unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UnitRecord {
    /// Position.
    pub pos: WorldPos,
    /// Movement class or inline footprint.
    pub mobility: Mobility,
    /// Footprint (must match the class's).
    pub footprint: Footprint,
    /// Whether it has a move order.
    pub moving: bool,
    /// Destination of its outstanding request.
    pub destination: Option<Goal>,
    /// Cells it occupies.
    pub cells: CellRect,
}

/// Maximum units in a [`GridWorld`].
pub const MAX_UNITS: usize = 65_535;

/// The in-crate grid world.
#[derive(Clone, Debug)]
pub struct GridWorld {
    cols: u32,
    rows: u32,
    samples: Vec<i32>,
    sea: i32,
    lava: Vec<bool>,
    statics: Vec<u16>,
    features: Vec<bool>,
    occ: Vec<u16>,
    buildings: Vec<Option<Building>>,
    units: Vec<Option<UnitRecord>>,
}

impl GridWorld {
    /// A flat map of `cols` x `rows` cells at `height`, sea level below it.
    pub fn flat(cols: u32, rows: u32, height: i32) -> Result<Self, Error> {
        struct Size(u32, u32);
        impl Terrain for Size {
            fn cells(&self) -> (u32, u32) {
                (self.0, self.1)
            }
            fn sample(&self, _: u32, _: u32) -> i32 {
                0
            }
            fn sea_level(&self) -> i32 {
                0
            }
        }
        check_map(&Size(cols, rows))?;
        let n = cols as usize * rows as usize;
        Ok(Self {
            cols,
            rows,
            samples: vec![height; (cols as usize + 1) * (rows as usize + 1)],
            sea: height.saturating_sub(1000),
            lava: vec![false; n],
            statics: vec![0; n],
            features: vec![false; n],
            occ: vec![0; n],
            buildings: Vec::new(),
            units: Vec::new(),
        })
    }

    fn cell_index(&self, x: i32, y: i32) -> Option<usize> {
        if x < 0 || y < 0 || x as u32 >= self.cols || y as u32 >= self.rows {
            None
        } else {
            Some(y as usize * self.cols as usize + x as usize)
        }
    }

    /// Set the height sample at corner (`x`, `y`) (ignored off the grid).
    pub fn set_sample(&mut self, x: u32, y: u32, h: i32) {
        if x <= self.cols && y <= self.rows {
            let i = y as usize * (self.cols as usize + 1) + x as usize;
            if let Some(s) = self.samples.get_mut(i) {
                *s = h;
            }
        }
    }

    /// Set every height sample from a function of its corner.
    pub fn set_heights(&mut self, mut f: impl FnMut(u32, u32) -> i32) {
        for y in 0..=self.rows {
            for x in 0..=self.cols {
                self.set_sample(x, y, f(x, y));
            }
        }
    }

    /// Set sea level.
    pub fn set_sea_level(&mut self, sea: i32) {
        self.sea = sea;
    }

    /// Mark a cell lava-excluded.
    pub fn set_lava(&mut self, c: Cell, lava: bool) {
        if let Some(i) = self.cell_index(c.x, c.y) {
            self.lava[i] = lava;
        }
    }

    /// Place or remove a blocking feature (rock, wall, wreck).
    pub fn set_feature(&mut self, c: Cell, blocking: bool) {
        if let Some(i) = self.cell_index(c.x, c.y) {
            self.features[i] = blocking;
        }
    }

    /// Fill a rectangle with blocking features.
    pub fn fill_features(&mut self, r: CellRect, blocking: bool) {
        for y in r.y0..r.y1 {
            for x in r.x0..r.x1 {
                self.set_feature(Cell::new(x, y), blocking);
            }
        }
    }

    /// Whether a cell has a blocking feature.
    pub fn feature(&self, c: Cell) -> bool {
        self.cell_index(c.x, c.y).is_some_and(|i| self.features[i])
    }

    fn mark_building(&mut self, b: &Building, add: bool) {
        for y in b.rect.y0..b.rect.y1 {
            for x in b.rect.x0..b.rect.x1 {
                if b.yard.contains(&Cell::new(x, y)) {
                    continue;
                }
                if let Some(i) = self.cell_index(x, y) {
                    self.statics[i] = if add {
                        self.statics[i].saturating_add(1)
                    } else {
                        self.statics[i].saturating_sub(1)
                    };
                }
            }
        }
    }

    /// Place a building over `rect`; `yard` cells inside it stay passable.
    pub fn add_building(&mut self, rect: CellRect, yard: &[Cell]) -> Result<BuildingId, Error> {
        if self.buildings.len() >= u32::MAX as usize {
            return Err(Error::Capacity);
        }
        let b = Building {
            rect,
            yard: yard.to_vec(),
        };
        self.mark_building(&b, true);
        self.buildings.push(Some(b));
        Ok(BuildingId(self.buildings.len() as u32 - 1))
    }

    /// Remove a building. Returns whether it existed.
    pub fn remove_building(&mut self, id: BuildingId) -> bool {
        let Some(b) = self.buildings.get_mut(id.0 as usize).and_then(Option::take) else {
            return false;
        };
        self.mark_building(&b, false);
        true
    }

    fn occupy(&mut self, r: CellRect, add: bool) {
        for y in r.y0..r.y1 {
            for x in r.x0..r.x1 {
                if let Some(i) = self.cell_index(x, y) {
                    self.occ[i] = if add {
                        self.occ[i].saturating_add(1)
                    } else {
                        self.occ[i].saturating_sub(1)
                    };
                }
            }
        }
    }

    /// Add a unit at `pos`.
    pub fn add_unit(
        &mut self,
        pos: WorldPos,
        mobility: Mobility,
        footprint: Footprint,
    ) -> Result<UnitId, Error> {
        if self.units.len() >= MAX_UNITS {
            return Err(Error::Capacity);
        }
        if !footprint.is_valid() {
            return Err(Error::BadFootprint);
        }
        let cells = CellRect::block(vertex_of(pos, footprint), footprint);
        self.occupy(cells, true);
        self.units.push(Some(UnitRecord {
            pos,
            mobility,
            footprint,
            moving: false,
            destination: None,
            cells,
        }));
        Ok(UnitId(self.units.len() as u32 - 1))
    }

    /// Move a unit.
    pub fn set_unit_pos(&mut self, id: UnitId, pos: WorldPos) -> Result<(), Error> {
        let rec = self.record(id)?;
        let cells = CellRect::block(vertex_of(pos, rec.footprint), rec.footprint);
        if cells != rec.cells {
            self.occupy(rec.cells, false);
            self.occupy(cells, true);
        }
        if let Some(Some(r)) = self.units.get_mut(id.0 as usize) {
            r.pos = pos;
            r.cells = cells;
        }
        Ok(())
    }

    /// Set whether a unit has a move order.
    pub fn set_unit_moving(&mut self, id: UnitId, moving: bool) -> Result<(), Error> {
        self.record_mut(id)?.moving = moving;
        Ok(())
    }

    /// Set the destination of a unit's outstanding request.
    pub fn set_unit_destination(&mut self, id: UnitId, d: Option<Goal>) -> Result<(), Error> {
        self.record_mut(id)?.destination = d;
        Ok(())
    }

    /// Remove a unit.
    pub fn remove_unit(&mut self, id: UnitId) -> bool {
        let Some(r) = self.units.get_mut(id.0 as usize).and_then(Option::take) else {
            return false;
        };
        self.occupy(r.cells, false);
        true
    }

    /// A unit's record.
    pub fn record(&self, id: UnitId) -> Result<UnitRecord, Error> {
        self.units
            .get(id.0 as usize)
            .copied()
            .flatten()
            .ok_or(Error::UnknownUnit(id))
    }

    fn record_mut(&mut self, id: UnitId) -> Result<&mut UnitRecord, Error> {
        self.units
            .get_mut(id.0 as usize)
            .and_then(Option::as_mut)
            .ok_or(Error::UnknownUnit(id))
    }

    /// Whether a cell is blocked by a building or feature (not units).
    pub fn static_blocked(&self, c: Cell) -> bool {
        self.cell_index(c.x, c.y)
            .is_none_or(|i| self.statics[i] > 0 || self.features[i])
    }

    /// Mobile units occupying a cell.
    pub fn occupancy(&self, c: Cell) -> u16 {
        self.cell_index(c.x, c.y).map_or(0, |i| self.occ[i])
    }
}

impl Terrain for GridWorld {
    fn cells(&self) -> (u32, u32) {
        (self.cols, self.rows)
    }
    fn sample(&self, x: u32, y: u32) -> i32 {
        let x = x.min(self.cols) as usize;
        let y = y.min(self.rows) as usize;
        self.samples
            .get(y * (self.cols as usize + 1) + x)
            .copied()
            .unwrap_or(0)
    }
    fn sea_level(&self) -> i32 {
        self.sea
    }
    fn lava(&self, x: u32, y: u32) -> bool {
        self.cell_index(x as i32, y as i32)
            .is_some_and(|i| self.lava[i])
    }
}

impl Obstacles for GridWorld {
    fn cell_blocked(&self, x: i32, y: i32, searcher: UnitId) -> bool {
        let Some(i) = self.cell_index(x, y) else {
            return true;
        };
        if self.statics[i] > 0 || self.features[i] {
            return true;
        }
        let own = self
            .units
            .get(searcher.0 as usize)
            .copied()
            .flatten()
            .is_some_and(|r| r.cells.contains(Cell::new(x, y)));
        self.occ[i] > u16::from(own)
    }
}

impl Units for GridWorld {
    fn unit(&self, id: UnitId) -> Option<UnitView> {
        self.units
            .get(id.0 as usize)
            .copied()
            .flatten()
            .map(|r| UnitView {
                pos: r.pos,
                mobility: r.mobility,
                moving: r.moving,
                destination: r.destination,
            })
    }
}
