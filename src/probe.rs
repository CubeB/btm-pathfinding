//! The live standability test shared by the search, the first pass and the
//! blocked-goal ring scan (SPEC 0 "Standable", 1.6).

use crate::classes::ClassTable;
use crate::error::Error;
use crate::geom::{allowed_moves, Cell, CellRect, Footprint};
use crate::terrain::ClassGrid;
use crate::world::{Mobility, UnitId, World};

pub(crate) struct Probe<'a, W: ?Sized> {
    world: &'a W,
    grid: Option<&'a ClassGrid>,
    pub(crate) fp: Footprint,
    own: CellRect,
    unit: UnitId,
    vw: i32,
    vh: i32,
    wader: bool,
}

impl<'a, W: World + ?Sized> Probe<'a, W> {
    pub(crate) fn new(
        world: &'a W,
        table: &'a ClassTable,
        unit: UnitId,
        mobility: Mobility,
        own: CellRect,
    ) -> Result<Self, Error> {
        let (fp, class) = table.resolve(mobility)?;
        let (vw, vh) = table.vertex_extent(fp);
        let grid = class.and_then(|c| table.grid(c));
        let wader = class
            .and_then(|c| table.class(c))
            .is_some_and(|c| c.is_wader());
        Ok(Self {
            world,
            grid,
            fp,
            own,
            unit,
            vw,
            vh,
            wader,
        })
    }

    pub(crate) fn in_range(&self, v: Cell) -> bool {
        v.x >= 0 && v.y >= 0 && v.x < self.vw && v.y < self.vh
    }

    /// Terrain rules and live obstacles. Cells the unit occupied when the search
    /// began are exempt from obstacles (SPEC 1.6).
    pub(crate) fn standable(&self, v: Cell) -> bool {
        if !self.in_range(v) {
            return false;
        }
        if let Some(g) = self.grid {
            if !g.standable(v) {
                return false;
            }
        }
        for cy in v.y..v.y + self.fp.h {
            for cx in v.x..v.x + self.fp.w {
                let c = Cell::new(cx, cy);
                if self.own.contains(c) {
                    continue;
                }
                if self.grid.is_none() && self.world.lava(cx as u32, cy as u32) {
                    return false;
                }
                if self.world.cell_blocked(cx, cy, self.unit) {
                    return false;
                }
            }
        }
        true
    }

    /// Terrain factor of the cell stepped into: 1, +1 rough, +1 wading (SPEC 1.2).
    pub(crate) fn factor(&self, v: Cell) -> u64 {
        match self.grid {
            None => 1,
            Some(g) => 1 + u64::from(g.rough(v)) + u64::from(self.wader && g.wet(v)),
        }
    }

    pub(crate) fn moves(&self, v: Cell) -> [bool; 8] {
        allowed_moves(v, |c| self.standable(c))
    }
}
