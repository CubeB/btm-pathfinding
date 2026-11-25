//! The registered movement classes with their terrain grids and lazy regions.

use crate::error::Error;
use crate::geom::Footprint;
use crate::region::RegionGrid;
use crate::terrain::{check_map, ClassGrid, MoveClass, Terrain};
use crate::world::{ClassId, Mobility};

/// Movement classes, their terrain grids (built at map load) and region grids
/// (built the first time a class is asked about).
#[derive(Clone, Debug)]
pub struct ClassTable {
    cols: u32,
    rows: u32,
    classes: Vec<MoveClass>,
    grids: Vec<ClassGrid>,
    regions: Vec<Option<RegionGrid>>,
}

impl ClassTable {
    /// Build the terrain grid of every class over `t`. Class ids are indices
    /// into `classes`.
    pub fn build<T: Terrain + ?Sized>(t: &T, classes: Vec<MoveClass>) -> Result<Self, Error> {
        let (cols, rows) = check_map(t)?;
        if classes.len() > usize::from(u16::MAX) {
            return Err(Error::Capacity);
        }
        let grids = classes
            .iter()
            .map(|c| ClassGrid::build(t, c))
            .collect::<Result<Vec<_>, _>>()?;
        let regions = vec![None; classes.len()];
        Ok(Self {
            cols,
            rows,
            classes,
            grids,
            regions,
        })
    }

    /// Map size in cells.
    pub fn map_size(&self) -> (u32, u32) {
        (self.cols, self.rows)
    }

    /// The class definition.
    pub fn class(&self, id: ClassId) -> Option<&MoveClass> {
        self.classes.get(usize::from(id.0))
    }

    /// The class's terrain grid.
    pub fn grid(&self, id: ClassId) -> Option<&ClassGrid> {
        self.grids.get(usize::from(id.0))
    }

    /// Label the class's regions if not done yet.
    pub fn ensure_regions(&mut self, id: ClassId) {
        let i = usize::from(id.0);
        if let (Some(slot), Some(grid)) = (self.regions.get_mut(i), self.grids.get(i)) {
            if slot.is_none() {
                *slot = Some(RegionGrid::label(grid));
            }
        }
    }

    /// The class's regions, if labelled.
    pub fn regions(&self, id: ClassId) -> Option<&RegionGrid> {
        self.regions.get(usize::from(id.0)).and_then(Option::as_ref)
    }

    /// Footprint and class of a mobility.
    pub fn resolve(&self, m: Mobility) -> Result<(Footprint, Option<ClassId>), Error> {
        match m {
            Mobility::Class(id) => self
                .class(id)
                .map(|c| (c.footprint, Some(id)))
                .ok_or(Error::UnknownClass(id)),
            Mobility::Inline(fp) if fp.is_valid() => Ok((fp, None)),
            Mobility::Inline(_) => Err(Error::BadFootprint),
        }
    }

    /// Vertex extent for a footprint: valid top-left cells are `0..w` x `0..h`.
    pub fn vertex_extent(&self, fp: Footprint) -> (i32, i32) {
        (self.cols as i32 - fp.w + 1, self.rows as i32 - fp.h + 1)
    }
}
