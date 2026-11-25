//! Terrain, movement classes and the per-class terrain grid (SPEC 1.4, 1.5, 1.7).

use crate::error::Error;
use crate::geom::{Cell, Footprint};

/// Largest supported map side, in cells (bounds memory).
pub const MAX_MAP_SIDE: u32 = 4096;

/// Static map terrain. Heights are corner samples: a map of `cols` x `rows`
/// cells has `(cols + 1) x (rows + 1)` samples, and cell (x, y) has corners
/// (x, y), (x+1, y), (x, y+1), (x+1, y+1).
pub trait Terrain {
    /// Map size in cells: (columns, rows).
    fn cells(&self) -> (u32, u32);
    /// Height sample at corner (`x`, `y`), `x <= cols`, `y <= rows`, in the map's
    /// height units.
    fn sample(&self, x: u32, y: u32) -> i32;
    /// Sea level in height units. Water depth is sea level minus height.
    fn sea_level(&self) -> i32;
    /// Whether cell (`x`, `y`) is lava-excluded.
    fn lava(&self, x: u32, y: u32) -> bool {
        let _ = (x, y);
        false
    }
}

pub(crate) fn check_map<T: Terrain + ?Sized>(t: &T) -> Result<(u32, u32), Error> {
    let (cols, rows) = t.cells();
    if cols == 0 || rows == 0 {
        return Err(Error::EmptyMap);
    }
    if cols > MAX_MAP_SIDE || rows > MAX_MAP_SIDE {
        return Err(Error::MapTooLarge { cols, rows });
    }
    Ok((cols, rows))
}

/// Lowest and highest corner of cell (`x`, `y`).
pub fn cell_range<T: Terrain + ?Sized>(t: &T, x: u32, y: u32) -> (i32, i32) {
    let s = [
        t.sample(x, y),
        t.sample(x + 1, y),
        t.sample(x, y + 1),
        t.sample(x + 1, y + 1),
    ];
    let lo = s.iter().copied().min().unwrap_or(0);
    let hi = s.iter().copied().max().unwrap_or(0);
    (lo, hi)
}

/// Per-unit-type terrain rules (SPEC 0, 1.4, 1.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MoveClass {
    /// Size in cells.
    pub footprint: Footprint,
    /// Minimum water depth (0 for land units).
    pub min_depth: i32,
    /// Maximum water depth.
    pub max_depth: i32,
    /// Maximum cell slope on land.
    pub max_slope: i32,
    /// Maximum cell slope under water.
    pub max_water_slope: i32,
    /// Slope above which land is rough; default `max_slope / 2`.
    pub bad_slope: Option<i32>,
    /// Slope above which ground under water is rough; default `max_water_slope / 2`.
    pub bad_water_slope: Option<i32>,
    /// Ships and other floating units: never charged for water.
    pub floater: bool,
    /// Hover craft: bad slope equals max slope, and no underwater slow-down.
    pub hover: bool,
}

impl MoveClass {
    /// A land unit that may wade up to `max_depth`.
    pub const fn tank(footprint: Footprint, max_slope: i32, max_depth: i32) -> Self {
        Self {
            footprint,
            min_depth: 0,
            max_depth,
            max_slope,
            max_water_slope: max_slope,
            bad_slope: None,
            bad_water_slope: None,
            floater: false,
            hover: false,
        }
    }

    /// A ship that needs at least `min_depth` of water.
    pub const fn ship(footprint: Footprint, min_depth: i32) -> Self {
        Self {
            footprint,
            min_depth,
            max_depth: i32::MAX,
            max_slope: 255,
            max_water_slope: 255,
            bad_slope: None,
            bad_water_slope: None,
            floater: true,
            hover: false,
        }
    }

    /// A hover craft: crosses land and water, floats on water, and pays nothing
    /// for slopes it can climb.
    pub const fn hover(footprint: Footprint, max_slope: i32) -> Self {
        Self {
            footprint,
            min_depth: 0,
            max_depth: i32::MAX,
            max_slope,
            max_water_slope: max_slope,
            bad_slope: None,
            bad_water_slope: None,
            floater: true,
            hover: true,
        }
    }

    /// Effective bad slope on land: hover = max; else explicit or `max / 2`,
    /// never above max (SPEC 1.4).
    pub fn land_bad_slope(&self) -> i32 {
        if self.hover {
            self.max_slope
        } else {
            self.bad_slope
                .unwrap_or(self.max_slope / 2)
                .min(self.max_slope)
        }
    }

    /// Effective bad slope under water.
    pub fn water_bad_slope(&self) -> i32 {
        if self.hover {
            self.max_water_slope
        } else {
            self.bad_water_slope
                .unwrap_or(self.max_water_slope / 2)
                .min(self.max_water_slope)
        }
    }

    /// A wader: not a floater and allowed a minimum depth of 0 (SPEC 1.2).
    pub fn is_wader(&self) -> bool {
        !self.floater && self.min_depth <= 0
    }
}

const STAND: u8 = 1;
const ROUGH: u8 = 2;
const WET: u8 = 4;

/// The precomputed terrain lookup of one movement class (SPEC 1.7): for every
/// vertex where the footprint fits, whether it is terrain-standable, rough and
/// under water. Built once and never changed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClassGrid {
    vw: i32,
    vh: i32,
    flags: Vec<u8>,
}

impl ClassGrid {
    /// Build the grid of `class` over `t`.
    ///
    /// Choices where SPEC leaves room: a cell's water depth is checked at every
    /// corner (the shallowest corner against the minimum, the deepest against the
    /// maximum); the slope limit (land or water) is chosen per vertex by whether
    /// its (footprint + 1) sample block touches water; "footprint plus one cell of
    /// margin" means cells `x - 1 ..= x + w` and `y - 1 ..= y + h` all on the map.
    pub fn build<T: Terrain + ?Sized>(t: &T, class: &MoveClass) -> Result<Self, Error> {
        let (cols, rows) = check_map(t)?;
        let fp = class.footprint;
        if !fp.is_valid() {
            return Err(Error::BadFootprint);
        }
        let (ci, ri) = (cols as i32, rows as i32);
        let vw = ci - fp.w + 1;
        let vh = ri - fp.h + 1;
        if vw <= 0 || vh <= 0 {
            return Ok(Self {
                vw: 0,
                vh: 0,
                flags: Vec::new(),
            });
        }
        let n = cols as usize * rows as usize;
        let mut lo = vec![0i32; n];
        let mut hi = vec![0i32; n];
        let mut lava = vec![false; n];
        for y in 0..rows {
            for x in 0..cols {
                let i = y as usize * cols as usize + x as usize;
                let (l, h) = cell_range(t, x, y);
                lo[i] = l;
                hi[i] = h;
                lava[i] = t.lava(x, y);
            }
        }
        let sea = t.sea_level();
        let bad_land = class.land_bad_slope();
        let bad_water = class.water_bad_slope();
        let mut flags = vec![0u8; vw as usize * vh as usize];
        for vy in 0..vh {
            for vx in 0..vw {
                let mut wet = false;
                for cy in vy..vy + fp.h {
                    for cx in vx..vx + fp.w {
                        if lo[cy as usize * cols as usize + cx as usize] < sea {
                            wet = true;
                        }
                    }
                }
                let limit = if wet {
                    class.max_water_slope
                } else {
                    class.max_slope
                };
                let margin = vx >= 1 && vy >= 1 && vx + fp.w < ci && vy + fp.h < ri;
                let mut ok = true;
                let mut rough = false;
                for cy in vy..vy + fp.h {
                    for cx in vx..vx + fp.w {
                        let i = cy as usize * cols as usize + cx as usize;
                        let slope = hi[i].saturating_sub(lo[i]);
                        let deepest = sea.saturating_sub(lo[i]).max(0);
                        let shallowest = sea.saturating_sub(hi[i]).max(0);
                        if lava[i]
                            || slope > limit
                            || shallowest < class.min_depth
                            || deepest > class.max_depth
                        {
                            ok = false;
                        }
                        let bad = if lo[i] >= sea { bad_land } else { bad_water };
                        if slope > bad {
                            rough = true;
                        }
                    }
                }
                let mut f = 0;
                if ok {
                    f |= STAND;
                }
                if rough && margin {
                    f |= ROUGH;
                }
                if wet {
                    f |= WET;
                }
                flags[vy as usize * vw as usize + vx as usize] = f;
            }
        }
        Ok(Self { vw, vh, flags })
    }

    /// Vertex extent: valid top-left cells are `0..w` x `0..h`.
    pub fn extent(&self) -> (i32, i32) {
        (self.vw, self.vh)
    }

    fn flag(&self, v: Cell) -> u8 {
        if v.x < 0 || v.y < 0 || v.x >= self.vw || v.y >= self.vh {
            return 0;
        }
        self.flags
            .get(v.y as usize * self.vw as usize + v.x as usize)
            .copied()
            .unwrap_or(0)
    }

    /// Whether `v` is terrain-standable (false off the grid).
    pub fn standable(&self, v: Cell) -> bool {
        self.flag(v) & STAND != 0
    }

    /// Whether `v` is rough (SPEC 1.4).
    pub fn rough(&self, v: Cell) -> bool {
        self.flag(v) & ROUGH != 0
    }

    /// Whether `v` is under water (SPEC 1.5).
    pub fn wet(&self, v: Cell) -> bool {
        self.flag(v) & WET != 0
    }
}
