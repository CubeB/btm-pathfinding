//! Per-attacker stand-off points so a group attack fans out (SPEC 7).

use crate::geom::{clamp_i32, isqrt_i64, WorldPos, UNIT};
use crate::world::UnitId;

/// Footprint stand-off margin (8 world units).
pub const FOOTPRINT_MARGIN: i32 = 8 * UNIT;
/// Range stand-off margin (8 world units).
pub const RANGE_MARGIN: i32 = 8 * UNIT;
/// Cache tolerance band (+/- 8 world units).
pub const CACHE_BAND: i32 = 8 * UNIT;

/// The stand-off distance: the larger of the target's larger half-extent plus 8
/// and the weapon's range minus 8 (0 if the range is 8 or less).
pub fn standoff_distance(target_half_extent: i32, weapon_range: i32) -> i32 {
    let fp = target_half_extent.saturating_add(FOOTPRINT_MARGIN);
    let range = weapon_range.saturating_sub(RANGE_MARGIN).max(0);
    fp.max(range)
}

/// `target + normalise(attacker - target) * distance` (+x if they coincide).
pub fn standoff_point(target: WorldPos, attacker: WorldPos, distance: i32) -> WorldPos {
    let dx = i64::from(attacker.x) - i64::from(target.x);
    let dy = i64::from(attacker.y) - i64::from(target.y);
    let len = isqrt_i64(dx.saturating_mul(dx).saturating_add(dy.saturating_mul(dy)));
    let s = i64::from(distance);
    if len == 0 {
        return WorldPos::new(clamp_i32(i64::from(target.x) + s), target.y);
    }
    WorldPos::new(
        clamp_i32(i64::from(target.x) + dx * s / len),
        clamp_i32(i64::from(target.y) + dy * s / len),
    )
}

/// Whether the target is within weapon range (horizontal distance <= range).
pub fn in_weapon_range(attacker: WorldPos, target: WorldPos, range: i32) -> bool {
    let r = i64::from(range.max(0));
    attacker.dist2(target) <= r * r
}

/// A unit being chased.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Target {
    /// The target unit.
    pub id: UnitId,
    /// Its position.
    pub pos: WorldPos,
    /// Its height (fixed point); the stand-off point takes it.
    pub z: i32,
    /// The larger half-extent of its footprint, fixed point.
    pub half_extent: i32,
}

/// What a chasing attacker should do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chase {
    /// Within range: stop chasing and set weapon targets.
    InRange,
    /// Walk to this point.
    Approach {
        /// The stand-off point.
        point: WorldPos,
        /// At the target's height.
        z: i32,
    },
}

/// One attacker's stand-off cache, separate from any "where is the target" cache.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StandoffCache {
    cached: Option<(UnitId, WorldPos)>,
}

impl StandoffCache {
    /// Decide the approach to `target`. The cached point is reused while its
    /// distance to the target's current position is strictly inside
    /// (distance - 8, distance + 8), lower bound floored at 0.
    pub fn chase(&mut self, attacker: WorldPos, target: &Target, weapon_range: i32) -> Chase {
        if in_weapon_range(attacker, target.pos, weapon_range) {
            return Chase::InRange;
        }
        let s = standoff_distance(target.half_extent, weapon_range);
        if let Some((id, p)) = self.cached {
            if id == target.id {
                let d2 = p.dist2(target.pos);
                let lo = i64::from(s.saturating_sub(CACHE_BAND).max(0));
                let hi = i64::from(s.saturating_add(CACHE_BAND));
                if d2 > lo * lo && d2 < hi * hi {
                    return Chase::Approach {
                        point: p,
                        z: target.z,
                    };
                }
            }
        }
        let p = standoff_point(target.pos, attacker, s);
        self.cached = Some((target.id, p));
        Chase::Approach {
            point: p,
            z: target.z,
        }
    }

    /// Forget the cached point.
    pub fn clear(&mut self) {
        self.cached = None;
    }
}
