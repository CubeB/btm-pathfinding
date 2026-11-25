//! The sliced A* search (SPEC 1.1 - 1.9, 2.1).
//!
//! A search can be run in slices of any size; it keeps its open list between
//! calls and the slices concatenate to the same result as one uninterrupted run.
//! Per-vertex state lives in a [`Scratch`] shared by all searches (one at a time),
//! stamped with a generation so it never needs clearing.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::classes::ClassTable;
use crate::cost::{octile_estimate, Cost, DIAG, ORTHO};
use crate::error::Error;
use crate::geom::{is_diagonal, octile_score, turn_size, Cell, CellRect, Footprint, DIRS};
use crate::probe::Probe;
use crate::world::{Mobility, UnitId, World};

const NO_PARENT: u8 = u8::MAX;

/// Per-vertex working memory for searches over one map: about 22 bytes per cell.
#[derive(Clone, Debug)]
pub struct Scratch {
    cols: i32,
    rows: i32,
    gen: u32,
    stamp: Vec<u32>,
    g: Vec<Cost>,
    parent: Vec<u8>,
    closed: Vec<bool>,
}

impl Scratch {
    /// Scratch space for a `cols` x `rows` map.
    pub fn new(cols: u32, rows: u32) -> Self {
        let n = cols as usize * rows as usize;
        Self {
            cols: cols as i32,
            rows: rows as i32,
            gen: 0,
            stamp: vec![0; n],
            g: vec![Cost::MAX; n],
            parent: vec![NO_PARENT; n],
            closed: vec![false; n],
        }
    }

    fn begin(&mut self) -> u32 {
        if self.gen == u32::MAX {
            self.stamp.fill(0);
            self.gen = 0;
        }
        self.gen += 1;
        self.gen
    }

    fn index(&self, v: Cell) -> usize {
        v.y as usize * self.cols as usize + v.x as usize
    }

    fn cell(&self, i: usize) -> Cell {
        let c = self.cols.max(1) as usize;
        Cell::new((i % c) as i32, (i / c) as i32)
    }

    fn touch(&mut self, i: usize, gen: u32) {
        if self.stamp[i] != gen {
            self.stamp[i] = gen;
            self.g[i] = Cost::MAX;
            self.parent[i] = NO_PARENT;
            self.closed[i] = false;
        }
    }
}

/// What a search aims at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SearchGoal {
    /// A point goal's vertex; accepted exactly or within the slack.
    Vertex(Cell),
    /// A rectangle: accept any vertex whose footprint touches its outside.
    Rect(CellRect),
}

/// Parameters of one search.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SearchSpec {
    /// The searching unit (passed to [`crate::world::Obstacles::cell_blocked`]).
    pub unit: UnitId,
    /// Its movement class or inline footprint.
    pub mobility: Mobility,
    /// Start vertex; never tested for standability.
    pub start: Cell,
    /// Cells exempt from obstacles: those the unit occupies.
    pub own: CellRect,
    /// The goal.
    pub goal: SearchGoal,
    /// Octile score within which a vertex counts as arrival (0 = exact only).
    pub slack: u32,
    /// Keep the order of expanded vertices (diagnostics and tests).
    pub record_expansions: bool,
}

/// How a search ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchOutcome {
    /// Whether the goal was accepted; false means exhausted.
    pub found: bool,
    /// The last vertex: the accepted one, or the closest one reached.
    pub end: Cell,
    /// Vertices from start to end.
    pub path: Vec<Cell>,
    /// Expansions over the whole search.
    pub expansions: u64,
}

/// Result of one slice.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SearchStatus {
    /// Budget ran out; call [`Search::run`] again.
    Suspended,
    /// The search ended.
    Finished(SearchOutcome),
}

#[derive(PartialEq, Eq)]
struct Entry {
    f: Cost,
    h: Cost,
    seq: u64,
    g: Cost,
    idx: usize,
}

impl Ord for Entry {
    // Reversed: BinaryHeap is a max-heap and we want the smallest f, then the
    // smallest estimate, then the earliest offer.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .cmp(&self.f)
            .then_with(|| other.h.cmp(&self.h))
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn abs_diff(a: i32, b: i32) -> u64 {
    (i64::from(a) - i64::from(b)).unsigned_abs()
}

/// The estimate of SPEC 1.3 (point) and 1.8 (rectangle).
pub fn heuristic(goal: SearchGoal, fp: Footprint, v: Cell) -> Cost {
    match goal {
        SearchGoal::Vertex(g) => octile_estimate(abs_diff(v.x, g.x), abs_diff(v.y, g.y)),
        SearchGoal::Rect(r) => {
            // Accepted top-left cells form the ring of [x0 - w, x1] x [y0 - h, y1].
            let (lx, hx) = (r.x0 - fp.w, r.x1);
            let (ly, hy) = (r.y0 - fp.h, r.y1);
            let dx = if v.x < lx {
                abs_diff(lx, v.x)
            } else if v.x > hx {
                abs_diff(v.x, hx)
            } else {
                0
            };
            let dy = if v.y < ly {
                abs_diff(ly, v.y)
            } else if v.y > hy {
                abs_diff(v.y, hy)
            } else {
                0
            };
            if dx > 0 || dy > 0 {
                octile_estimate(dx, dy)
            } else {
                let d = abs_diff(v.x, lx)
                    .min(abs_diff(hx, v.x))
                    .min(abs_diff(v.y, ly))
                    .min(abs_diff(hy, v.y));
                octile_estimate(d, 0)
            }
        }
    }
}

/// Whether the footprint block at `v` touches `r` from outside (corners count).
pub fn touches(v: Cell, fp: Footprint, r: CellRect) -> bool {
    let b = CellRect::block(v, fp);
    !b.overlaps(r) && b.expand(1, 1).overlaps(r)
}

fn accepts(goal: SearchGoal, slack: u32, fp: Footprint, v: Cell) -> bool {
    match goal {
        SearchGoal::Vertex(g) => v == g || (slack > 0 && octile_score(v, g) <= slack),
        SearchGoal::Rect(r) => touches(v, fp, r),
    }
}

/// One A* search, resumable across ticks.
#[derive(Debug)]
pub struct Search {
    spec: SearchSpec,
    fp: Footprint,
    gen: u32,
    heap: BinaryHeap<Entry>,
    seq: u64,
    expansions: u64,
    best: Option<(Cost, usize)>,
    expanded: Vec<Cell>,
    finished: bool,
}

impl std::fmt::Debug for Entry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Entry({:?}, {})", self.f, self.idx)
    }
}

impl Search {
    /// Start a search. This claims `scratch`: starting another search with the
    /// same scratch invalidates this one.
    pub fn new(spec: SearchSpec, table: &ClassTable, scratch: &mut Scratch) -> Result<Self, Error> {
        let (fp, _) = table.resolve(spec.mobility)?;
        let (cols, rows) = table.map_size();
        if scratch.cols != cols as i32 || scratch.rows != rows as i32 {
            return Err(Error::StaleSearch);
        }
        let (vw, vh) = table.vertex_extent(fp);
        if vw <= 0 || vh <= 0 {
            return Err(Error::FootprintExceedsMap);
        }
        let s = spec.start;
        if s.x < 0 || s.y < 0 || s.x >= vw || s.y >= vh {
            return Err(Error::OutOfBounds);
        }
        if let SearchGoal::Rect(r) = spec.goal {
            if r.is_empty() {
                return Err(Error::EmptyRect);
            }
        }
        let gen = scratch.begin();
        let idx = scratch.index(s);
        scratch.touch(idx, gen);
        scratch.g[idx] = Cost::ZERO;
        let h = heuristic(spec.goal, fp, s);
        let mut heap = BinaryHeap::new();
        heap.push(Entry {
            f: h,
            h,
            seq: 0,
            g: Cost::ZERO,
            idx,
        });
        Ok(Self {
            spec,
            fp,
            gen,
            heap,
            seq: 1,
            expansions: 0,
            best: None,
            expanded: Vec::new(),
            finished: false,
        })
    }

    /// The parameters.
    pub fn spec(&self) -> &SearchSpec {
        &self.spec
    }

    /// Expansions so far.
    pub fn expansions(&self) -> u64 {
        self.expansions
    }

    /// Expanded vertices in order, if recording was asked for.
    pub fn expanded(&self) -> &[Cell] {
        &self.expanded
    }

    /// Run up to `budget` expansions. Returns the expansions spent and whether
    /// the search finished. Obstacles are read live from `world`.
    pub fn run<W: World + ?Sized>(
        &mut self,
        world: &W,
        table: &ClassTable,
        scratch: &mut Scratch,
        budget: u32,
    ) -> Result<(u32, SearchStatus), Error> {
        if self.finished {
            return Err(Error::SearchFinished);
        }
        if scratch.gen != self.gen {
            return Err(Error::StaleSearch);
        }
        let probe = Probe::new(
            world,
            table,
            self.spec.unit,
            self.spec.mobility,
            self.spec.own,
        )?;
        let gen = self.gen;
        let mut spent = 0u32;
        loop {
            if self.heap.is_empty() {
                let end = self
                    .best
                    .map_or_else(|| scratch.index(self.spec.start), |(_, i)| i);
                return Ok((spent, self.finish(scratch, false, end)));
            }
            if spent >= budget {
                return Ok((spent, SearchStatus::Suspended));
            }
            let Some(e) = self.heap.pop() else {
                continue;
            };
            if scratch.closed[e.idx] || scratch.g[e.idx] != e.g {
                continue;
            }
            spent += 1;
            self.expansions += 1;
            scratch.closed[e.idx] = true;
            let v = scratch.cell(e.idx);
            if self.spec.record_expansions {
                self.expanded.push(v);
            }
            if accepts(self.spec.goal, self.spec.slack, self.fp, v) {
                return Ok((spent, self.finish(scratch, true, e.idx)));
            }
            if self.best.is_none_or(|(bh, _)| e.h < bh) {
                self.best = Some((e.h, e.idx));
            }
            let moves = probe.moves(v);
            let incoming = scratch.parent[e.idx];
            for d in 0..8u8 {
                if !moves[usize::from(d)] {
                    continue;
                }
                let n = v.step(d);
                let ni = scratch.index(n);
                scratch.touch(ni, gen);
                if scratch.closed[ni] {
                    continue;
                }
                let base = if is_diagonal(d) { DIAG } else { ORTHO };
                let step = Cost {
                    len: base * probe.factor(n),
                    turns: if incoming == NO_PARENT {
                        0
                    } else {
                        turn_size(incoming, d)
                    },
                };
                let ng = e.g.plus(step);
                if ng < scratch.g[ni] {
                    scratch.g[ni] = ng;
                    scratch.parent[ni] = d;
                    let h = heuristic(self.spec.goal, self.fp, n);
                    self.heap.push(Entry {
                        f: ng.plus(h),
                        h,
                        seq: self.seq,
                        g: ng,
                        idx: ni,
                    });
                    self.seq += 1;
                }
            }
        }
    }

    fn finish(&mut self, scratch: &Scratch, found: bool, end: usize) -> SearchStatus {
        self.finished = true;
        self.heap = BinaryHeap::new();
        let mut path = Vec::new();
        let mut i = end;
        for _ in 0..=scratch.stamp.len() {
            let c = scratch.cell(i);
            path.push(c);
            let p = scratch.parent[i];
            if p == NO_PARENT || scratch.stamp[i] != self.gen {
                break;
            }
            let (dx, dy) = DIRS[usize::from(p & 7)];
            i = scratch.index(Cell::new(c.x - dx, c.y - dy));
        }
        path.reverse();
        SearchStatus::Finished(SearchOutcome {
            found,
            end: scratch.cell(end),
            path,
            expansions: self.expansions,
        })
    }
}
