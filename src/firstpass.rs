//! The cheap first pass (SPEC 2.3): a bounded greedy walk toward the goal that
//! follows obstacle boundaries when blocked.
//!
//! The walk steps to the neighbour with the lowest octile score that improves on
//! the current one. When no such step is allowed it records a hit point and
//! follows the boundary with the obstacle on its left (scanning clockwise from a
//! known blocked direction). It leaves the boundary at the first vertex strictly
//! nearer than the hit point that has an improving step, and stops on reaching
//! the goal, on going all the way round (back at the hit point about to repeat
//! its first boundary move), when boxed in, or at the step bound. Each hit point
//! is strictly nearer than the last, so the walk cannot cycle between obstacles.

use crate::geom::{octile_score, Cell};
use crate::probe::Probe;
use crate::world::World;

/// What the first pass found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FirstPass {
    /// The closest vertex reached (by octile score; the first reached among ties).
    pub closest: Cell,
    /// Its octile score to the goal.
    pub closest_score: u32,
    /// Whether the walk stood on the goal vertex.
    pub reached: bool,
    /// Steps taken.
    pub steps: u32,
}

enum Mode {
    Greedy,
    Follow {
        hit: Cell,
        hit_score: u32,
        first: Option<u8>,
        wall: u8,
    },
}

fn improving(cur: Cell, goal: Cell, moves: &[bool; 8]) -> Option<u8> {
    let s = octile_score(cur, goal);
    let mut best: Option<(u32, u8)> = None;
    for d in 0..8u8 {
        if !moves[usize::from(d)] {
            continue;
        }
        let ns = octile_score(cur.step(d), goal);
        if ns < s && best.is_none_or(|(bs, _)| ns < bs) {
            best = Some((ns, d));
        }
    }
    best.map(|(_, d)| d)
}

fn desired(cur: Cell, goal: Cell) -> u8 {
    let s = ((goal.x - cur.x).signum(), (goal.y - cur.y).signum());
    crate::geom::DIRS
        .iter()
        .position(|&d| d == s)
        .map_or(0, |i| i as u8)
}

pub(crate) fn walk<W: World + ?Sized>(
    probe: &Probe<'_, W>,
    start: Cell,
    goal: Cell,
    max_steps: u32,
) -> FirstPass {
    let mut cur = start;
    let mut res = FirstPass {
        closest: start,
        closest_score: octile_score(start, goal),
        reached: false,
        steps: 0,
    };
    let mut mode = Mode::Greedy;
    // Mode switches without a step are bounded by this guard as well.
    let mut guard = u64::from(max_steps) * 4 + 16;
    while res.steps < max_steps && guard > 0 {
        guard -= 1;
        if cur == goal {
            res.reached = true;
            break;
        }
        let moves = probe.moves(cur);
        let s = octile_score(cur, goal);
        let d = match mode {
            Mode::Greedy => match improving(cur, goal, &moves) {
                Some(d) => d,
                None => {
                    if moves.iter().all(|m| !m) {
                        break;
                    }
                    mode = Mode::Follow {
                        hit: cur,
                        hit_score: s,
                        first: None,
                        wall: desired(cur, goal),
                    };
                    continue;
                }
            },
            Mode::Follow {
                hit,
                hit_score,
                first,
                wall,
            } => {
                if s < hit_score && improving(cur, goal, &moves).is_some() {
                    mode = Mode::Greedy;
                    continue;
                }
                let mut w = wall;
                let mut n = 0;
                while moves[usize::from(w)] && n < 8 {
                    w = (w + 7) & 7;
                    n += 1;
                }
                if n == 8 {
                    // Lost the wall: everything around is open.
                    mode = Mode::Greedy;
                    continue;
                }
                let Some(m) = (1..=8u8)
                    .map(|k| (w + k) & 7)
                    .find(|&c| moves[usize::from(c)])
                else {
                    break;
                };
                if cur == hit {
                    match first {
                        Some(f) if f == m => break,
                        Some(_) => {}
                        None => {
                            mode = Mode::Follow {
                                hit,
                                hit_score,
                                first: Some(m),
                                wall,
                            };
                        }
                    }
                }
                if let Mode::Follow {
                    wall: ref mut wl, ..
                } = mode
                {
                    *wl = if crate::geom::is_diagonal(m) {
                        (m + 5) & 7
                    } else {
                        (m + 6) & 7
                    };
                }
                m
            }
        };
        cur = cur.step(d);
        res.steps += 1;
        let ns = octile_score(cur, goal);
        if ns < res.closest_score {
            res.closest_score = ns;
            res.closest = cur;
        }
    }
    if cur == goal {
        res.reached = true;
    }
    res
}
