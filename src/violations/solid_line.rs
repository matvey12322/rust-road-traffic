//! Solid-line crossing detection that does not depend on a track being inside
//! a zone.
//!
//! A solid line is configured inside a zone (`rules.solid_line`), but the
//! vehicle that crosses it does not have to be inside that zone's polygon, or
//! inside any zone at all. The detector is therefore fed every active track
//! once per frame and owns its own, zone-independent, crossing state.
//!
//! The search is a bucket grid over the line AABBs expanded by `margin_px`.
//! For a track it is enough to inspect the cells its swept AABB touches,
//! rather than testing every line against every box.

use std::collections::HashMap;

use uuid::Uuid;

use crate::lib::track_observations::{BoundingBoxPx, PointPx, TrackObservation};
use crate::lib::zones::SolidLine;
use crate::violations::event::{ViolationEvent, ViolationStatus};

/// Side of a cell in the solid-line bucket grid, in pixels.
pub const SOLID_LINE_GRID_CELL_PX: f32 = 64.0;

/// A line spanning more cells than this is kept in the `large` list instead.
/// Such a line would otherwise fill a big part of the grid, and a line across
/// the whole frame is not the common case.
const MAX_CELLS_PER_LINE: usize = 256;

/// A query spanning more cells than this falls back to checking all lines.
/// This keeps a bogus, huge bounding box from making the query loop very long.
const MAX_CELLS_PER_QUERY: usize = 4096;

const GEOMETRY_EPSILON: f32 = 1e-3;

/// Axis-aligned bounding box in frame pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

impl Aabb {
    pub fn new(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Self {
        Aabb {
            min_x: min_x.min(max_x),
            min_y: min_y.min(max_y),
            max_x: min_x.max(max_x),
            max_y: min_y.max(max_y),
        }
    }

    pub fn from_bbox(bbox: &BoundingBoxPx) -> Self {
        let x = bbox.x as f32;
        let y = bbox.y as f32;
        let right = x + bbox.width as f32;
        let bottom = y + bbox.height as f32;
        Aabb::new(x, y, right, bottom)
    }

    pub fn point(point: &PointPx) -> Self {
        Aabb {
            min_x: point.x,
            min_y: point.y,
            max_x: point.x,
            max_y: point.y,
        }
    }

    pub fn center(&self) -> PointPx {
        PointPx {
            x: (self.min_x + self.max_x) / 2.0,
            y: (self.min_y + self.max_y) / 2.0,
        }
    }

    pub fn union(&self, other: &Aabb) -> Self {
        Aabb {
            min_x: self.min_x.min(other.min_x),
            min_y: self.min_y.min(other.min_y),
            max_x: self.max_x.max(other.max_x),
            max_y: self.max_y.max(other.max_y),
        }
    }

    pub fn intersects(&self, other: &Aabb) -> bool {
        !(self.max_x < other.min_x
            || other.max_x < self.min_x
            || self.max_y < other.min_y
            || other.max_y < self.min_y)
    }

    pub fn expanded(&self, margin: f32) -> Self {
        let margin = if margin.is_finite() {
            margin.max(0.0)
        } else {
            0.0
        };
        Aabb {
            min_x: self.min_x - margin,
            min_y: self.min_y - margin,
            max_x: self.max_x + margin,
            max_y: self.max_y + margin,
        }
    }

    pub fn is_finite(&self) -> bool {
        self.min_x.is_finite()
            && self.min_y.is_finite()
            && self.max_x.is_finite()
            && self.max_y.is_finite()
    }
}

/// One configured solid line together with the zone it belongs to.
///
/// The line does not have to be geometrically inside `zone_id`; the zone is
/// used to keep the configuration and to attribute the crossing.
#[derive(Debug, Clone)]
pub struct SolidLineDefinition {
    /// Stable identifier of the line, also written into the violation.
    pub id: String,
    /// Zone this line is configured in. `None` is allowed for a line that has
    /// no zone at all; such a crossing is reported without a zone.
    pub zone_id: Option<String>,
    pub line: SolidLine,
}

impl SolidLineDefinition {
    pub fn new(id: String, zone_id: Option<String>, line: SolidLine) -> Self {
        SolidLineDefinition { id, zone_id, line }
    }

    pub fn from_zone(zone_id: String, line: SolidLine) -> Self {
        Self::new(format!("{zone_id}.solid_line"), Some(zone_id), line)
    }

    fn margin(&self) -> f32 {
        if self.line.margin_px.is_finite() {
            self.line.margin_px.max(0.0)
        } else {
            0.0
        }
    }

    fn endpoints(&self) -> (PointPx, PointPx) {
        (
            PointPx {
                x: self.line.line[0][0],
                y: self.line.line[0][1],
            },
            PointPx {
                x: self.line.line[1][0],
                y: self.line.line[1][1],
            },
        )
    }

    fn signature(&self) -> LineSignature {
        LineSignature {
            start: self.line.line[0],
            end: self.line.line[1],
            margin_px: self.line.margin_px,
        }
    }

    /// AABB of the segment expanded by `margin_px` on every side.
    pub fn expanded_aabb(&self) -> Aabb {
        let (a, b) = self.endpoints();
        Aabb::new(a.x, a.y, b.x, b.y).expanded(self.margin())
    }

    /// Fast AABB prefilter followed by the exact segment-rectangle test.
    /// `margin_px` widens the rectangle as well, because the margin is a
    /// corridor around the line rather than a property of the box.
    pub fn intersects_bbox(&self, bbox: &BoundingBoxPx) -> bool {
        let bbox_aabb = Aabb::from_bbox(bbox);
        if !self.expanded_aabb().intersects(&bbox_aabb) {
            return false;
        }
        let (a, b) = self.endpoints();
        segment_intersects_rect(&a, &b, &bbox_aabb.expanded(self.margin()))
    }

    /// Signed perpendicular distance from the line through its endpoints, in
    /// pixels. Positive and negative values are opposite sides.
    fn signed_distance(&self, point: &PointPx) -> f32 {
        let (a, b) = self.endpoints();
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let length = (dx * dx + dy * dy).sqrt();
        if !length.is_finite() || length <= GEOMETRY_EPSILON {
            return 0.0;
        }
        let area2 = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
        area2 / length
    }

    /// 1 and -1 are opposite sides, 0 is the neutral band of half-width
    /// `margin_px` around the line.
    fn classify_side(&self, point: &PointPx) -> i32 {
        let distance = self.signed_distance(point);
        let margin = self.margin();
        if distance > margin {
            1
        } else if distance < -margin {
            -1
        } else {
            0
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct LineSignature {
    start: [f32; 2],
    end: [f32; 2],
    margin_px: f32,
}

/// Bucket grid over solid lines. Cells hold indices into the line slice the
/// index was built from.
#[derive(Debug)]
pub struct SolidLineIndex {
    cell_size: f32,
    cells: HashMap<(i32, i32), Vec<usize>>,
    large: Vec<usize>,
    line_count: usize,
}

impl SolidLineIndex {
    pub fn new(lines: &[SolidLineDefinition]) -> Self {
        Self::with_cell_size(lines, SOLID_LINE_GRID_CELL_PX)
    }

    pub fn with_cell_size(lines: &[SolidLineDefinition], cell_size: f32) -> Self {
        let cell_size = if cell_size.is_finite() && cell_size > 0.0 {
            cell_size
        } else {
            SOLID_LINE_GRID_CELL_PX
        };
        let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        let mut large = Vec::new();

        for (index, line) in lines.iter().enumerate() {
            let aabb = line.expanded_aabb();
            if !aabb.is_finite() {
                large.push(index);
                continue;
            }
            let Some((min_cx, max_cx)) = cell_range(aabb.min_x, aabb.max_x, cell_size) else {
                large.push(index);
                continue;
            };
            let Some((min_cy, max_cy)) = cell_range(aabb.min_y, aabb.max_y, cell_size) else {
                large.push(index);
                continue;
            };

            let span_x = (max_cx - min_cx) as i64 + 1;
            let span_y = (max_cy - min_cy) as i64 + 1;
            if span_x > MAX_CELLS_PER_LINE as i64
                || span_y > MAX_CELLS_PER_LINE as i64
                || span_x.saturating_mul(span_y) > MAX_CELLS_PER_LINE as i64
            {
                large.push(index);
                continue;
            }

            for cx in min_cx..=max_cx {
                for cy in min_cy..=max_cy {
                    cells.entry((cx, cy)).or_default().push(index);
                }
            }
        }

        SolidLineIndex {
            cell_size,
            cells,
            large,
            line_count: lines.len(),
        }
    }

    /// Indices of lines whose expanded AABB may touch the query box.
    pub fn candidates(&self, query: &Aabb) -> Vec<usize> {
        let mut out = Vec::new();
        self.candidates_into(query, &mut out);
        out
    }

    /// Same as `candidates`, but appends to a caller-provided buffer so a
    /// caller that queries many boxes per frame reuses the allocation.
    pub fn candidates_into(&self, query: &Aabb, out: &mut Vec<usize>) {
        out.clear();
        if !query.is_finite() {
            return;
        }

        let Some((min_cx, max_cx)) = cell_range(query.min_x, query.max_x, self.cell_size) else {
            return;
        };
        let Some((min_cy, max_cy)) = cell_range(query.min_y, query.max_y, self.cell_size) else {
            return;
        };

        let span_x = (max_cx - min_cx) as i64 + 1;
        let span_y = (max_cy - min_cy) as i64 + 1;
        if span_x > MAX_CELLS_PER_QUERY as i64
            || span_y > MAX_CELLS_PER_QUERY as i64
            || span_x.saturating_mul(span_y) > MAX_CELLS_PER_QUERY as i64
        {
            // A gigantic query is cheaper to answer by looking at everything.
            out.extend(0..self.line_count);
            return;
        }

        for cx in min_cx..=max_cx {
            for cy in min_cy..=max_cy {
                if let Some(bucket) = self.cells.get(&(cx, cy)) {
                    out.extend_from_slice(bucket);
                }
            }
        }
        out.extend_from_slice(&self.large);
        out.sort_unstable();
        out.dedup();
    }
}

/// Converts a coordinate span into an inclusive cell range. `None` means the
/// span is not finite or does not fit into `i32` cell coordinates.
fn cell_range(min: f32, max: f32, cell_size: f32) -> Option<(i32, i32)> {
    if !min.is_finite() || !max.is_finite() {
        return None;
    }
    let min_cell = (min / cell_size).floor();
    let max_cell = (max / cell_size).floor();
    if !min_cell.is_finite() || !max_cell.is_finite() {
        return None;
    }
    if min_cell < i32::MIN as f32 || max_cell > i32::MAX as f32 {
        return None;
    }
    Some((min_cell as i32, max_cell as i32))
}

#[derive(Debug, Clone)]
struct TrackState {
    /// Last centre that was outside the neutral `margin_px` band. A vehicle
    /// crossing the band over several frames still has a point on each side to
    /// compare, so a thick margin does not hide the crossing.
    last_non_neutral_center: PointPx,
    last_non_neutral_side: i32,
    last_seen_at: f32,
    fired: bool,
}

/// Zone-independent solid-line detector.
///
/// One instance lives for the whole run and is fed every active track once per
/// frame. State is kept per `(line, track)` so the same crossing is reported
/// once; line changes reset the state of that line, and stale entries are
/// removed after `state_ttl_sec`.
pub struct SolidLineDetector {
    /// Crossing state per line, then per track. Two levels keep the hot path
    /// free of String allocations: lookups use `&str`, and the line id is
    /// cloned only when a line is seen for the first time.
    tracks: HashMap<String, HashMap<Uuid, TrackState>>,
    last_centers: HashMap<Uuid, (PointPx, f32)>,
    line_signatures: HashMap<String, LineSignature>,
    /// Bucket grid over the current lines. Rebuilt only when the line set or
    /// its geometry changes, not on every frame.
    index: SolidLineIndex,
    /// Reused across candidate queries within a frame.
    scratch: Vec<usize>,
    state_ttl_sec: f32,
    cleanup_interval_sec: f32,
    next_cleanup_at: f32,
}

impl SolidLineDetector {
    pub fn new(state_ttl_sec: f32, cleanup_interval_sec: f32) -> Self {
        let state_ttl_sec = if state_ttl_sec.is_finite() {
            state_ttl_sec.max(0.0)
        } else {
            0.0
        };
        let cleanup_interval_sec = if cleanup_interval_sec.is_finite() {
            cleanup_interval_sec.max(0.0)
        } else {
            0.0
        };
        SolidLineDetector {
            tracks: HashMap::new(),
            last_centers: HashMap::new(),
            line_signatures: HashMap::new(),
            index: SolidLineIndex::new(&[]),
            scratch: Vec::new(),
            state_ttl_sec,
            cleanup_interval_sec,
            next_cleanup_at: 0.0,
        }
    }

    /// Evaluates every line against every observation of one frame.
    ///
    /// The returned events belong to the frame; the caller publishes them.
    ///
    /// Generic over the observation container so a caller that shares one
    /// observation between consumers can pass `&[Arc<TrackObservation>]`
    /// while unit tests keep passing `&[TrackObservation]`.
    pub fn process_frame<T>(
        &mut self,
        lines: &[SolidLineDefinition],
        observations: &[T],
    ) -> Vec<ViolationEvent>
    where
        T: std::borrow::Borrow<TrackObservation>,
    {
        self.refresh_lines(lines);
        if observations.is_empty() {
            return Vec::new();
        }

        let now = observations
            .iter()
            .map(|obs| obs.borrow().source_time_relative_sec)
            .filter(|time| time.is_finite())
            .fold(f32::NEG_INFINITY, f32::max);
        if now.is_finite() {
            self.cleanup_if_needed(now);
        }

        let mut events = Vec::new();
        for obs in observations {
            let obs = obs.borrow();
            let now = obs.source_time_relative_sec;
            if !now.is_finite() {
                continue;
            }

            let bbox = Aabb::from_bbox(&obs.bbox);
            if !bbox.is_finite() {
                continue;
            }
            let center = bbox.center();

            // A fast track can jump from one side of the line to the other
            // between two processed frames without touching the line in either
            // sampled box. The query covers the whole swept area, so such a
            // crossing still finds its line.
            let query = match self.last_centers.get(&obs.track_id) {
                Some((previous, _)) => bbox.union(&Aabb::point(previous)),
                None => bbox,
            };

            self.index.candidates_into(&query, &mut self.scratch);
            let mut scratch_index = 0;
            while scratch_index < self.scratch.len() {
                let line_index = self.scratch[scratch_index];
                let line = &lines[line_index];
                if let Some(event) = self.process_line(line, obs, &center, now) {
                    events.push(event);
                }
                scratch_index += 1;
            }

            self.last_centers.insert(obs.track_id, (center, now));
        }
        events
    }

    /// Drops state for lines that disappeared or changed geometry. Without
    /// this, moving a line under a track that already crossed it could either
    /// suppress a real crossing or fire one that never happened.
    ///
    /// The common case is checked without allocations: the ids and signatures
    /// of the incoming lines are compared against the stored ones by borrowing
    /// their keys. The map is rebuilt only when something actually changed.
    fn refresh_lines(&mut self, lines: &[SolidLineDefinition]) {
        let unchanged = self.line_signatures.len() == lines.len()
            && lines.iter().all(|line| {
                self.line_signatures.get(line.id.as_str()) == Some(&line.signature())
            });
        if unchanged {
            return;
        }

        let current: HashMap<String, LineSignature> = lines
            .iter()
            .map(|line| (line.id.clone(), line.signature()))
            .collect();

        let previous = &self.line_signatures;
        self.tracks.retain(|line_id, _| {
            current
                .get(line_id)
                .is_some_and(|signature| previous.get(line_id) == Some(signature))
        });
        self.line_signatures = current;
        self.index = SolidLineIndex::new(lines);
    }

    fn cleanup_if_needed(&mut self, now: f32) {
        if now < self.next_cleanup_at {
            return;
        }

        let ttl = self.state_ttl_sec;
        for by_track in self.tracks.values_mut() {
            by_track.retain(|_, state| {
                let age = now - state.last_seen_at;
                age >= 0.0 && age <= ttl
            });
        }
        self.tracks.retain(|_, by_track| !by_track.is_empty());
        self.last_centers.retain(|_, (_, seen_at)| {
            let age = now - *seen_at;
            age >= 0.0 && age <= ttl
        });
        self.next_cleanup_at = now + self.cleanup_interval_sec;
    }

    fn process_line(
        &mut self,
        line: &SolidLineDefinition,
        obs: &TrackObservation,
        center: &PointPx,
        now: f32,
    ) -> Option<ViolationEvent> {
        if !line.line.is_valid() {
            return None;
        }

        let side = line.classify_side(center);
        let touches = line.intersects_bbox(&obs.bbox);
        let (line_start, line_end) = line.endpoints();

        let expired = self
            .tracks
            .get(line.id.as_str())
            .and_then(|by_track| by_track.get(&obs.track_id))
            .map_or(false, |state| {
                let age = now - state.last_seen_at;
                age < 0.0 || age > self.state_ttl_sec
            });
        if expired {
            if let Some(by_track) = self.tracks.get_mut(line.id.as_str()) {
                by_track.remove(&obs.track_id);
            }
        }

        let state = self
            .tracks
            .entry(line.id.clone())
            .or_default()
            .entry(obs.track_id)
            .or_insert_with(|| TrackState {
                last_non_neutral_center: center.clone(),
                last_non_neutral_side: side,
                last_seen_at: now,
                fired: false,
            });
        state.last_seen_at = now;

        if state.fired {
            if side != 0 {
                state.last_non_neutral_center = center.clone();
                state.last_non_neutral_side = side;
            }
            return None;
        }

        // Compare against the last point outside the neutral band, not against
        // the immediately previous frame: with a wide margin the previous side
        // is often 0, and the crossing would otherwise be lost.
        let previous_side = state.last_non_neutral_side;
        let previous_center = state.last_non_neutral_center.clone();
        let changed_side = previous_side != 0 && side != 0 && previous_side != side;
        let path_crosses =
            changed_side && segments_intersect(&previous_center, center, &line_start, &line_end);
        let crossed = changed_side && (touches || path_crosses);

        if side != 0 {
            state.last_non_neutral_center = center.clone();
            state.last_non_neutral_side = side;
        }

        if !crossed {
            return None;
        }
        state.fired = true;

        let intersection =
            segment_intersection_point(&previous_center, center, &line_start, &line_end)
                .unwrap_or_else(|| center.clone());

        Some(make_event(line, obs, previous_side, side, intersection))
    }

    /// Test hook: how many `(line, track)` states are alive right now.
    #[cfg(test)]
    fn state_count(&self) -> usize {
        self.tracks.values().map(|by_track| by_track.len()).sum()
    }
}

fn make_event(
    line: &SolidLineDefinition,
    obs: &TrackObservation,
    previous_side: i32,
    side: i32,
    intersection: PointPx,
) -> ViolationEvent {
    ViolationEvent {
        event_id: Uuid::new_v4(),
        event_type: "traffic.solid_line_crossing".to_string(),
        status: ViolationStatus::Confirmed,
        equipment_id: obs.equipment_id.clone(),
        track_id: obs.track_id,
        observed_at: obs.observed_at,
        source_time_relative_sec: obs.source_time_relative_sec,
        class_name: obs.class_name.clone(),
        confidence: obs.confidence,
        bbox: obs.bbox.clone(),
        anchor_px: obs.anchor_px.clone(),
        image_point_px: obs.image_point_px.clone(),
        projected_point: obs.projected_point.clone(),
        zone_id: line.zone_id.clone(),
        previous_zone_id: obs.previous_zone_id.clone(),
        crossed_virtual_line: obs.crossed_virtual_line,
        speed_kmh: obs.speed_kmh,
        solid_line_id: Some(line.id.clone()),
        intersection_px: Some(intersection),
        rule_id: line.id.clone(),
        details: format!(
            "track bbox crossed solid line '{}': side {} -> {}",
            line.id, previous_side, side
        ),
    }
}

fn signed_area2(a: &PointPx, b: &PointPx, c: &PointPx) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn on_segment(a: &PointPx, b: &PointPx, point: &PointPx) -> bool {
    point.x >= a.x.min(b.x) - GEOMETRY_EPSILON
        && point.x <= a.x.max(b.x) + GEOMETRY_EPSILON
        && point.y >= a.y.min(b.y) - GEOMETRY_EPSILON
        && point.y <= a.y.max(b.y) + GEOMETRY_EPSILON
}

fn segments_intersect(a: &PointPx, b: &PointPx, c: &PointPx, d: &PointPx) -> bool {
    let ab_c = signed_area2(a, b, c);
    let ab_d = signed_area2(a, b, d);
    let cd_a = signed_area2(c, d, a);
    let cd_b = signed_area2(c, d, b);

    if ab_c.abs() <= GEOMETRY_EPSILON && on_segment(a, b, c) {
        return true;
    }
    if ab_d.abs() <= GEOMETRY_EPSILON && on_segment(a, b, d) {
        return true;
    }
    if cd_a.abs() <= GEOMETRY_EPSILON && on_segment(c, d, a) {
        return true;
    }
    if cd_b.abs() <= GEOMETRY_EPSILON && on_segment(c, d, b) {
        return true;
    }

    let ab_cross = (ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0);
    let cd_cross = (cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0);

    ab_cross && cd_cross
}

/// Point where two finite segments cross, when it is defined. Collinear
/// overlap returns `None`; the caller falls back to the current box centre.
fn segment_intersection_point(
    a: &PointPx,
    b: &PointPx,
    c: &PointPx,
    d: &PointPx,
) -> Option<PointPx> {
    let rx = b.x - a.x;
    let ry = b.y - a.y;
    let sx = d.x - c.x;
    let sy = d.y - c.y;
    let denominator = rx * sy - ry * sx;
    if denominator.abs() <= GEOMETRY_EPSILON {
        return None;
    }

    let cax = c.x - a.x;
    let cay = c.y - a.y;
    let t = (cax * sy - cay * sx) / denominator;
    let u = (cax * ry - cay * rx) / denominator;
    if !(-GEOMETRY_EPSILON..=1.0 + GEOMETRY_EPSILON).contains(&t)
        || !(-GEOMETRY_EPSILON..=1.0 + GEOMETRY_EPSILON).contains(&u)
    {
        return None;
    }

    Some(PointPx {
        x: a.x + t * rx,
        y: a.y + t * ry,
    })
}

fn point_in_rect(point: &PointPx, rect: &Aabb) -> bool {
    point.x >= rect.min_x - GEOMETRY_EPSILON
        && point.x <= rect.max_x + GEOMETRY_EPSILON
        && point.y >= rect.min_y - GEOMETRY_EPSILON
        && point.y <= rect.max_y + GEOMETRY_EPSILON
}

fn segment_intersects_rect(segment_start: &PointPx, segment_end: &PointPx, rect: &Aabb) -> bool {
    if point_in_rect(segment_start, rect) || point_in_rect(segment_end, rect) {
        return true;
    }

    let corners = [
        PointPx {
            x: rect.min_x,
            y: rect.min_y,
        },
        PointPx {
            x: rect.max_x,
            y: rect.min_y,
        },
        PointPx {
            x: rect.max_x,
            y: rect.max_y,
        },
        PointPx {
            x: rect.min_x,
            y: rect.max_y,
        },
    ];

    for index in 0..4 {
        let next = (index + 1) % 4;
        if segments_intersect(segment_start, segment_end, &corners[index], &corners[next]) {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    fn observation(
        track_id: Uuid,
        time: f32,
        center_x: f32,
        center_y: f32,
        width: i32,
        height: i32,
        zone_id: Option<&str>,
    ) -> TrackObservation {
        let left = (center_x - width as f32 / 2.0).round() as i32;
        let top = (center_y - height as f32 / 2.0).round() as i32;
        TrackObservation {
            equipment_id: "eq".to_string(),
            track_id,
            observed_at: Utc::now(),
            source_time_relative_sec: time,
            class_name: "car".to_string(),
            confidence: 0.9,
            bbox: BoundingBoxPx {
                x: left,
                y: top,
                width,
                height,
            },
            anchor_px: PointPx {
                x: center_x,
                y: center_y + height as f32 / 2.0,
            },
            image_point_px: PointPx {
                x: center_x,
                y: center_y,
            },
            projected_point: None,
            speed_kmh: None,
            zone_id: zone_id.map(str::to_string),
            previous_zone_id: None,
            crossed_virtual_line: false,
            no_match_times: 0,
        }
    }

    fn horizontal_line(identifier: &str, y: f32, margin: f32) -> SolidLineDefinition {
        SolidLineDefinition::new(
            identifier.to_string(),
            Some("zone-1".to_string()),
            SolidLine::new([[0.0, y], [200.0, y]], margin),
        )
    }

    #[test]
    fn segment_intersects_rectangle() {
        let rect = Aabb::new(10.0, 10.0, 30.0, 30.0);
        assert!(segment_intersects_rect(
            &PointPx { x: 0.0, y: 20.0 },
            &PointPx { x: 40.0, y: 20.0 },
            &rect
        ));
        // Diagonally through opposite corners.
        assert!(segment_intersects_rect(
            &PointPx { x: 0.0, y: 0.0 },
            &PointPx { x: 40.0, y: 40.0 },
            &rect
        ));
        // One endpoint inside.
        assert!(segment_intersects_rect(
            &PointPx { x: 20.0, y: 20.0 },
            &PointPx { x: 50.0, y: 50.0 },
            &rect
        ));
        // Fully outside.
        assert!(!segment_intersects_rect(
            &PointPx { x: 0.0, y: 5.0 },
            &PointPx { x: 40.0, y: 5.0 },
            &rect
        ));
    }

    #[test]
    fn line_bbox_intersection_respects_margin() {
        let bbox = BoundingBoxPx {
            x: 50,
            y: 85,
            width: 20,
            height: 10,
        };
        let with_margin = horizontal_line("line", 100.0, 10.0);
        let without_margin = horizontal_line("line", 100.0, 0.0);

        assert!(with_margin.intersects_bbox(&bbox));
        assert!(!without_margin.intersects_bbox(&bbox));
        assert!(!with_margin.intersects_bbox(&BoundingBoxPx {
            x: 300,
            y: 85,
            width: 20,
            height: 10,
        }));
    }

    #[test]
    fn index_returns_only_nearby_lines() {
        let near = horizontal_line("near", 100.0, 0.0);
        let far = SolidLineDefinition::new(
            "far".to_string(),
            None,
            SolidLine::new([[1000.0, 1000.0], [1100.0, 1000.0]], 0.0),
        );
        let index = SolidLineIndex::new(&[near, far]);
        let bbox = BoundingBoxPx {
            x: 10,
            y: 90,
            width: 20,
            height: 20,
        };

        assert_eq!(index.candidates(&Aabb::from_bbox(&bbox)), vec![0]);
    }

    #[test]
    fn a_long_line_falls_back_to_the_always_checked_list() {
        let long = SolidLineDefinition::new(
            "long".to_string(),
            None,
            SolidLine::new([[0.0, 0.0], [20000.0, 0.0]], 0.0),
        );
        let short = horizontal_line("short", 100.0, 0.0);
        let index = SolidLineIndex::new(&[long, short]);
        let bbox = BoundingBoxPx {
            x: 10,
            y: 90,
            width: 20,
            height: 20,
        };

        let candidates = index.candidates(&Aabb::from_bbox(&bbox));
        assert!(candidates.contains(&0));
        assert!(candidates.contains(&1));
    }

    #[test]
    fn crossing_is_detected_outside_any_zone() {
        let line = horizontal_line("zone-1.solid_line", 100.0, 5.0);
        let track = Uuid::new_v4();
        let mut detector = SolidLineDetector::new(10.0, 1.0);

        let before = observation(track, 1.0, 100.0, 80.0, 20, 20, None);
        assert!(
            detector
                .process_frame(&[line.clone()], &[before])
                .is_empty()
        );

        let after = observation(track, 1.1, 100.0, 120.0, 20, 20, None);
        let events = detector.process_frame(&[line], &[after]);

        assert_eq!(events.len(), 1);
        let event = &events[0];
        assert_eq!(event.solid_line_id.as_deref(), Some("zone-1.solid_line"));
        assert_eq!(event.zone_id.as_deref(), Some("zone-1"));
        assert_eq!(event.track_id, track);
        let intersection = event.intersection_px.as_ref().expect("intersection");
        assert!((intersection.y - 100.0).abs() < 1.0, "{intersection:?}");
    }

    #[test]
    fn a_line_without_a_zone_reports_no_zone() {
        let line = SolidLineDefinition::new(
            "global.solid_line".to_string(),
            None,
            SolidLine::new([[0.0, 100.0], [200.0, 100.0]], 5.0),
        );
        let track = Uuid::new_v4();
        let mut detector = SolidLineDetector::new(10.0, 1.0);

        detector.process_frame(
            &[line.clone()],
            &[observation(track, 1.0, 100.0, 80.0, 20, 20, None)],
        );
        let events = detector.process_frame(
            &[line],
            &[observation(track, 1.1, 100.0, 120.0, 20, 20, None)],
        );

        assert_eq!(events.len(), 1);
        assert_eq!(events[0].zone_id, None);
        assert_eq!(
            events[0].solid_line_id.as_deref(),
            Some("global.solid_line")
        );
    }

    #[test]
    fn one_track_reports_a_crossing_once() {
        let line = horizontal_line("line", 100.0, 5.0);
        let track = Uuid::new_v4();
        let mut detector = SolidLineDetector::new(10.0, 1.0);

        detector.process_frame(
            &[line.clone()],
            &[observation(track, 1.0, 100.0, 80.0, 20, 20, None)],
        );
        assert_eq!(
            detector
                .process_frame(
                    &[line.clone()],
                    &[observation(track, 1.1, 100.0, 120.0, 20, 20, None)]
                )
                .len(),
            1
        );
        // Going back over the same line must not produce a second event.
        assert!(
            detector
                .process_frame(
                    &[line],
                    &[observation(track, 1.2, 100.0, 80.0, 20, 20, None)]
                )
                .is_empty()
        );
    }

    #[test]
    fn margin_can_keep_a_nearby_track_neutral() {
        let track = Uuid::new_v4();
        let wide = horizontal_line("wide", 100.0, 50.0);
        let mut detector = SolidLineDetector::new(10.0, 1.0);

        detector.process_frame(
            &[wide.clone()],
            &[observation(track, 1.0, 100.0, 80.0, 20, 20, None)],
        );
        assert!(
            detector
                .process_frame(
                    &[wide],
                    &[observation(track, 1.1, 100.0, 120.0, 20, 20, None)]
                )
                .is_empty()
        );

        let narrow = horizontal_line("narrow", 100.0, 5.0);
        let mut detector = SolidLineDetector::new(10.0, 1.0);
        detector.process_frame(
            &[narrow.clone()],
            &[observation(track, 1.0, 100.0, 80.0, 20, 20, None)],
        );
        assert_eq!(
            detector
                .process_frame(
                    &[narrow],
                    &[observation(track, 1.1, 100.0, 120.0, 20, 20, None)]
                )
                .len(),
            1
        );
    }

    #[test]
    fn crossing_through_the_margin_band_is_detected() {
        let line = horizontal_line("line", 100.0, 10.0);
        let track = Uuid::new_v4();
        let mut detector = SolidLineDetector::new(10.0, 1.0);

        // Above the line, outside the band.
        detector.process_frame(
            &[line.clone()],
            &[observation(track, 1.0, 100.0, 80.0, 20, 20, None)],
        );
        // Inside the band: the side is 0 on both of these frames.
        detector.process_frame(
            &[line.clone()],
            &[observation(track, 1.1, 100.0, 95.0, 20, 20, None)],
        );
        detector.process_frame(
            &[line.clone()],
            &[observation(track, 1.2, 100.0, 105.0, 20, 20, None)],
        );
        // Out of the band on the other side. The comparison is against the
        // last non-neutral point, so this is one crossing.
        let events = detector.process_frame(
            &[line],
            &[observation(track, 1.3, 100.0, 120.0, 20, 20, None)],
        );
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].solid_line_id.as_deref(), Some("line"));
    }

    #[test]
    fn moving_a_line_resets_the_state_of_that_line() {
        let track = Uuid::new_v4();
        let first = horizontal_line("line", 100.0, 5.0);
        let moved = horizontal_line("line", 300.0, 5.0);
        let mut detector = SolidLineDetector::new(10.0, 1.0);

        detector.process_frame(
            &[first.clone()],
            &[observation(track, 1.0, 100.0, 80.0, 20, 20, None)],
        );
        assert_eq!(
            detector
                .process_frame(
                    &[first],
                    &[observation(track, 1.1, 100.0, 120.0, 20, 20, None)]
                )
                .len(),
            1
        );
        assert_eq!(detector.state_count(), 1);

        // The line moved below the track. The old crossing state must not be
        // reused; the first frame with the new line establishes a baseline.
        detector.process_frame(
            &[moved.clone()],
            &[observation(track, 1.2, 100.0, 200.0, 20, 20, None)],
        );
        assert_eq!(detector.state_count(), 0);

        // The track is below the new line now: no crossing yet.
        assert!(
            detector
                .process_frame(
                    &[moved.clone()],
                    &[observation(track, 1.3, 100.0, 400.0, 20, 20, None)]
                )
                .is_empty()
        );
        // Moving back over it is a real crossing of the new geometry.
        assert_eq!(
            detector
                .process_frame(
                    &[moved],
                    &[observation(track, 1.4, 100.0, 250.0, 20, 20, None)]
                )
                .len(),
            1
        );
    }
}
