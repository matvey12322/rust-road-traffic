use crate::lib::cv::Rect;
use crate::lib::spatial::Point2f;
use crate::lib::track_observations::TrackObservation;
use chrono::{DateTime, Utc};
use serde::Serialize;
use uuid::Uuid;

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ViolationStatus {
    Confirmed,
}

#[derive(Debug, Clone, Serialize)]
pub struct ViolationEvent {
    pub event_id: Uuid,
    pub event_type: String,
    pub status: ViolationStatus,

    pub equipment_id: String,
    pub track_id: Uuid,

    pub observed_at: DateTime<Utc>,
    pub source_time_relative_sec: f32,

    pub class_name: String,
    pub confidence: f32,

    pub bbox: Rect,
    pub anchor_px: Point2f,
    pub image_point_px: Point2f,
    pub projected_point: Option<Point2f>,

    pub zone_id: Option<String>,
    pub previous_zone_id: Option<String>,
    pub crossed_virtual_line: bool,

    pub speed_kmh: Option<f32>,
    /// Solid line that was crossed, when the event comes from one.
    /// Other rules leave it out of the JSON.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solid_line_id: Option<String>,
    /// Point where the track crossed the solid line, when known.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intersection_px: Option<Point2f>,
    pub rule_id: String,
    pub details: String,
}

impl ViolationEvent {
    /// Builds a confirmed event that mirrors the observation. `event_type` and
    /// `rule_id` identify the rule; `details` carries the rule's explanation.
    /// Rules override specific fields (for example `speed_kmh` or the solid
    /// line) after construction.
    pub fn from_observation(
        obs: &TrackObservation,
        event_type: &str,
        zone_id: Option<String>,
        rule_id: String,
        details: String,
    ) -> Self {
        ViolationEvent {
            event_id: Uuid::new_v4(),
            event_type: event_type.to_string(),
            status: ViolationStatus::Confirmed,
            equipment_id: obs.equipment_id.clone(),
            track_id: obs.track_id,
            observed_at: obs.observed_at,
            source_time_relative_sec: obs.source_time_relative_sec,
            class_name: obs.class_name.clone(),
            confidence: obs.confidence,
            bbox: obs.bbox,
            anchor_px: obs.anchor_px,
            image_point_px: obs.image_point_px,
            projected_point: obs.projected_point,
            zone_id,
            previous_zone_id: obs.previous_zone_id.clone(),
            crossed_virtual_line: obs.crossed_virtual_line,
            speed_kmh: obs.speed_kmh,
            solid_line_id: None,
            intersection_px: None,
            rule_id,
            details,
        }
    }

    /// Attaches the solid line the event comes from and the crossing point.
    pub fn for_solid_line(mut self, line_id: String, intersection: Point2f) -> Self {
        self.solid_line_id = Some(line_id);
        self.intersection_px = Some(intersection);
        self
    }
}
