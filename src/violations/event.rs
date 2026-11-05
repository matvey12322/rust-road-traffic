use crate::lib::cv::Rect;
use crate::lib::spatial::Point2f;
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
