use chrono::{DateTime, Utc};
use serde::Serialize;
use std::sync::Arc;
use tokio::sync::broadcast;
use uuid::Uuid;

/// Broadcast sender used to share track observations between consumers.
pub type TrackObservations = broadcast::Sender<Arc<TrackObservation>>;

/// Broadcast receiver used by an individual track observation consumer.
pub type TrackObservationRx = broadcast::Receiver<Arc<TrackObservation>>;

/// A point in video-frame pixel coordinates.
#[derive(Debug, Clone, Serialize)]
pub struct PointPx {
    pub x: f32,
    pub y: f32,
}

/// A point in the coordinate space used by the zone projection.
#[derive(Debug, Clone, Serialize)]
pub struct PointProjected {
    pub x: f32,
    pub y: f32,
}

/// An axis-aligned bounding box in video-frame pixel coordinates.
#[derive(Debug, Clone, Serialize)]
pub struct BoundingBoxPx {
    /// Horizontal coordinate of the top-left corner.
    pub x: i32,
    /// Vertical coordinate of the top-left corner.
    pub y: i32,
    pub width: i32,
    pub height: i32,
}

/// A snapshot of a tracked object's geometry, metadata, and zone context.
#[derive(Debug, Clone, Serialize)]
pub struct TrackObservation {
    /// Identifier of the installation producing the observation.
    pub equipment_id: String,
    /// Identifier assigned to the object by the tracker.
    pub track_id: Uuid,
    /// UTC timestamp associated with processing the observed frame.
    pub observed_at: DateTime<Utc>,
    /// Frame timestamp in seconds relative to the source timeline.
    pub source_time_relative_sec: f32,

    /// Object class associated with the track.
    pub class_name: String,
    /// Detection confidence supplied by the observation producer.
    /// This is not a confidence score for a violation.
    pub confidence: f32,

    /// Object bounding box in the video frame.
    pub bbox: BoundingBoxPx,
    /// Anchor point supplied by the producer, typically the bottom center
    /// of the bounding box.
    pub anchor_px: PointPx,
    /// Image-space track point used for zone geometry calculations.
    pub image_point_px: PointPx,
    /// Projected point, when available.
    pub projected_point: Option<PointProjected>,
    /// Estimated speed in kilometres per hour, when available.
    pub speed_kmh: Option<f32>,

    /// Identifier of the zone associated with this observation.
    pub zone_id: Option<String>,
    /// Previous zone identifier supplied by the producer, when available.
    pub previous_zone_id: Option<String>,
    /// Whether the producer detected a direction-specific virtual-line
    /// crossing for this observation.
    pub crossed_virtual_line: bool,

    /// Number of consecutive unmatched updates reported by the tracker.
    pub no_match_times: usize,
}

/// Subscribes to track observations and starts an asynchronous consumer.
///
/// Each received observation was serialized to JSON here, but the success
/// branch performed no output, so the serialization was pure waste: every
/// zone observation was encoded and then discarded. The only consumer left
/// is the violation observer in `violations::spawn_violation_observer`;
/// nothing else needs to be spawned for console output until there is
/// actually something to print.
#[cfg(test)]
pub(crate) mod test_support {
    use super::*;
    use chrono::Utc;

    /// Minimal observation for rule unit tests: the anchor and image point are
    /// the same, the box is 20x20 around that point.
    pub fn observation(
        track_id: Uuid,
        time: f32,
        x: f32,
        y: f32,
        zone_id: Option<&str>,
        speed_kmh: Option<f32>,
    ) -> TrackObservation {
        TrackObservation {
            equipment_id: "eq".to_string(),
            track_id,
            observed_at: Utc::now(),
            source_time_relative_sec: time,
            class_name: "car".to_string(),
            confidence: 0.9,
            bbox: BoundingBoxPx {
                x: (x - 10.0).round() as i32,
                y: (y - 10.0).round() as i32,
                width: 20,
                height: 20,
            },
            anchor_px: PointPx { x, y },
            image_point_px: PointPx { x, y },
            projected_point: None,
            speed_kmh,
            zone_id: zone_id.map(str::to_string),
            previous_zone_id: None,
            crossed_virtual_line: false,
            no_match_times: 0,
        }
    }
}
