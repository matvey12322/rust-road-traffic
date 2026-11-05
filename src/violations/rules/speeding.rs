use std::collections::HashMap;

use uuid::Uuid;

use crate::lib::spatial::Point2f;
use crate::lib::track_observations::TrackObservation;
use crate::lib::zones::geometry::polygon_contains_point;
use crate::settings::LaneSpeedingRuleSettings;
use crate::violations::coordinator::RuleRuntimeConfig;
use crate::violations::event::{ViolationEvent, ViolationStatus};
use crate::violations::rules::ViolationRule;

pub struct SpeedingRule {
    rule_id: String,
    zone_id: String,
    polygon_px: Vec<Point2f>,
    threshold_kmh: f32,
    fired_tracks: HashMap<Uuid, f32>,

    state_ttl_sec: f32,
    cleanup_interval_sec: f32,
    next_cleanup_at: f32,
}

impl SpeedingRule {
    /// Creates a speeding rule from the zone geometry, speed threshold,
    /// and runtime settings used to expire reported-track records.
    pub fn from_embedded(
        zone_id: String,
        geometry: Vec<[i32; 2]>,
        settings: LaneSpeedingRuleSettings,
        runtime: RuleRuntimeConfig,
    ) -> Self {
        Self {
            rule_id: format!("{zone_id}.speeding"),
            zone_id,
            polygon_px: geometry
                .into_iter()
                .map(|pt| Point2f::new(pt[0] as f32, pt[1] as f32))
                .collect(),
            threshold_kmh: settings.threshold_kmh,
            fired_tracks: HashMap::new(),
            state_ttl_sec: runtime.state_ttl_sec,
            cleanup_interval_sec: runtime.cleanup_interval_sec,
            next_cleanup_at: 0.0,
        }
    }

    /// Removes expired reported-track records when the next cleanup is due.
    /// Each stored timestamp is the track's latest observation in source time;
    /// records with a negative age are also discarded.
    fn cleanup_if_needed(&mut self, now: f32) {
        if now < self.next_cleanup_at {
            return;
        }

        let ttl = self.state_ttl_sec;

        self.fired_tracks.retain(|_, last_seen_at| {
            let age = now - *last_seen_at;
            age >= 0.0 && age <= ttl
        });

        self.next_cleanup_at = now + self.cleanup_interval_sec;
    }

    /// Builds a confirmed speeding event from the current observation
    /// and the speed that exceeded the configured threshold.
    fn make_event(&self, obs: &TrackObservation, speed_kmh: f32) -> ViolationEvent {
        ViolationEvent {
            event_id: Uuid::new_v4(),
            event_type: "traffic.speeding".to_string(),
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
            zone_id: Some(self.zone_id.clone()),
            previous_zone_id: obs.previous_zone_id.clone(),
            crossed_virtual_line: obs.crossed_virtual_line,
            speed_kmh: Some(speed_kmh),
            solid_line_id: None,
            intersection_px: None,
            rule_id: self.rule_id.clone(),
            details: format!(
                "track exceeded speed threshold inside zone '{}' ({:.2} km/h > {:.2} km/h)",
                self.zone_id, speed_kmh, self.threshold_kmh,
            ),
        }
    }
}

impl ViolationRule for SpeedingRule {
    /// Processes a track observation and emits an event when its anchor
    /// is inside the configured polygon and its valid speed strictly
    /// exceeds the configured threshold.
    ///
    /// Only observations from this rule's zone are considered. The
    /// deduplication record is kept while the track stays in the zone and is
    /// dropped when it leaves the zone or the polygon, so a later visit can
    /// be reported again.
    fn process(&mut self, obs: &TrackObservation) -> Vec<ViolationEvent> {
        let now = obs.source_time_relative_sec;

        if !now.is_finite() {
            return Vec::new();
        }

        self.cleanup_if_needed(now);

        // Expire this track immediately, even between full cleanup passes.
        let expired = self
            .fired_tracks
            .get(&obs.track_id)
            .map_or(false, |last_seen_at| {
                let age = now - *last_seen_at;
                age < 0.0 || age > self.state_ttl_sec
            });

        if expired {
            self.fired_tracks.remove(&obs.track_id);
        }

        // A rule only applies to observations from its own zone. Otherwise an
        // observation from an overlapping or stale zone could be reported
        // against the wrong zone.
        if obs.zone_id.as_deref() != Some(self.zone_id.as_str()) {
            self.fired_tracks.remove(&obs.track_id);
            return Vec::new();
        }

        if !polygon_contains_point(&self.polygon_px, obs.anchor_px.x, obs.anchor_px.y) {
            self.fired_tracks.remove(&obs.track_id);
            return Vec::new();
        }

        let speed_kmh = match obs.speed_kmh {
            Some(speed) if speed.is_finite() && speed >= 0.0 => speed,
            _ => return Vec::new(),
        };

        if speed_kmh <= self.threshold_kmh {
            return Vec::new();
        }

        // Suppress repeats while the same speeding episode continues.
        if let Some(last_seen_at) = self.fired_tracks.get_mut(&obs.track_id) {
            *last_seen_at = now;
            return Vec::new();
        }

        self.fired_tracks.insert(obs.track_id, now);

        vec![self.make_event(obs, speed_kmh)]
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::lib::track_observations::test_support::observation;
    use crate::settings::LaneSpeedingRuleSettings;
    use crate::violations::coordinator::RuleRuntimeConfig;
    use uuid::Uuid;

    fn rule() -> SpeedingRule {
        SpeedingRule::from_embedded(
            "zone-a".to_string(),
            vec![[0, 0], [100, 0], [100, 100], [0, 100]],
            LaneSpeedingRuleSettings {
                threshold_kmh: 50.0,
            },
            RuleRuntimeConfig {
                state_ttl_sec: 7.0,
                cleanup_interval_sec: 1.0,
            },
        )
    }

    #[test]
    fn only_own_zone_and_new_speeding_episode() {
        let mut rule = rule();
        let track = Uuid::new_v4();

        // Another zone must not create an event even if its point is inside
        // this rule's polygon.
        assert!(
            rule.process(&observation(
                track,
                1.0,
                50.0,
                50.0,
                Some("zone-b"),
                Some(60.0)
            ))
            .is_empty()
        );
        // Own zone: first speeding event.
        assert_eq!(
            rule.process(&observation(
                track,
                1.1,
                50.0,
                50.0,
                Some("zone-a"),
                Some(60.0)
            ))
            .len(),
            1
        );
        // Still speeding in the same episode: suppressed.
        assert!(
            rule.process(&observation(
                track,
                1.2,
                50.0,
                50.0,
                Some("zone-a"),
                Some(60.0)
            ))
            .is_empty()
        );
        // Leaving the zone clears the deduplication record.
        assert!(
            rule.process(&observation(
                track,
                1.3,
                50.0,
                50.0,
                Some("zone-b"),
                Some(60.0)
            ))
            .is_empty()
        );
        // A later visit to the own zone can report again.
        assert_eq!(
            rule.process(&observation(
                track,
                1.4,
                50.0,
                50.0,
                Some("zone-a"),
                Some(60.0)
            ))
            .len(),
            1
        );
    }
}
