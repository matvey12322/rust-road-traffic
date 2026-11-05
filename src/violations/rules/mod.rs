pub mod speeding;
pub mod state;
pub mod stationary;
pub mod wrong_way;

use crate::lib::track_observations::TrackObservation;
use crate::violations::event::ViolationEvent;

pub trait ViolationRule: Send {
    fn process(&mut self, obs: &TrackObservation) -> Vec<ViolationEvent>;
}
