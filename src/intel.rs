use super::message::Msg;
use super::track::Track;
use oort_api::prelude::{ScanResult, Vec2};

/// This holds all general target, strategy and scenario-specific information
/// Ideally every ship's picture would converge to represent the same information about the current game state.
struct Picture<C, M> {
    contact_handler: C,
    message_handler: M,
    state: PictureState,
}
struct PictureState {
    tracks: Vec<Track>,
}

/// A trait for defining how to deal with new radar contact and how to integrate them into a picture.
trait ContactHandler {
    fn integrate(&mut self, contact: ScanResult, picture: &mut PictureState);
}

/// A trait for defining how to deal with and integrat a received message into a picture.
trait MessageHandler {
    fn integrate(&mut self, message: Msg, picture: &mut PictureState);
}

struct MoveCmd {
    target_pos: Vec2,
}

trait MoveStrategy {
    fn cmd(&mut self, picture: &PictureState) -> MoveCmd;
}
