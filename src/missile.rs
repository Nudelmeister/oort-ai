pub struct Missile {}
use oort_api::prelude as oa;

pub const FRAG_LIFE: f64 = 10.0 * oa::TICK_LENGTH;
pub const FRAG_RANGE: f64 = 150.0;
pub const FRAG_SPEED: f64 = FRAG_RANGE / FRAG_LIFE;
pub const FRAG_LETHAL_RANGE: f64 = 40.0;

impl Missile {
    pub fn new() -> Self {
        Self {}
    }

    pub fn tick(&mut self) {
        oa::explode();
    }
}

impl Default for Missile {
    fn default() -> Self {
        Self::new()
    }
}
