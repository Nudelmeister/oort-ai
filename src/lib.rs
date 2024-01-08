//mod intel;
//mod liblib;
//mod message;
//mod missile;
mod radar;
mod roots;
mod track;

use fighter::Fighter;
use oort_api::{
    prelude::{self as oa, Vec2, Vec2Extras},
    Class,
};

mod fighter;

pub enum Ship {
    Fighter(Fighter),
}

impl Ship {
    pub fn new() -> Self {
        match oa::class() {
            Class::Fighter => Self::Fighter(Fighter::new()),
            _ => unimplemented!(),
        }
    }

    pub fn tick(&mut self) {
        match self {
            Self::Fighter(s) => s.tick(),
        }
    }
}

impl Default for Ship {
    fn default() -> Self {
        Self::new()
    }
}

fn position_in(time: f64) -> Vec2 {
    oa::position() + oa::velocity() * time
}

fn distance_to(p: Vec2) -> f64 {
    (p - oa::position()).length()
}
fn heading_to(p: Vec2) -> f64 {
    (p - oa::position()).angle()
}
fn elapsed(time: f64) -> f64 {
    oa::current_time() - time
}

pub struct F64Ord(pub f64);

impl Eq for F64Ord {}
impl PartialEq for F64Ord {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}
impl Ord for F64Ord {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}
impl PartialOrd for F64Ord {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
