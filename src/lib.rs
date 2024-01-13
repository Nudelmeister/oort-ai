//mod intel;
//mod liblib;
//mod message;
mod missile;
mod radar;
mod radar_util;
mod roots;
mod track;

use std::f64::consts::TAU;

use fighter::Fighter;
use missile::Missile;
use oort_api::{
    prelude::{self as oa, Vec2, Vec2Extras},
    Class,
};

mod fighter;

pub enum Ship {
    Fighter(Fighter),
    Missile(Missile),
}

impl Ship {
    pub fn new() -> Self {
        match oa::class() {
            Class::Fighter => Self::Fighter(Fighter::new()),
            Class::Missile => Self::Missile(Missile::new()),
            _ => unimplemented!(),
        }
    }

    pub fn tick(&mut self) {
        match self {
            Self::Fighter(s) => s.tick(),
            Self::Missile(s) => s.tick(),
        }
    }
}

impl Default for Ship {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Aimbot {
    last_heading: f64,
    last_time: f64,
}

impl Aimbot {
    pub fn new() -> Self {
        Self {
            last_heading: 0.0,
            last_time: 0.0,
        }
    }
    pub fn aim_at_torque(&mut self, target_pos: Vec2, var_radius: f64) -> f64 {
        let var_period = 0.5;
        let target_pos =
            target_pos + Vec2::new(var_radius, 0.0).rotate(TAU * oa::current_time() / var_period);

        let to_t = target_pos - oa::position();
        let diff = oa::angle_diff(oa::heading(), to_t.angle());

        let rel_rot = oa::angle_diff(self.last_heading, diff) / elapsed(self.last_time);

        self.last_heading = diff;
        self.last_time = oa::current_time();

        //oa::draw_line(oa::position(), target_pos, 0x00a000);
        //oa::draw_line(
        //    oa::position(),
        //    oa::position() + Vec2::new(10000.0, 0.0).rotate(oa::heading()),
        //    0x00a0a0,
        //);

        oa::max_angular_acceleration()
            * (100.0 * diff + 10.0 * rel_rot.signum() * rel_rot.powi(2) + 1.0 * rel_rot)
    }
}

impl Default for Aimbot {
    fn default() -> Self {
        Self::new()
    }
}

fn position_in(time: f64) -> Vec2 {
    oa::position() + oa::velocity() * time
}

fn direction_to(p: Vec2) -> Vec2 {
    (p - oa::position()).normalize()
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
