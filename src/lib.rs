use message::{Msg, MsgKind, QuantState, QuantTrack};
use missile::Missile;
use oort_api::prelude::{maths_rs::prelude::Base, *};
use radar::UnifiedRadar;
use std::collections::VecDeque;

mod message;
mod missile;
mod radar;
mod track;

const BULLET_SPEED: f64 = 1000.;
const BULLET_RANGE: f64 = 10000.;

pub enum Ship {
    Fighter(Fighter),
    Missile(Missile),
}
impl Default for Ship {
    fn default() -> Self {
        Self::new()
    }
}

impl Ship {
    pub fn new() -> Self {
        match class() {
            Class::Fighter => Self::Fighter(Fighter::new()),
            Class::Missile => Self::Missile(Missile::new()),
            _ => unimplemented!(),
        }
    }
    pub fn tick(&mut self) {
        match self {
            Self::Fighter(inner) => inner.tick(),
            Self::Missile(inner) => inner.tick(),
        }
    }
}

pub struct Fighter {
    radar: UnifiedRadar,
    aimbot: AimBot,
    aim_id: u32,
    next_missile_id: u16,
    move_target: Vec2,
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

impl Fighter {
    pub fn new() -> Fighter {
        Fighter {
            radar: UnifiedRadar::new(0.0..=40_000., 0., TAU, TAU * TICK_LENGTH, true),
            aimbot: AimBot::new(70., -70_000.),
            aim_id: 0,
            next_missile_id: 0,
            move_target: Vec2::zero(),
        }
    }

    pub fn tick(&mut self) {
        draw_line(
            position(),
            position() + vec2(100_000., 0.).rotate(heading()),
            0xFFFF_FFFF,
        );
        self.radar.tick();

        if let Some(track) = self.radar.prio_track() {
            if let Some(lead_heading) = track.projectile_lead_heading(BULLET_SPEED) {
                if track.id != self.aim_id {
                    self.aimbot.reset();
                    self.aim_id = track.id;
                }
                let spread_angle = 0.00015 * TAU;
                let spread_ticks = 20;

                let mut spread_deflection = ((current_tick() as i64 % spread_ticks)
                    - spread_ticks / 2) as f64
                    * spread_angle;
                if (current_tick() as i64 / spread_ticks) % 2 == 0 {
                    spread_deflection *= -1.;
                };

                let aim = lead_heading + spread_deflection;
                draw_line(
                    position(),
                    position() + vec2(1000., 0.).rotate(aim),
                    0xFFFF_00FF,
                );

                let lead_torque = self.aimbot.aim_torque(aim);
                torque(lead_torque);
                if angle_diff(heading(), aim) < 0.01 * TAU && track.distance() < BULLET_RANGE {
                    fire(0);
                }
            }
        }

        if let Some(t) = self
            .radar
            .contacts()
            .iter()
            .filter(|c| !matches!(c.class, Class::Asteroid | Class::Missile | Class::Torpedo))
            .max_by_key(|c| (c.priority() * 100000.) as i64)
            .cloned()
        {
            // Priority Not Gun Target
            self.radar.scan_direction = t.direction().angle();
            self.radar.scan_angle_max = 0.1 * TAU;

            if reload_ticks(1) == 0 {
                fire(1);
                Msg::send(MsgKind::SenderTgt(QuantTrack::from(t)));
            } else if current_tick() % 15 == 0 {
                Msg::send(MsgKind::SenderTgt(QuantTrack::from(t)));
            }
        }

        if current_tick() % 600 == 0 {
            self.move_target = vec2(
                rand(-world_size(), world_size()),
                rand(-world_size(), world_size()),
            ) * 0.4;
        }

        let stop_pos = position_in(velocity().length() / max_lateral_acceleration());
        if stop_pos.x.abs() >= world_size() * 0.4 || stop_pos.x.abs() >= world_size() * 0.4 {
            accelerate(-position());
        } else {
            accelerate(self.move_target - position());
            draw_triangle(self.move_target, 1000., 0xFF00_FF00);
        }
    }
}

fn direction_to(pos: Vec2) -> Vec2 {
    (pos - position()).normalize()
}

fn position_in(time: f64) -> Vec2 {
    position() + velocity() * time
}

fn elapsed(time: f64) -> f64 {
    current_time() - time
}

struct AimBot {
    p: f64,
    d: f64,
    last_diff: f64,
    last_time: f64,
}

impl AimBot {
    fn new(p: f64, d: f64) -> Self {
        Self {
            p,
            d,
            last_diff: 0.,
            last_time: current_time(),
        }
    }
    fn reset(&mut self) {
        self.last_diff = 0.;
        self.last_time = current_time();
    }
    fn aim_torque(&mut self, aim: f64) -> f64 {
        let diff = angle_diff(heading(), aim);
        let diff_diff = angle_diff(diff, self.last_diff);
        let delta = elapsed(self.last_time);

        let p_term = self.p * diff;
        let d_term = self.d * diff_diff * delta;

        self.last_diff = diff;
        self.last_time = current_time();
        p_term + d_term
    }
}
