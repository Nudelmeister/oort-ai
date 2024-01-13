use crate::{
    direction_to, heading_to,
    missile::{self, MissileProgram},
    radar::Radar,
    track::{class_radius, Track},
    Aimbot,
};
use oort_api::{
    prelude::{
        self as oa, angle_diff, current_time, fire, maths_rs::prelude::Base, Vec2, Vec2Extras,
    },
    Class,
};
use std::f64::consts::TAU;

pub struct Fighter {
    radar: Radar,
    last_target: Option<u64>,
    aimbot: Aimbot,
    mover: Mover,
}

impl Fighter {
    pub fn new() -> Self {
        Self {
            radar: Radar::new(0.01 * TAU),
            last_target: None,
            aimbot: Aimbot::new(),
            mover: Mover::new(2.5),
        }
    }
    pub fn tick(&mut self) {
        self.mover.tick(1.0, MoveCmd::Point(Vec2::new(0.0, 0.0)));
        oa::draw_line(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), 0xFF0000);
        oa::draw_line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 10.0), 0x00FF00);

        //oa::fire(1);
        self.radar.tick();
        if oa::current_time() < (0.05 / oa::TICK_LENGTH).recip() * 2.0 {
            return;
        }
        //if (oa::current_tick() / 600) % 2 == 0 {
        //    oa::accelerate(Vec2::new(100.0, 0.0));
        //} else {
        //    oa::accelerate(Vec2::new(-100.0, 0.0));
        //}

        if let Some(f) = self
            .radar
            .tracks()
            .iter()
            .find(|t| t.class() != Class::Missile)
        {
            if oa::reload_ticks(1) == 0 {
                oa::fire(1);
                oa::send_bytes(
                    &MissileProgram::Target {
                        //point: f.pos_in(7.5)
                        //    + Vec2::new(oa::rand(-3000.0, 3000.0), oa::rand(-3000.0, 3000.0))
                        //    - 0.5 * (f.pos() - oa::position()),
                        //t: oa::current_time() + oa::rand(5.0, 15.0),
                        pos: f.pos(),
                        vel: f.vel(),
                    }
                    .to_message(),
                );
            }
        }

        let x;
        if let Some(t) = self.radar.get_target(|t| target_prio(t, self.last_target)) {
            x = heading_to(t.pos());
            self.last_target = Some(t.id());
            if let Some((impact_time, impact_pos)) = t.intercept_aim_pos(1000.0, 0.0) {
                oa::torque(self.aimbot.aim_at_torque(impact_pos, 10.0));
                oa::draw_triangle(impact_pos, 50.0, 0xFFA0A0);
                oa::draw_line(
                    oa::position(),
                    oa::position() + Vec2::new(1000000.0, 0.0).rotate(oa::heading()),
                    0xFFA0A0,
                );

                let ad = oa::angle_diff(oa::heading(), heading_to(impact_pos)).abs();
                if ad < 0.01 * TAU && impact_time < 10.0 {
                    oa::debug!("Fire");
                    oa::fire(0);
                }
            }
        } else {
            x = 0.0;
            self.last_target = None;
        }
        if self.last_target.is_some() {
            self.radar.set_cone_search(0.01 * TAU, 0.25 * TAU, x);
        } else {
            self.radar.set_circular_search(0.01 * TAU);
        }
    }
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

struct Mover {
    evade_pos: Vec2,
    evade_vel: Vec2,
    evade_acc: Vec2,
    evade_dur: f64,
    evade_time: f64,
}

enum MoveCmd {
    Heading(f64),
    Point(Vec2),
    Orbit(Vec2, f64),
}

impl Mover {
    pub fn new(evade_dur: f64) -> Self {
        Self {
            evade_pos: Vec2::zero(),
            evade_vel: Vec2::zero(),
            evade_acc: Vec2::zero(),
            evade_time: evade_dur,
            evade_dur,
        }
    }

    fn evade(&mut self, strength: f64) -> Vec2 {
        self.evade_time += oa::TICK_LENGTH;

        let center = (self.evade_pos.length() / (100.0 * strength * self.evade_dur)).min(1.0);
        oa::debug!("{center}");
        if self.evade_time >= self.evade_dur {
            self.evade_time = 0.0;
            let center =
                (self.evade_pos.length() / (100.0 * strength * self.evade_dur)).clamp(0.1, 0.5);
            let ev = if self.evade_pos.length() == 0.0 {
                Vec2::zero()
            } else {
                self.evade_pos.normalize()
            };
            self.evade_acc = oa::max_forward_acceleration()
                * strength
                * (center * -ev + (1.0 - center) * Vec2::new(1.0, 0.0).rotate(oa::rand(0.0, TAU)));
        }

        self.evade_vel += self.evade_acc * oa::TICK_LENGTH;
        self.evade_pos += self.evade_vel * oa::TICK_LENGTH;
        oa::debug!("{:?}", self.evade_pos);
        oa::debug!("{:?}", self.evade_vel);
        oa::debug!("{:?}", self.evade_acc);
        self.evade_acc
    }

    pub fn tick(&mut self, evade_strength: f64, cmd: MoveCmd) {
        let evade = self.evade(evade_strength);

        let acc = match cmd {
            MoveCmd::Heading(heading) => {
                Vec2::new(1.0, 0.0).rotate(heading) * oa::max_forward_acceleration()
            }
            MoveCmd::Point(p) => direction_to(p) * oa::max_forward_acceleration(),
            MoveCmd::Orbit(_, _) => todo!(),
        };

        let avoidance = ((oa::position().length() - 15_000.0) / 5_000.0).clamp(0.0, 1.0);
        oa::debug!("{avoidance}");
        oa::accelerate(
            avoidance * 10.0 * oa::max_forward_acceleration() * direction_to(Vec2::zero())
                + (1.0 - avoidance) * acc
                + evade,
        );
    }
}

fn missile_prio(track: &Track) -> f64 {
    let Some((time, offset)) = track.closest_approach_offset() else {
        return 0.0;
    };
    let danger = ((class_radius(oa::class()) + missile::FRAG_LETHAL_RANGE) / offset.length())
        .powi(2)
        .min(1.0);
    let urgency = (time.max(0.1) / 5.0).recip();

    let max_prio = danger * 25.0;

    oa::debug!("{danger}, {urgency}, {max_prio}");

    (urgency * danger * 10.0).min(max_prio)
}

fn target_prio(track: &Track, last_target_id: Option<u64>) -> f64 {
    let x = (0.5 * TAU - angle_diff(oa::heading(), heading_to(track.pos())).abs()) / (0.5 * TAU)
        + match track.class() {
            Class::Missile | Class::Torpedo => missile_prio(track),
            _ => 1.0,
        }
        + last_target_id.map_or(0.0, |id| if track.id() == id { 1.0 } else { 0.0 });

    oa::draw_text!(track.pos(), 0xFFFFFF, "{:.2}", x);
    x
}

fn aim_torque(target_pos: Vec2, target_vel: Vec2) -> f64 {
    let to_t = target_pos - oa::position();
    let side_dir = to_t.normalize().rotate(0.25 * TAU);
    let rel_rot =
        (target_vel - oa::velocity()).dot(side_dir) / to_t.length() - oa::angular_velocity();

    let diff = angle_diff(oa::heading(), to_t.angle());

    let var_period = (0.2 / oa::TICK_LENGTH).round() as u32;
    let var_magnitude = TAU * 0.002;
    let var_tick = oa::current_tick() % var_period;

    let var = if (oa::current_tick() / var_period) % 2 == 0 {
        var_magnitude * var_tick as f64 / var_period as f64 - 0.5 * var_magnitude
    } else {
        0.5 * var_magnitude - var_magnitude * var_tick as f64 / var_period as f64
    };

    oa::max_angular_acceleration()
        * (150.0 * (diff + var) + 10.0 * rel_rot.signum() * rel_rot.powi(2) + 0.5 * rel_rot)
}
