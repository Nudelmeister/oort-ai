use crate::{
    heading_to, position_in,
    radar::Radar,
    track::{class_radius, Track},
};
use oort_api::{
    prelude::{self as oa, angle_diff, Vec2, Vec2Extras},
    Class,
};
use std::f64::consts::TAU;

pub struct Fighter {
    radar: Radar,
    last_target: Option<u64>,
}

impl Fighter {
    pub fn new() -> Self {
        Self {
            radar: Radar::new(0.01 * TAU),
            last_target: None,
        }
    }
    pub fn tick(&mut self) {
        self.radar.tick();
        if oa::current_time() < (0.05 / oa::TICK_LENGTH).recip() * 2.0 {
            return;
        }
        //if (oa::current_tick() / 600) % 2 == 0 {
        //    oa::accelerate(Vec2::new(100.0, 0.0));
        //} else {
        //    oa::accelerate(Vec2::new(-100.0, 0.0));
        //}
        for track in self.radar.tracks() {
            let (t, cp) = track.closest_approach();
            let p = position_in(t);
            let d = cp - p;
            if d.length() < 10.0 * class_radius(oa::class()) {
                oa::accelerate(-100.0 * d.normalize());
            }
            oa::draw_square(cp, 25.0, 0x00A000);
            oa::draw_square(p, 25.0, 0xA0A000);
            oa::draw_line(cp, p, 0xA0A0A0);
            oa::draw_text!(cp, 0xA0FFA0, "{}: {t:.2}", track.id());
        }

        let x;
        if let Some(t) = self.radar.get_target(|t| target_prio(t, self.last_target)) {
            x = heading_to(t.pos());
            self.last_target = Some(t.id());
            if let Some((impact_time, impact_pos)) = t.intercept_aim_pos(1000.0) {
                oa::torque(aim_torque(impact_pos, t.vel_in(impact_time)));
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
            self.radar
                .set_limits(Some((x + TAU * 0.125, x - TAU * 0.125)));
        } else {
            self.radar.set_limits(None);
        }
    }
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

fn missile_prio(track: &Track) -> f64 {
    let (time, pos) = track.closest_approach();
    20.0 * class_radius(oa::class()) / pos.distance(oa::position() + oa::velocity() * time)
        * (time / 2.0 + 0.5).recip()
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
        * (150.0 * (diff + var) + 10.0 * rel_rot.signum() * rel_rot.powi(2) + 0.25 * rel_rot)
}
