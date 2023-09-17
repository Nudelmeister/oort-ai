use super::{direction_to, elapsed, position_in};
use oort_api::prelude::{maths_rs::prelude::Base, *};
use std::f64::consts::SQRT_2;

fn class_color(class: Class) -> u32 {
    match class {
        Class::Fighter => 0xFF00_FFFF,
        Class::Frigate => 0xFFFF_FF00,
        Class::Cruiser => 0xFFFF_00FF,
        Class::Asteroid => 0xFF00_FF00,
        Class::Target | Class::Unknown => 0xFFFF_FFFF,
        Class::Missile | Class::Torpedo => 0xFFFF_0000,
    }
}

fn class_priority(class: Class) -> f64 {
    match class {
        Class::Cruiser => 0.9,
        Class::Frigate => 0.7,
        Class::Fighter => 0.5,
        Class::Torpedo => 0.4,
        Class::Missile => 0.3,
        Class::Target => 0.2,
        Class::Unknown => 0.1,
        Class::Asteroid => 0.,
    }
}

fn inverse_prio(value: f64, half_point: f64) -> f64 {
    half_point / (value + half_point)
}

fn missile_priority(track: &Track) -> f64 {
    const DEATH_DIST: f64 = 50.;
    const DANGER_DIST: f64 = 150.;
    const DANGER_TIME: f64 = 10.;
    const DIST_FACTOR: f64 = 0.6;

    if let Some(intercept_time) = track.impact_time(0.) {
        let intercept_dist = track
            .position_in(intercept_time)
            .distance(position_in(intercept_time));
        (1. - DIST_FACTOR) * inverse_prio(intercept_time, DANGER_TIME)
            + DIST_FACTOR * inverse_prio((intercept_dist - DEATH_DIST).max(0.), DANGER_DIST)
    } else {
        0.
    }
}

#[derive(Clone)]
pub struct Track {
    pub id: u32,
    pub class: Class,
    pub last_seen: f64,
    state: TrackState,
}

impl Track {
    pub fn priority(&self) -> f64 {
        let time = elapsed(self.last_seen);
        let dist = self.distance();
        if matches!(self.class, Class::Missile | Class::Torpedo) {
            return missile_priority(self);
        }
        class_priority(self.class) * inverse_prio(dist, 5000.) * inverse_prio(time, 10.)
    }
    pub fn from_contact(id: u32, contact: &ScanResult) -> Self {
        let noise = Vec2::from(0.5 * contact.snr.recip() * -contact.rssi);
        Self {
            id,
            class: contact.class,
            last_seen: current_time(),
            state: TrackState {
                p: contact.position,
                v: contact.velocity,
                a: Vec2::zero(),
                p_c: noise,
                v_c: noise,
                a_c: noise,
            },
        }
    }
    pub fn with_uncertanty(id: u32, class: Class, pos: Vec2, vel: Vec2, uncertanty: f64) -> Self {
        Self {
            id,
            class,
            last_seen: current_time(),
            state: TrackState {
                p: pos,
                v: vel,
                a: Vec2::zero(),
                p_c: Vec2::one() * uncertanty / SQRT_2,
                v_c: Vec2::one() * uncertanty / SQRT_2,
                a_c: Vec2::one() * uncertanty / SQRT_2,
            },
        }
    }
    pub fn new(id: u32, class: Class, pos: Vec2, vel: Vec2) -> Self {
        Self {
            id,
            class,
            last_seen: current_time(),
            state: TrackState {
                p: pos,
                v: vel,
                a: Vec2::zero(),
                p_c: Vec2::one(),
                v_c: Vec2::one(),
                a_c: Vec2::one(),
            },
        }
    }
    pub fn debug(&self) {
        let (p, v, a) = self.state.predict_pva(elapsed(self.last_seen));
        let color = class_color(self.class);
        draw_square(self.state.p, 100., color);
        draw_diamond(p, 100., color);
        draw_line(p, p + v, color);
        draw_polygon(p + a, 25., 8, 0., color);
        draw_text!(
            p + Vec2::new(100., 100.),
            color,
            "{} {:.2}",
            self.id,
            self.priority()
        );
    }
    pub fn pva(&self) -> (Vec2, Vec2, Vec2) {
        self.pva_in(0.)
    }
    pub fn pva_in(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        self.state.predict_pva(elapsed(self.last_seen) + time)
    }
    pub fn position(&self) -> Vec2 {
        self.position_in(0.)
    }
    pub fn position_in(&self, time: f64) -> Vec2 {
        let (p, _, _) = self.state.predict_pva(elapsed(self.last_seen) + time);
        p
    }
    pub fn velocity(&self) -> Vec2 {
        let (_, v, _) = self.state.predict_pva(elapsed(self.last_seen));
        v
    }
    pub fn closing_speed(&self) -> f64 {
        let dir = self.direction();
        velocity().dot(dir) + self.velocity().dot(-dir)
    }
    pub fn distance(&self) -> f64 {
        self.position().distance(position())
    }
    pub fn direction(&self) -> Vec2 {
        (self.position() - position()).normalize()
    }
    pub fn heading(&self) -> f64 {
        self.direction().angle()
    }
    /// heading, area, distance
    pub fn radar_track_look(&self) -> (f64, f64, f64) {
        let (p, _, _) = self
            .state
            .predict_pva(elapsed(self.last_seen) + 2. * TICK_LENGTH);
        let towards = p - position();
        (towards.angle(), 150. * self.uncertanty(), towards.length())
    }
    pub fn impact_time(&self, extra_speed: f64) -> Option<f64> {
        const PREC_SUB_STEPS: i32 = 10;
        const PREC_STEPS: i32 = 4;

        let (p, v, _) = self.pva();

        let p_dir = direction_to(p);

        let closing_speed = velocity().dot(p_dir) + v.dot(-p_dir) + extra_speed;
        if closing_speed.is_sign_negative() {
            return None;
        }
        let mut closest_time = p.distance(position()) / closing_speed;
        let track_pos = self.position_in(closest_time);
        let track_dir = direction_to(track_pos);
        let ship_pos = position_in(closest_time) + track_dir * extra_speed * closest_time;
        let mut closest_dist = track_pos.distance(ship_pos);

        for precision in 1..=PREC_STEPS {
            let precision = ((PREC_SUB_STEPS + 1) as f64).powi(precision).recip();
            let time = closest_time;
            let mut last_d = f64::MAX;
            for t in -PREC_SUB_STEPS..=PREC_SUB_STEPS {
                if t == 0 {
                    continue;
                }
                let t = time + precision * time * t as f64;
                let track_pos = self.position_in(t);
                let track_dir = direction_to(track_pos);
                let ship_pos = position_in(t) + track_dir * extra_speed * t;
                let d = track_pos.distance(ship_pos);
                if last_d < d {
                    break;
                }
                last_d = d;
                if d < closest_dist {
                    //draw_diamond(ship_pos, precision * 10000., 0xFFFF_FFFF);
                    //draw_diamond(track_pos, precision * 10000., 0xFFFF_0000);
                    closest_dist = d;
                    closest_time = t;
                }
            }
        }

        closest_time.is_sign_positive().then_some(closest_time)
    }
    pub fn projectile_lead_heading(&self, projectile_speed: f64) -> Option<f64> {
        self.impact_time(projectile_speed)
            .map(|t| ((self.position_in(t) - velocity() * t) - position()).angle())
    }
    pub fn update(&mut self, observ: &ScanResult) {
        self.state.update(observ, elapsed(self.last_seen));
        self.class = observ.class;
        self.last_seen = current_time();
    }
    pub fn update_fixed_noise(&mut self, pos: Vec2, vel: Vec2, noise: f64) {
        self.state
            .update_fixed_noise(pos, vel, noise, elapsed(self.last_seen));
        self.last_seen = current_time();
    }

    pub fn uncertanty(&self) -> f64 {
        let (p_c, v_c, a_c) = self.state.predict_cov(elapsed(self.last_seen));
        p_c.length() + v_c.length() + a_c.length()
    }
}

#[derive(Clone)]
struct TrackState {
    p: Vec2,
    v: Vec2,
    a: Vec2,
    p_c: Vec2,
    v_c: Vec2,
    a_c: Vec2,
}

impl TrackState {
    fn predict_pva(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        let t = time;
        (
            self.p + self.v * t + self.a * t.powi(2) * 0.5,
            self.v + self.a * t,
            self.a,
        )
    }
    fn predict_cov(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        let t = time;
        let p_noise = Vec2::from((1. / 6.) * t.powi(3));
        let v_noise = Vec2::from(0.5 * t.powi(2));
        let a_noise = Vec2::from(t);
        let p_c = self.p_c + self.v_c * t + self.a_c * t.powi(2) * 0.5;
        let v_c = self.v_c + self.a_c * t;
        let a_c = self.a_c;
        (
            p_c + p_noise,
            v_c + p_c * t + v_noise,
            a_c + v_c * t + p_c * t.powi(2) * 0.5 + a_noise,
        )
    }

    fn update_fixed_noise(&mut self, pos: Vec2, vel: Vec2, noise: f64, time: f64) {
        let time = if time == 0. { TICK_LENGTH / 2. } else { time };
        let acc = (vel - self.v) / time;
        let (p, v, a) = self.predict_pva(time);
        let (p_c, v_c, a_c) = self.predict_cov(time);

        let innov_pos = pos - p;
        let innov_vel = vel - v;
        let innov_acc = acc - a;
        let innov_pos_cov = p_c + noise;
        let innov_vel_cov = v_c + noise;
        let innov_acc_cov = a_c + noise;

        // 1. / is component wise
        let gain_pos = p_c * (1. / innov_pos_cov);
        let gain_vel = v_c * (1. / innov_vel_cov);
        let gain_acc = a_c * (1. / innov_acc_cov);

        // * is component wise
        self.p = p + gain_pos * innov_pos;
        self.v = v + gain_vel * innov_vel;
        self.a = a + 0.1 * gain_acc * innov_acc; // Fudge to dampen acc, otherwise its occilating a lot

        self.p_c = (Vec2::one() - gain_pos) * p_c;
        self.v_c = (Vec2::one() - gain_vel) * v_c;
        self.a_c = (Vec2::one() - gain_acc) * a_c;
    }
    fn update(&mut self, observ: &ScanResult, time: f64) {
        let noise = observ.snr.recip() * -observ.rssi;
        self.update_fixed_noise(observ.position, observ.velocity, noise, time);
    }
}
