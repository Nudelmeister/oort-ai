use crate::{elapsed, position_in, radar::scan_error_heuristic, roots, F64Ord};
use oort_api::{
    prelude::{self as oa, maths_rs::prelude::Base, ScanResult, Vec2, Vec2Extras},
    Class,
};

pub const fn class_color(class: Class) -> u32 {
    match class {
        Class::Fighter => 0xFFFFFF,
        Class::Frigate => 0xFFFFFF,
        Class::Cruiser => 0xFFFFFF,
        Class::Asteroid => 0xFFFFFF,
        Class::Target => 0xFFFFFF,
        Class::Missile => 0xFF0000,
        Class::Torpedo => 0xFF0000,
        Class::Unknown => 0xFFFFFF,
    }
}
pub const fn class_timeout(class: Class) -> f64 {
    match class {
        Class::Fighter => 2.5,
        Class::Frigate => 5.0,
        Class::Cruiser => 5.0,
        Class::Asteroid => 5.0,
        Class::Target => 2.5,
        Class::Missile => 0.75,
        Class::Torpedo => 0.75,
        Class::Unknown => 2.5,
    }
}

pub const fn class_radius(class: Class) -> f64 {
    match class {
        Class::Fighter => 15.0,
        Class::Frigate => 0.,
        Class::Cruiser => 0.,
        Class::Asteroid => 0.,
        Class::Target => 0.,
        Class::Missile => 7.5,
        Class::Torpedo => 15.0,
        Class::Unknown => 0.,
    }
}

#[derive(Clone)]
pub struct Track {
    state: TrackState,
    last_seen: f64,
    id: u64,
    class: Class,
}

impl Track {
    pub fn new(id: u64, contact: &ScanResult) -> Self {
        let error = scan_error_heuristic(contact.rssi, contact.snr);
        let error = Vec2::new(error, error);
        Self {
            state: TrackState {
                p: contact.position,
                v: contact.velocity,
                a: Vec2::zero(),
                p_c: error,
                v_c: error,
                a_c: error,
            },
            last_seen: oa::current_time(),
            id,
            class: contact.class,
        }
    }

    pub fn debug(&self) {
        let p = self.pos();
        let error = self.error();
        oa::draw_polygon(p, error, 16, 0.0, 0xFFA0A0);
        oa::draw_polygon(
            p,
            class_radius(self.class()),
            8,
            0.0,
            class_color(self.class()),
        );
        let maxt = 5.0 * class_timeout(self.class);
        let dt = maxt / 15.0;
        let mut x = p;
        for i in 1..=15 {
            let t = dt * i as f64;
            let xnext = self.pva_in(t).0;
            oa::draw_line(x, xnext, 0xA0A0A0);
            x = xnext;
        }
    }

    pub fn closest_approach_offset(&self) -> Option<(f64, Vec2)> {
        self.closest_approach()
            .map(|time| (time, self.pos_in(time) - position_in(time)))
    }

    pub fn closest_approach(&self) -> Option<f64> {
        let p = self.pos() - oa::position();
        let v = self.vel() - oa::velocity();
        let a = self.acc();

        // d(t) = |t^2 * 0.5 * a + t * v + p|
        // d(t) = (t^2 * 0.5 * ax + t * vx + px)^2 + (t^2 * 0.5 * ay + t * vy + py)^2
        //
        // d(t) = (t^2 * 0.5 * ax + t * vx + px) * (t^2 * 0.5 * ax + t * vx + px) + ...
        // d(t) = t^4*0.25*ax^2 + t^3*ax*vx + t^2*(ax*px + vx^2) + t*2*vx*px + px^2
        //      + t^4*0.25*ay^2 + t^3*ay*vy + t^2*(ay*py + vy^2) + t*2*vy*py + py^2
        // d(t) = t^4*0.25*(ax^2+ay^2) + t^3*(ax*vx+ay*vy) + t^2*(ax*px + vx^2 + ay*py + vy^2) + t*2*(vx*px+vy*py) + px^2 + py^2
        // d'(t) = t^3*(ax^2+ay^2) + t^2*3*(ax*vx+ay*vy) + t*2*(ax*px + vx^2 + ay*py + vy^2) + 2*(vx*px+vy*py)
        // d''(t) = t^2*3*(ax^2+ay^2) + t*6*(ax*vx+ay*vy) + 2*(ax*px + vx^2 + ay*py + vy^2)

        let is_closest = |t: f64| {
            0.0 <= t.powi(2) * 3.0 * (a.x.powi(2) + a.y.powi(2))
                + t * 6.0 * (a.x * v.x + a.y * v.y)
                + 2.0 * (a.x * p.x + a.y * p.y + v.x.powi(2) + v.y.powi(2))
        };

        let a3 = a.x.powi(2) + a.y.powi(2);
        let a2 = 3.0 * (a.x * v.x + a.y * v.y);
        let a1 = 2.0 * (a.x * p.x + a.y * p.y + v.x.powi(2) + v.y.powi(2));
        let a0 = 2.0 * (v.x * p.x + v.y * p.y);

        let roots = roots::find_roots_cubic(a3, a2, a1, a0);
        roots
            .as_ref()
            .iter()
            .copied()
            .filter(|r| *r > 0.0)
            .filter(|r| is_closest(*r))
            .min_by_key(|r| F64Ord(*r))
    }
    pub fn intercept_aim_pos(&self, c_vel: f64, c_acc: f64) -> Option<(f64, Vec2)> {
        self.intercept(c_vel, c_acc)
            .map(|time| (time, self.pos_in(time) - oa::velocity() * time))
    }
    pub fn intercept(&self, c_vel: f64, c_acc: f64) -> Option<f64> {
        let (pt, vt, a) = self.pva();
        let p = pt - oa::position();
        let v = vt - oa::velocity();

        // 0 =
        //     t^4 * 0.25 * (a.a - c_acc^2)
        //   + t^3 * a.v
        //   + t^2 * (a.p+v.v-c_vel^2)
        //   + t   * 2 * p.v
        //   +       p.p
        let a4 = 0.25 * (a.dot(a) - c_acc.powi(2));
        let a3 = a.dot(v);
        let a2 = a.dot(p) + v.dot(v) - c_vel.powi(2);
        let a1 = 2.0 * p.dot(v);
        let a0 = p.dot(p);

        let roots = super::roots::find_roots_quartic(a4, a3, a2, a1, a0);

        roots
            .as_ref()
            .iter()
            .copied()
            .filter(|r| *r > 0.0)
            .min_by_key(|r| F64Ord(*r))
        //.map(|t| t - oa::TICK_LENGTH)
    }

    pub fn same_track(&self, other: &Self) -> bool {
        if self.class() != other.class() {
            return false;
        }
        let (p, v, a) = self.pva();
        let (op, ov, oa) = other.pva();

        class_radius(self.class()) * 2.0
            > (0.85 * p.distance(op) + 0.1 * v.distance(ov) + 0.05 * a.distance(oa))
                - (self.error() + other.error())
    }
    pub fn timed_out(&self) -> bool {
        elapsed(self.last_seen) > class_timeout(self.class)
    }

    pub fn track_update(&mut self, other: &Self) {
        let (op, ov, _) = other.pva();
        let noise = other.error();
        self.state.update(op, ov, noise, elapsed(self.last_seen));
        self.last_seen = oa::current_time();
        self.id = self.id().min(other.id());
    }
    pub fn scan_update(&mut self, contact: &ScanResult) {
        if contact.class != self.class {
            return;
        }
        let noise = scan_error_heuristic(contact.rssi, contact.snr);
        self.state.update(
            contact.position,
            contact.velocity,
            noise,
            elapsed(self.last_seen),
        );
        self.last_seen = oa::current_time();
    }
    pub fn last_seen(&self) -> f64 {
        self.last_seen
    }
    pub fn id(&self) -> u64 {
        self.id
    }
    pub fn class(&self) -> Class {
        self.class
    }
    pub fn pos(&self) -> Vec2 {
        self.pos_in(0.0)
    }
    pub fn vel(&self) -> Vec2 {
        self.vel_in(0.0)
    }
    pub fn acc(&self) -> Vec2 {
        self.acc_in(0.0)
    }
    pub fn pos_in(&self, time: f64) -> Vec2 {
        self.state.predict_pos(elapsed(self.last_seen) + time)
    }
    pub fn vel_in(&self, time: f64) -> Vec2 {
        self.state.predict_vel(elapsed(self.last_seen) + time)
    }
    pub fn acc_in(&self, _: f64) -> Vec2 {
        self.state.a
    }
    pub fn pva(&self) -> (Vec2, Vec2, Vec2) {
        self.pva_in(0.0)
    }
    pub fn pva_in(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        (self.pos_in(time), self.vel_in(time), self.acc_in(time))
    }
    pub fn error(&self) -> f64 {
        self.error_in(0.0)
    }
    pub fn error_in(&self, time: f64) -> f64 {
        fn max_comp(v: Vec2) -> f64 {
            v.x.max(v.y)
        }
        let (p_c, v_c, a_c) = self.state.predict_cov(elapsed(self.last_seen) + time);
        25.0 * max_comp(p_c).max(max_comp(v_c)).max(max_comp(a_c))
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
    fn predict_pos(&self, time: f64) -> Vec2 {
        self.p + self.v * time + self.a * time.powi(2) * 0.5
    }
    fn predict_vel(&self, time: f64) -> Vec2 {
        self.v + self.a * time
    }
    fn predict_cov(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        let t = time;
        let p_noise = Vec2::from(0.166_667 * t.powi(3));
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

    fn update(&mut self, pos: Vec2, vel: Vec2, noise: f64, time: f64) {
        let time = time.max(oa::TICK_LENGTH);
        let acc = (vel - self.v) / time;
        let p = self.predict_pos(time);
        let v = self.predict_vel(time);
        let a = self.a;
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
}
