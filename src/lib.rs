use oort_api::prelude::*;

const BULLET_SPEED: f64 = 1000.0; // m/s

pub struct Ship {
    target: TargetInfo,
    track: RadarTrack,
    search: RadarSearch,
    tracking: bool,
    aim: AimPid,
}

impl Default for Ship {
    fn default() -> Self {
        Self::new()
    }
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            target: TargetInfo::default(),
            track: RadarTrack {
                last_contact_time: 0.,
                err_per_sec: 2000.,
            },
            search: RadarSearch {
                targets: Vec::new(),
                target_fuse_dist: 100.,
                target_timeout: 2.,
            },
            tracking: false,
            aim: AimPid::new(100., 0., -110_000.),
        }
    }

    pub fn tick(&mut self) {
        if self.tracking {
            let (contact, track_lost) = self.track.track(self.target.position());
            self.tracking = !track_lost;
            if let Some(contact) = contact {
                self.target
                    .integrate_data(contact.position, contact.velocity, 0.75);
            }
            self.target.debug();
        } else {
            self.search.search();
            if let Some(target) = self.search.in_front_target() {
                self.tracking = true;
                self.track.last_contact_time = current_time();
                self.target = *target;
                self.aim.reset();
            }
        }
        let bullet_impact_time = projectile_impact_time2(&self.target, BULLET_SPEED);
        let aim_pos = rel_vel_target_pos(&self.target, bullet_impact_time);
        let turn_torque = self.aim.aim_torque((aim_pos - position()).angle());
        torque(turn_torque);
        if angle_diff(heading(), (aim_pos - position()).angle()).abs() < 0.01 * TAU {
            fire(0);
        }

        kite(&self.target, 3500., 200.);

        debug!("tracking {}", self.tracking);
        draw_diamond(aim_pos, 5., 0xFFFF_0000);
        draw_diamond(
            self.target.position_in(bullet_impact_time),
            10.,
            0xFFFF_FF00,
        );
        draw_line(
            position(),
            position() + vec2(100_000., 0.).rotate(heading()),
            0xFFFF_FFFF,
        );
    }
}

fn kite(target: &TargetInfo, distance: f64, desired_side_speed: f64) {
    let target_dir = dir(position(), target.position());
    let towards_speed = (velocity() - target.vel).dot(target_dir);
    let side_speed = (velocity() - target.vel).dot(target_dir.rotate(0.25 * TAU));

    let target_dist = position().distance(target.position());
    let desired_towards_speed = (target_dist - distance) / 10.;

    let towards_acc = target_dir * (desired_towards_speed - towards_speed);
    let side_acc = target_dir.rotate(0.25 * TAU) * (desired_side_speed - side_speed);

    let acc = (towards_acc + side_acc) * max_forward_acceleration();
    accelerate(acc);
}

struct RadarSearch {
    targets: Vec<TargetInfo>,
    target_fuse_dist: f64,
    target_timeout: f64,
}
impl RadarSearch {
    const FOV: f64 = 0.05 * TAU;
    const HEADING_STEP: f64 = 0.25 * TAU + 0.5 * Self::FOV;

    fn search(&mut self) {
        self.targets.iter().for_each(TargetInfo::debug);
        set_radar_width(Self::FOV);
        set_radar_heading(radar_heading() + Self::HEADING_STEP);
        set_radar_min_distance(0.);
        set_radar_max_distance(f64::MAX);

        if let Some(contact) = scan() {
            if let Some(fuse_target) = self
                .targets
                .iter_mut()
                .find(|t| t.position().distance(contact.position) < self.target_fuse_dist)
            {
                fuse_target.integrate_data(contact.position, contact.velocity, 1.);
            } else {
                self.targets.push(TargetInfo {
                    pos: contact.position,
                    vel: contact.velocity,
                    acc: vec2(0., 0.),
                    t: current_time(),
                });
            }
        }

        self.targets
            .retain(|t| delta_time(t.t) < self.target_timeout);
    }

    fn close_target(&self) -> Option<&TargetInfo> {
        self.targets
            .iter()
            .min_by_key(|t| (t.position().distance(position()) * 1000.) as i64)
    }
    fn in_front_target(&self) -> Option<&TargetInfo> {
        self.targets.iter().min_by_key(|t| {
            (angle_diff(heading(), dir(position(), t.position()).angle() * 100_000.)) as i64
        })
    }
}

struct RadarTrack {
    last_contact_time: f64,
    err_per_sec: f64,
}

impl RadarTrack {
    fn track(&mut self, target: Vec2) -> (Option<ScanResult>, bool) {
        let towards_target = target - position();
        set_radar_heading(towards_target.angle());

        let distance = towards_target.length();
        let err = delta_time(self.last_contact_time) * self.err_per_sec;

        debug!("err: {}, md: {}", err, distance + 100. + err);

        set_radar_max_distance(distance + 100. + err);
        set_radar_min_distance(distance - 100. - err);
        set_radar_width(0.5 * TAU * (50. + err) / distance);

        let contact = scan();
        if contact.is_some() {
            self.last_contact_time = current_time();
        }
        (contact, err > 1000.)
    }
}

struct RadarSweep {
    max_angle: f64,
    min_fov: f64,
    max_fov: f64,
    fov_step: f64,
}
impl RadarSweep {
    fn sweep(&mut self, target: Vec2) -> Option<ScanResult> {
        let target_angle = (target - position()).angle();
        let angle_to_target = angle_diff(radar_heading(), target_angle);

        set_radar_max_distance(f64::MAX);
        set_radar_min_distance(0.);

        if radar_width() < self.min_fov {
            set_radar_heading(target_angle);
            set_radar_width(self.max_fov);
        } else if 2. * angle_to_target.abs() > self.max_angle {
            set_radar_heading(target_angle);
            set_radar_width(radar_width() - self.fov_step);
        } else {
            set_radar_heading(
                radar_heading()
                    + 2. * angle_to_target
                    + 0.5 * angle_to_target.signum() * radar_width(),
            );
        }

        scan()
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct TargetInfo {
    pos: Vec2,
    vel: Vec2,
    acc: Vec2,
    t: f64,
}

impl TargetInfo {
    fn debug(&self) {
        draw_diamond(self.position(), 100., 0xFFFF_FFFF);
        draw_triangle(self.pos, 100., 0xFFFF_FFFF);
        draw_line(self.pos, self.pos + self.vel, 0xFFFF_FFFF);
        draw_line(self.pos, self.pos + self.acc, 0xFFC8_6400);
        draw_text!(self.pos, 0xFF64_6464, "{}", delta_time(self.t));
    }

    fn integrate_data(&mut self, position: Vec2, velocity: Vec2, importance: f64) {
        assert!((0. ..=1.).contains(&importance));
        let p_importance = importance;
        let v_importance = 0.75 * p_importance;
        let a_importance = 0.75 * v_importance;
        let p_inv_importance = 1. - p_importance;
        let v_inv_importance = 1. - v_importance;
        let a_inv_importance = 1. - a_importance;

        self.pos = self.pos * p_inv_importance + position * p_importance;

        let old_velocity = self.vel;
        self.vel = self.vel * v_inv_importance + velocity * v_importance;

        let acceleration = (self.vel - old_velocity) / delta_time(self.t);
        if acceleration.x.is_finite() && acceleration.y.is_finite() {
            let acc_damp = (self.acc.distance(acceleration).recip() * 100.).clamp(0.001, 0.999);
            self.acc = self.acc * a_inv_importance + acceleration * acc_damp * a_importance;
        }

        self.t = current_time();
    }

    fn position(&self) -> Vec2 {
        self.position_in(0.)
    }
    fn position_in(&self, time: f64) -> Vec2 {
        let delta_t = delta_time(self.t) + time;
        self.pos + self.vel * delta_t + self.acc * delta_t * delta_t * 0.5
    }
}

fn rel_vel_target_pos(target: &TargetInfo, time: f64) -> Vec2 {
    target.position_in(time) - velocity() * time
}

fn delta_time(time: f64) -> f64 {
    current_time() - time
}

fn projectile_impact_time2(target: &TargetInfo, projectile_speed: f64) -> f64 {
    let mut time = 0.;
    for _ in 0..20 {
        time = rel_vel_target_pos(target, time).distance(position()) / projectile_speed;
    }

    time.max(0.001).min(100.)
}

fn dir(from: Vec2, to: Vec2) -> Vec2 {
    (to - from).normalize()
}

struct AimPid {
    p: f64,
    i: f64,
    d: f64,
    last_diff: f64,
    accumulated_diff: f64,
    last_time: f64,
}

impl AimPid {
    fn new(p: f64, i: f64, d: f64) -> Self {
        Self {
            p,
            i,
            d,
            last_diff: 0.,
            accumulated_diff: 0.,
            last_time: current_time(),
        }
    }
    fn reset(&mut self) {
        self.last_diff = 0.;
        self.accumulated_diff = 0.;
        self.last_time = current_time();
    }
    fn aim_torque(&mut self, aim: f64) -> f64 {
        let diff = angle_diff(heading(), aim);
        let diff_diff = angle_diff(diff, self.last_diff);
        let delta = delta_time(self.last_time);

        let p_term = self.p * diff;
        let i_term = self.accumulated_diff;
        let d_term = self.d * diff_diff * delta;

        self.last_diff = diff;
        self.accumulated_diff += self.i * diff * delta;
        self.last_time = current_time();
        debug!("p {}, i {}, d {}", p_term, i_term, d_term);

        p_term + d_term
    }
}
