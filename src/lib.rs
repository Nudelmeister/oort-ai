use oort_api::prelude::*;

const BULLET_SPEED: f64 = 1000.0; // m/s

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

pub struct Missile {
    long_range_radar: LongRangeSearchAndTrack,
    target_received: bool,
    closest_dist: f64,
}
impl Default for Missile {
    fn default() -> Self {
        Self::new()
    }
}

impl Missile {
    const FUSE_DISTANCE: f64 = 250.;
    const MIN_DISTANCE: f64 = 50.;

    pub fn new() -> Self {
        Self {
            long_range_radar: LongRangeSearchAndTrack {
                target: None,
                target_lost_timeout: 60.,
                last_contact_time: current_time(),
                sweep_width_per_time: 0.01 * TAU,
                sweep_direction: 1.,
            },
            target_received: false,
            closest_dist: f64::MAX,
        }
    }
    pub fn tick(&mut self) {
        if !self.target_received {
            self.receive_target();
            return;
        };
        self.long_range_radar.tick();
        let Some(target) = self.long_range_radar.target() else {
            self.receive_target();
            return;
        };

        let tti = Self::time_to_impact(target);
        let impact_pos = target.position_in(tti);
        draw_diamond(impact_pos, 50., 0xFFFF_FFFF);
        debug!("Boom in {:.2}s", tti);

        let towards_target = target.position() - position();

        if fuel() > 25. {
            let acc_power = fuel() / tti.abs();
            if (0. ..=10.).contains(&tti) || fuel() <= 800. {
                let towards_impact = impact_pos - position();
                let sideways_dir = towards_impact.normalize().rotate(0.25 * TAU);
                let sideways_vel = velocity().dot(sideways_dir);
                let sideways_factor = 200. * towards_impact.length().recip();
                let accel =
                    towards_impact.normalize() - sideways_dir * sideways_vel * sideways_factor;
                turn(10. * angle_diff(heading(), accel.angle()));
                accelerate(accel.normalize() * 0.8 * acc_power);
            } else if fuel() > 800. {
                turn(20. * angle_diff(heading(), towards_target.angle()));
                accelerate(towards_target.normalize() * max_forward_acceleration());
            }
        } else {
            accelerate(vec2(0., 0.));
        }

        self.trigger();
    }
    fn trigger(&mut self) {
        let Some(distance) = self
            .long_range_radar
            .target()
            .map(|t| t.position().distance(position()))
        else {
            return;
        };
        if distance < Self::MIN_DISTANCE
            || self.closest_dist < Self::FUSE_DISTANCE && distance > self.closest_dist
        {
            explode();
        }
        if self.closest_dist > distance {
            self.closest_dist = distance;
        }
    }
    fn receive_target(&mut self) {
        if let Some([px, py, vx, vy]) = receive() {
            let target = TargetInfo {
                pos: vec2(px, py),
                vel: vec2(vx, vy),
                acc: vec2(0., 0.),
                t: current_time(),
            };
            self.long_range_radar.target = Some(target);
            self.long_range_radar.last_contact_time = current_time();
            self.target_received = true;
        }
    }
    fn time_to_impact(target: &TargetInfo) -> f64 {
        let towards_target = target.position() - position();
        let towards_target_dir = towards_target.normalize();
        let closing_speed =
            velocity().dot(towards_target_dir) + target.vel.dot(-towards_target_dir);

        let distance = towards_target.length();
        let mut time = distance / closing_speed;
        let acc_power = fuel() / time.abs();
        let closing_acc =
            0.5 * max_forward_acceleration().min(acc_power) + target.acc.dot(-towards_target_dir);

        for _ in 0..10 {
            time = distance / (closing_speed + closing_acc * time * 0.5);
        }
        time
    }
}

pub struct Fighter {
    long_range_radar: LongRangeSearchAndTrack,
    aim: AimPid,
    missile_fire_tick: u32,
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

impl Fighter {
    pub fn new() -> Fighter {
        Fighter {
            long_range_radar: LongRangeSearchAndTrack {
                target: None,
                target_lost_timeout: 2.5,
                last_contact_time: current_time(),
                sweep_width_per_time: 0.01 * TAU,
                sweep_direction: 1.,
            },
            aim: AimPid::new(100., 0., -110_000.),
            missile_fire_tick: u32::MAX,
        }
    }

    pub fn tick(&mut self) {
        self.long_range_radar.tick();
        let target = self.long_range_radar.target().copied().unwrap_or_default();

        if reload_ticks(1) == 0 {
            fire(1);
            self.missile_fire_tick = current_tick();
        }
        if current_tick() - self.missile_fire_tick < 5 {
            let pos = target.position();
            let vel = target.vel;
            send([pos.x, pos.y, vel.x, vel.y]);
        }

        //let bullet_impact_time = projectile_impact_time2(&self.target, BULLET_SPEED);
        //let aim_pos = rel_vel_target_pos(&self.target, bullet_impact_time);
        let turn_torque = self
            .aim
            .aim_torque((target.position() - position()).angle() + rand(-0.05, 0.05));
        torque(turn_torque);
        //if angle_diff(heading(), (aim_pos - position()).angle()).abs() < 0.01 * TAU {
        fire(0);
        //}

        kite(&target, 40000., 1000.);

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
    let mut side_acc = target_dir.rotate(0.25 * TAU) * (desired_side_speed - side_speed);
    if (current_tick() / 1000) % 2 == 1 {
        side_acc *= -1.;
    }

    let towards_center_acc = if position().length() < 19_000.
        && (position().length() < 15_000. || velocity().dot(position()) < 0.)
    {
        vec2(0., 0.)
    } else {
        debug!("avoid death by cowardice");
        -position() * 0.25
    };
    let acc = (towards_acc + side_acc + towards_center_acc) * max_forward_acceleration();
    accelerate(acc);
}

struct LongRangeSearchAndTrack {
    target: Option<TargetInfo>,
    target_lost_timeout: f64,
    last_contact_time: f64,
    sweep_width_per_time: f64,
    sweep_direction: f64,
}

impl LongRangeSearchAndTrack {
    const SEARCH_FOV: f64 = 0.0035 * TAU;
    const TRACK_FOV: f64 = 0.0001 * TAU;

    fn target(&self) -> Option<&TargetInfo> {
        self.target.as_ref()
    }
    fn search(&mut self) {
        if let Some(contact) = scan() {
            self.target
                .get_or_insert_with(TargetInfo::default)
                .integrate_data(contact.position, contact.velocity, 1.);
            self.last_contact_time = current_time();
            set_radar_width(Self::TRACK_FOV);
            set_radar_heading(dir(position(), contact.position).angle());
            return;
        }
        set_radar_width(Self::SEARCH_FOV);
        set_radar_heading(radar_heading() + radar_width());
    }
    fn tick(&mut self) {
        self.target.map(|t| t.debug(0xFFFF_FFFF));
        if self.target.is_none() || delta_time(self.last_contact_time) > self.target_lost_timeout {
            self.search();
            return;
        }
        let target = self.target.as_mut().unwrap();

        if let Some(contact) = scan() {
            if contact.class != Class::Missile {
                debug!("snr {}\nrssi {}", contact.snr, contact.rssi);
                target.integrate_data(
                    contact.position,
                    contact.velocity,
                    (contact.snr - 10.) / 20. + 5. * delta_time(self.last_contact_time),
                );
                self.last_contact_time = current_time();
            }
        }

        let sweep_width = delta_time(self.last_contact_time) * self.sweep_width_per_time;
        let target_heading = dir(position(), target.position()).angle();

        if sweep_width < Self::TRACK_FOV {
            set_radar_heading(target_heading);
        } else if angle_diff(radar_heading(), target_heading).abs() > sweep_width {
            set_radar_heading(target_heading - sweep_width);
        } else {
            set_radar_heading(radar_heading() + radar_width());
        }
        set_radar_width(Self::TRACK_FOV);
    }
}

struct RadarSearch {
    targets: Vec<TargetInfo>,
    target_fuse_dist: f64,
    target_timeout: f64,
}
impl RadarSearch {
    const FOV: f64 = 0.025 * TAU;
    const HEADING_STEP: f64 = 0.25 * TAU + 0.5 * Self::FOV;

    fn search(&mut self) {
        self.targets.iter().for_each(|t| t.debug(0xFFA0_A0A0));
        set_radar_width(Self::FOV);
        set_radar_heading(radar_heading() + Self::HEADING_STEP);
        set_radar_min_distance(0.);
        set_radar_max_distance(f64::MAX);

        if let Some(contact) = scan() {
            if contact.class != Class::Missile {
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

        set_radar_max_distance(distance + 100. + err);
        set_radar_min_distance(distance - 100. - err);
        set_radar_width(0.5 * TAU * (50. + err) / distance);

        let contact = scan();
        if contact.is_some() {
            self.last_contact_time = current_time();
        }
        debug!("{}", err);
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
    fn debug(&self, color: u32) {
        draw_diamond(self.position(), 100., color);
        draw_triangle(self.pos, 100., color);
        draw_line(self.pos, self.pos + self.vel, color);
        draw_text!(self.pos, color, "{}", delta_time(self.t));
    }

    fn integrate_data(&mut self, position: Vec2, velocity: Vec2, importance: f64) {
        let importance = importance.clamp(0.001, 1.);
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
        p_term + d_term
    }
}
