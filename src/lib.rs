use oort_api::prelude::{maths_rs::prelude::Base, *};
use std::ops::RangeInclusive;

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

pub struct Missile {}
impl Default for Missile {
    fn default() -> Self {
        Self::new()
    }
}

impl Missile {
    pub fn new() -> Self {
        Self {}
    }
    pub fn tick(&mut self) {}
}

pub struct Fighter {
    radar: UnifiedRadar,
    aimbot: AimBot,
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

impl Fighter {
    pub fn new() -> Fighter {
        Fighter {
            radar: UnifiedRadar::new(1., 0.0..=40_000., 0., TAU, TAU * TICK_LENGTH, None),
            aimbot: AimBot::new(200., -100_000.),
        }
    }

    pub fn tick(&mut self) {
        draw_line(
            position(),
            position() + vec2(100_000., 0.).rotate(heading()),
            0xFFFF_FFFF,
        );
        self.radar.tick();
        for c in self.radar.contacts.clone() {
            c.debug();
        }
        let fighters = self.radar.fighters().copied().collect::<Vec<_>>();
        let missiles = self.radar.missiles().copied().collect::<Vec<_>>();
        if fighters.is_empty() {
            self.radar.scan_distance_range = 0.0..=f64::MAX;
            self.radar.scan_direction = 0.;
            self.radar.scan_angle_max = TAU;
        }
        for f in fighters.iter().take(1) {
            self.radar.start_tracking(f.id);
            self.radar.scan_distance_range = 0.0..=f.distance() - 100.;
            self.radar.scan_direction = f.direction().angle();
            self.radar.scan_angle_max = 0.125 * TAU;
            //let lead_pos = f.position_in(f.projectile_impact_time(BULLET_SPEED));
            //draw_diamond(lead_pos, 50., 0xFFA0_A0FF);
            //let aim_torque = self.aimbot.aim_torque((lead_pos - position()).angle());
            //torque(aim_torque);
            //fire(0);
        }
        if let Some((missile, impact_time)) = missiles
            .iter()
            .filter_map(|m| {
                let intercept_time = m.closest_intercept_time();
                let intersept_pos = m.position_in(intercept_time);
                (intersept_pos.distance(position_in(intercept_time)) < 200.)
                    .then_some((m, intercept_time))
            })
            .filter(|(_, t)| t.is_sign_positive())
            .min_by_key(|(_, t)| (t * 1000.) as i64)
        {
            debug!("{}, d: {}", missile.id, impact_time);
            let projectile_time = missile.projectile_impact_time(BULLET_SPEED);
            let lead_pos = missile.position_in(projectile_time) - velocity() * projectile_time;
            draw_diamond(lead_pos, 50., 0xFFA0_A0FF);
            let aim_torque =
                self.aimbot.aim_torque((lead_pos - position()).angle()) + rand(-25., 25.);
            torque(aim_torque);
            if angle_diff(heading(), (lead_pos - position()).angle()).abs() < 0.005 * TAU {
                fire(0);
            }
        } else {
            let aim_torque = self.aimbot.aim_torque(
                self.radar
                    .fighters()
                    .next()
                    .map(|f| f.heading())
                    .unwrap_or(heading()),
            );
            torque(aim_torque);
        }

        for m in missiles {
            if m.distance() < 15000. {
                self.radar.start_tracking(m.id);
            }
        }
        accelerate(-position() * 0.001);
    }
}

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

#[derive(Debug, Clone, Copy)]
struct RadarContact {
    id: u32,
    class: Class,
    time: f64,
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
}

impl RadarContact {
    fn debug(&self) {
        draw_square(self.position, 100., class_color(self.class));
        draw_diamond(self.position(), 100., class_color(self.class));
        draw_line(
            self.position(),
            self.position() + self.velocity,
            class_color(self.class),
        );
        draw_text!(
            self.position(),
            class_color(self.class),
            "{} t: {:.2}",
            self.id,
            elapsed(self.time)
        );
    }

    fn closest_intercept_time(&self) -> f64 {
        let dir = self.direction();
        let closing_speed = velocity().dot(dir) + self.velocity.dot(-dir);

        let time = self.distance() / closing_speed;
        let mut closest_dist = self.position_in(time).distance(position_in(time));
        let mut closest_time = time;

        for i in -10..=10 {
            let t = time + time * i as f64 / 20.;
            let dist = self.position_in(time).distance(position_in(time));
            if dist < closest_dist {
                closest_time = t;
                closest_dist = dist;
            }
        }

        closest_time
    }
    fn projectile_impact_time(&self, projectile_speed: f64) -> f64 {
        let dir = self.direction();
        let time =
            self.distance() / (projectile_speed + velocity().dot(dir) + self.velocity.dot(-dir));

        let mut closest_dist = (self.position_in(time) - velocity() * time).distance(
            position()
                + ((self.position_in(time) - velocity() * time) - position()).normalize()
                    * projectile_speed,
        );
        let mut closest_time = time;

        for i in -10..=10 {
            let t = time + time * i as f64 / 20.;
            let dist = (self.position_in(t) - velocity() * t).distance(
                position()
                    + ((self.position_in(t) - velocity() * t) - position()).normalize()
                        * projectile_speed,
            );
            if dist < closest_dist {
                closest_time = t;
                closest_dist = dist;
            }
        }
        closest_time
    }

    fn position(&self) -> Vec2 {
        self.position_in(0.)
    }
    fn position_in(&self, time: f64) -> Vec2 {
        let elapsed = elapsed(self.time) + time;
        self.position + self.velocity * elapsed + self.acceleration * elapsed.powi(2) * 0.5
    }

    fn distance(&self) -> f64 {
        self.position().distance(position())
    }

    fn direction(&self) -> Vec2 {
        (self.position() - position()).normalize()
    }
    fn heading(&self) -> f64 {
        self.direction().angle()
    }

    fn integrate(&mut self, contact: &ScanResult) {
        self.acceleration = (contact.velocity - self.velocity) * elapsed(self.time);
        self.velocity = contact.velocity;
        self.position = contact.position;
        self.time = current_time();
        self.class = contact.class;
    }
}

fn position_in(time: f64) -> Vec2 {
    position() + velocity() * time
}

type ScanFilter = Box<dyn FnMut(&ScanResult) -> bool>;

struct UnifiedRadar {
    next_id: u32,
    scan_heading: f64,
    scan_distance_range: RangeInclusive<f64>,
    scan_direction: f64,
    scan_angle_max: f64,
    scan_fov: f64,
    contacts: Vec<RadarContact>,
    contact_timeout: f64,
    filter: Option<ScanFilter>,
    tracking: Vec<u32>,
}

impl UnifiedRadar {
    const TWS_TRACK_FOV: f64 = 0.001 * TAU;
    const TWS_TRACK_DIST_RANGE: f64 = 100.;
    const TWS_TRACK_LOST_ZOOM_FACTOR: f64 = 100.;
    const CONTACT_FUSE_DIST_PER_KM: f64 = 200.;

    fn new(
        contact_timeout: f64,
        scan_distance_range: RangeInclusive<f64>,
        scan_direction: f64,
        scan_angle_max: f64,
        scan_fov: f64,
        filter: Option<ScanFilter>,
    ) -> Self {
        Self {
            next_id: 0,
            scan_heading: 0.,
            scan_distance_range,
            scan_direction,
            scan_angle_max,
            scan_fov,
            contacts: Vec::new(),
            contact_timeout,
            filter,
            tracking: Vec::new(),
        }
    }

    fn evict_contacts(&mut self) {
        self.contacts
            .retain(|c| elapsed(c.time) <= self.contact_timeout);
        self.tracking
            .retain(|id| self.contacts.iter().any(|c| c.id == *id));
    }

    fn tick(&mut self) {
        debug!("{:?}", self.tracking);
        match scan() {
            Some(contact) if self.filter.as_mut().map_or(true, |f| f(&contact)) => {
                self.integrate_contact(&contact);
            }
            _ => (),
        }

        let select = current_tick() as usize % (self.tracking.len() + 1);
        let track_contact = (select != 0)
            .then(|| {
                self.contacts
                    .iter()
                    .find(|c| c.id == self.tracking[select - 1])
            })
            .flatten();

        if let Some(c) = track_contact {
            let zoom = 1. + elapsed(c.time) * Self::TWS_TRACK_LOST_ZOOM_FACTOR;
            let radial_zoom = 1. + zoom * TAU / (c.distance() * 0.001);
            set_radar_min_distance(c.distance() - Self::TWS_TRACK_DIST_RANGE * zoom);
            set_radar_max_distance(c.distance() + Self::TWS_TRACK_DIST_RANGE * zoom);
            set_radar_width(Self::TWS_TRACK_FOV * radial_zoom);
            set_radar_heading(c.heading());
        } else {
            set_radar_width(self.scan_fov);
            self.scan_heading += self.scan_fov;
            if angle_diff(self.scan_heading, self.scan_direction).abs() > self.scan_angle_max {
                self.scan_heading = self.scan_direction - self.scan_angle_max;
            }
            set_radar_heading(self.scan_heading);
            if let Some(same_heading_track_dist) = self
                .tracking
                .iter()
                .flat_map(|id| self.contacts.iter().find(|c| c.id == *id))
                .filter(|tc| angle_diff(tc.heading(), self.scan_heading).abs() < self.scan_fov)
                .map(|tc| tc.distance())
                .min_by_key(|d| (d * 1000.) as i64)
            {
                set_radar_max_distance(
                    self.scan_distance_range
                        .end()
                        .min(same_heading_track_dist - 100.),
                );
            } else {
                set_radar_max_distance(*self.scan_distance_range.end());
            }
            set_radar_min_distance(*self.scan_distance_range.start());
        }
        self.evict_contacts();
        self.merge_contacts();
    }

    fn integrate_contact(&mut self, contact: &ScanResult) {
        if let Some(matching_contact) = self
            .contacts
            .iter_mut()
            .filter(|c| {
                c.class == contact.class
                    && c.position().distance(contact.position)
                        < c.distance() * 0.001 * Self::CONTACT_FUSE_DIST_PER_KM
            })
            .min_by_key(|c| (c.position().distance(contact.position) * 1000.) as i64)
        {
            matching_contact.integrate(contact);
        } else {
            self.contacts.push(RadarContact {
                id: self.next_id,
                class: contact.class,
                time: current_time(),
                position: contact.position,
                velocity: contact.velocity,
                acceleration: Vec2::zero(),
            });
            self.next_id += 1;
        }
    }

    fn merge_contacts(&mut self) {
        for (a_idx, a) in self.contacts.iter().enumerate() {
            for (b_idx, b) in self.contacts.iter().enumerate() {
                if a_idx == b_idx || a.id == b.id || a.class != b.class {
                    continue;
                }
                if a.position().distance(b.position())
                    < ((a.position() + b.position()) * 0.5).distance(position())
                        * 0.001
                        * Self::CONTACT_FUSE_DIST_PER_KM
                {
                    let a = *a;
                    let merged = RadarContact {
                        id: a.id,
                        class: a.class,
                        time: (a.time + b.time) * 0.5,
                        position: (a.position + b.position) * 0.5,
                        velocity: (a.velocity + b.velocity) * 0.5,
                        acceleration: (a.acceleration + b.acceleration) * 0.5,
                    };
                    let tracking_a = self.tracking.iter().any(|id| *id == a.id);
                    if let Some((idx, _)) = self
                        .tracking
                        .iter_mut()
                        .enumerate()
                        .find(|(_, id)| **id == b.id)
                    {
                        if tracking_a {
                            self.tracking.remove(idx);
                        } else {
                            self.tracking[idx] = a.id;
                        }
                    }
                    self.contacts[a_idx] = merged;
                    self.contacts.swap_remove(b_idx);
                    return;
                }
            }
        }
    }

    fn start_tracking(&mut self, id: u32) {
        if self.contacts.iter().any(|c| c.id == id) {
            self.tracking.push(id);
            self.tracking.sort_unstable();
            self.tracking.dedup();
        }
    }

    fn fighters(&self) -> impl Iterator<Item = &RadarContact> {
        self.contacts.iter().filter(|c| c.class == Class::Fighter)
    }
    fn missiles(&self) -> impl Iterator<Item = &RadarContact> {
        self.contacts.iter().filter(|c| c.class == Class::Missile)
    }
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
