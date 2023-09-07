use std::ops::RangeInclusive;

use oort_api::prelude::{maths_rs::prelude::Base, *};

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
    x: bool,
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

impl Fighter {
    pub fn new() -> Fighter {
        let mut radar = UnifiedRadar::new(5.);
        radar.mode_scan(0.0..=f64::MAX, TAU * TICK_LENGTH / 1., None);
        Fighter { radar, x: false }
    }

    pub fn tick(&mut self) {
        self.radar.tick();
        let mut y = None;
        for c in self
            .radar
            .scan_contacts
            .iter()
            .chain(&self.radar.tracking_contact)
        {
            c.debug();
            y = Some(c.id);
        }
        if y.is_some() && self.x == false {
            self.radar.mode_tws(
                0.0..=10000.0,
                TAU * TICK_LENGTH * 4.,
                Some(Box::new(|x| x.class == Class::Missile)),
                y.unwrap(),
            );
            self.x = true;
        }
    }
}

fn class_color(class: Class) -> u32 {
    match class {
        Class::Fighter => 0xFFFF_FFFF,
        Class::Frigate => 0xFFFF_FFFF,
        Class::Cruiser => 0xFFFF_FFFF,
        Class::Asteroid => 0xFFFF_FFFF,
        Class::Target => 0xFFFF_FFFF,
        Class::Missile => 0xFFFF_0000,
        Class::Torpedo => 0xFFFF_0000,
        Class::Unknown => 0xFFFF_FFFF,
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

    fn position(&self) -> Vec2 {
        let elapsed = elapsed(self.time);
        self.position + self.velocity * elapsed + self.acceleration * elapsed.powi(2) * 0.5 * 0.25
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

    fn integrate(&mut self, contact: ScanResult) {
        self.acceleration = (contact.velocity - self.velocity) * elapsed(self.time);
        self.velocity = contact.velocity;
        self.position = contact.position;
        self.time = current_time();
        self.class = contact.class;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RadarMode {
    Scan,
    TrackWhileScan,
    Lock,
}

type ScanFilter = Box<dyn FnMut(&ScanResult) -> bool>;

struct UnifiedRadar {
    mode: RadarMode,
    next_id: u32,
    scan_heading: f64,
    scan_range: RangeInclusive<f64>,
    scan_fov: f64,
    scan_contacts: Vec<RadarContact>,
    tracking_contact: Option<RadarContact>,
    contact_timeout: f64,
    scan_filter: Option<ScanFilter>,
}

impl UnifiedRadar {
    const TWS_TRACK_FOV: f64 = 0.001 * TAU;
    const TWS_TRACK_DIST_RANGE: f64 = 100.;
    const CONTACT_FUSE_DIST: f64 = 250.;

    fn new(contact_timeout: f64) -> Self {
        Self {
            mode: RadarMode::Scan,
            next_id: 0,
            scan_heading: 0.,
            scan_range: 0. ..=f64::MAX,
            scan_fov: TAU * TICK_LENGTH,
            scan_contacts: Vec::new(),
            tracking_contact: None,
            contact_timeout,
            scan_filter: None,
        }
    }

    fn evict_contacts(&mut self) {
        self.scan_contacts
            .retain(|c| elapsed(c.time) <= self.contact_timeout);
    }

    fn mode_tws(
        &mut self,
        scan_range: RangeInclusive<f64>,
        scan_fov: f64,
        scan_filter: Option<ScanFilter>,
        track_id: u32,
    ) -> bool {
        if let Some((idx, contact)) = self
            .scan_contacts
            .iter()
            .enumerate()
            .find(|(_, c)| c.id == track_id)
        {
            self.tracking_contact = Some(*contact);
            self.scan_contacts.swap_remove(idx);
            self.mode = RadarMode::TrackWhileScan;
        } else if self.tracking_contact.is_none() {
            self.mode = RadarMode::Scan;
        }
        self.scan_range = scan_range;
        self.scan_fov = scan_fov;
        self.scan_filter = scan_filter;

        self.tracking_contact.is_some()
    }

    fn mode_scan(
        &mut self,
        scan_range: RangeInclusive<f64>,
        scan_fov: f64,
        scan_filter: Option<ScanFilter>,
    ) {
        self.mode = RadarMode::Scan;
        set_radar_min_distance(*scan_range.start());
        set_radar_max_distance(*scan_range.end());
        self.scan_range = scan_range;
        self.scan_fov = scan_fov;
        self.scan_filter = scan_filter;
        self.tracking_contact = None;
        set_radar_width(scan_fov);
    }

    fn tick(&mut self) {
        match self.mode {
            RadarMode::Scan => self.tick_scan(),
            RadarMode::TrackWhileScan => self.tick_tws(),
            RadarMode::Lock => self.tick_lock(),
        }
        self.evict_contacts();
    }

    fn tick_lock(&mut self) {
        todo!()
    }

    fn tick_tws(&mut self) {
        let track = current_tick() % 2 == 0;

        match scan() {
            Some(contact) if !track && self.scan_filter.as_mut().map_or(true, |f| f(&contact)) => {
                self.integrate_contact(contact);
            }
            Some(contact) if track => self.tracking_contact.unwrap().integrate(contact),
            _ => (),
        }

        if track {
            set_radar_min_distance(*self.scan_range.start());
            set_radar_max_distance(*self.scan_range.end());
            set_radar_width(self.scan_fov);
            self.scan_heading += self.scan_fov;
            set_radar_heading(self.scan_heading);
        } else {
            set_radar_min_distance(
                self.tracking_contact.unwrap().distance() - Self::TWS_TRACK_DIST_RANGE,
            );
            set_radar_max_distance(
                self.tracking_contact.unwrap().distance() + Self::TWS_TRACK_DIST_RANGE,
            );
            set_radar_width(Self::TWS_TRACK_FOV + elapsed(self.tracking_contact.unwrap().time));
            set_radar_heading(self.tracking_contact.unwrap().heading());
        }
    }

    fn tick_scan(&mut self) {
        match scan() {
            Some(contact) if self.scan_filter.as_mut().map_or(true, |f| f(&contact)) => {
                self.integrate_contact(contact);
            }
            _ => (),
        }
        self.scan_heading += self.scan_fov;
        set_radar_heading(self.scan_heading);
    }

    fn integrate_contact(&mut self, contact: ScanResult) {
        if let Some(matching_contact) = self
            .scan_contacts
            .iter_mut()
            .chain(&mut self.tracking_contact)
            .filter(|c| {
                c.class == contact.class
                    && c.position().distance(contact.position) < Self::CONTACT_FUSE_DIST
            })
            .min_by_key(|c| (c.position().distance(contact.position) * 1000.) as i64)
        {
            matching_contact.integrate(contact);
        } else {
            self.scan_contacts.push(RadarContact {
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
}

fn elapsed(time: f64) -> f64 {
    current_time() - time
}
