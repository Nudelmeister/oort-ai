use super::{elapsed, track::Track};
use oort_api::prelude::*;
use std::ops::RangeInclusive;
fn class_timeout(class: Class) -> f64 {
    match class {
        Class::Frigate => 5.,
        Class::Cruiser => 10.,
        Class::Asteroid => 30.,
        Class::Missile | Class::Torpedo => 0.25,
        Class::Target | Class::Fighter | Class::Unknown => 2.5,
    }
}

pub struct LockingRadar {
    pub track: Option<Track>,
}
impl LockingRadar {
    pub fn new() -> Self {
        Self { track: None }
    }
    pub fn tick(&mut self) {
        if let Some(contact) = scan() {
            match self.track.as_mut() {
                Some(track) if contact.class == track.class => track.update(&contact),
                None => self.track = Some(Track::from_contact(0, &contact)),
                _ => (),
            }
        }
        if self
            .track
            .as_ref()
            .map_or(false, |t| elapsed(t.last_seen) > class_timeout(t.class))
        {
            self.track = None;
        }
        match self.track.as_ref() {
            Some(track) => {
                let (h, a, d) = track.radar_track_look();
                set_radar_heading(h);
                set_radar_width(a / d);
                set_radar_min_distance(d - a);
                set_radar_max_distance(d + a);
            }
            None => {
                set_radar_heading(heading());
                set_radar_width(0.125 * TAU);
                set_radar_min_distance(0.);
                set_radar_max_distance(f64::MAX);
            }
        }
    }
}

pub struct UnifiedRadar {
    next_id: u32,
    scan_heading: f64,
    scan_distance_range: RangeInclusive<f64>,
    scan_direction: f64,
    scan_angle_max: f64,
    scan_fov: f64,
    contacts: Vec<Track>,
    tracking: Vec<u32>,
    check_behind: bool,
    track_all: bool,
}

impl UnifiedRadar {
    const CONTACT_FUSE_PER_DIST: f64 = 0.1;
    const CONTACT_FUSE_MIN: f64 = 50.;

    pub fn new(
        scan_distance_range: RangeInclusive<f64>,
        scan_direction: f64,
        scan_angle_max: f64,
        scan_fov: f64,
        track_all: bool,
    ) -> Self {
        Self {
            next_id: 0,
            scan_heading: 0.,
            scan_distance_range,
            scan_direction,
            scan_angle_max,
            scan_fov,
            contacts: Vec::new(),
            tracking: Vec::new(),
            check_behind: false,
            track_all,
        }
    }

    pub fn contacts(&self) -> &[Track] {
        &self.contacts
    }

    fn evict_contacts(&mut self) {
        self.contacts
            .retain(|c| elapsed(c.last_seen) <= class_timeout(c.class));
        self.tracking
            .retain(|id| self.contacts.iter().any(|c| *id == c.id));
    }

    fn prepare_track_tick(&mut self, idx: usize) {
        let track = self
            .contacts
            .iter()
            .find(|c| c.id == self.tracking[idx])
            .unwrap();

        let (h, a, d) = track.radar_track_look();
        let w = a / d;
        set_radar_width(w);
        set_radar_heading(h);
        set_radar_min_distance(d - 0.5 * a);
        set_radar_max_distance(d + 0.5 * a);
    }
    fn prepare_search_tick(&mut self) {
        set_radar_width(self.scan_fov);
        if self.check_behind {
            self.check_behind = false;
            set_radar_min_distance(
                radar_max_distance() + radar_max_distance() * Self::CONTACT_FUSE_PER_DIST,
            );
            set_radar_max_distance(*self.scan_distance_range.end());
            set_radar_heading(self.scan_heading);
        } else {
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
                set_radar_max_distance(self.scan_distance_range.end().min(
                    same_heading_track_dist - same_heading_track_dist * Self::CONTACT_FUSE_PER_DIST,
                ));
                self.check_behind = true;
            } else {
                set_radar_max_distance(*self.scan_distance_range.end());
            }
            set_radar_min_distance(*self.scan_distance_range.start());
        }
    }

    pub fn tick(&mut self) {
        debug!("{:?}", self.tracking);
        for c in &self.contacts {
            c.debug();
        }
        if let Some(contact) = scan() {
            self.integrate_contact(&contact);
        }

        let select = current_tick() as usize % (self.tracking.len() + 1);
        if select > 0 {
            self.prepare_track_tick(select - 1);
        } else {
            self.prepare_search_tick();
        }

        self.evict_contacts();
    }

    fn integrate_contact(&mut self, contact: &ScanResult) {
        if let Some((existing_track, _)) = self
            .contacts
            .iter_mut()
            .filter(|c| c.class == contact.class)
            .filter_map(|c| {
                let (p, v, _) = c.pva();
                let fuse_dist = (p.distance(position()) * Self::CONTACT_FUSE_PER_DIST)
                    .max(Self::CONTACT_FUSE_MIN);
                let pos_match = p.distance(contact.position) < fuse_dist;
                let vel_match = v.distance(contact.velocity) < fuse_dist;
                (pos_match && vel_match).then_some((
                    c,
                    p.distance(contact.position) + v.distance(contact.velocity),
                ))
            })
            .min_by_key(|(_, d)| (d * 1000.) as i64)
        {
            existing_track.update(contact);
        } else {
            self.contacts.push(Track::new(
                self.next_id,
                contact.class,
                contact.position,
                contact.velocity,
            ));
            if self.track_all {
                self.start_tracking(self.next_id);
            }
            self.next_id += 1;
        }
    }

    pub fn start_tracking(&mut self, id: u32) {
        if self.contacts.iter().any(|c| c.id == id) && !self.tracking.contains(&id) {
            self.tracking.push(id);
        }
    }

    pub fn prio_track(&self) -> Option<&Track> {
        self.contacts
            .iter()
            .max_by_key(|t| (t.priority() * 1000.) as i64)
    }
}
