use crate::{
    distance_to, heading_to,
    track::{class_radius, Track},
    F64Ord,
};
use oort_api::prelude::{self as oa, Vec2Extras};

pub struct Radar {
    next_track_id: u64,
    tracks: Vec<Track>,
    track_idx: usize,
    tracked_id: Option<u64>,
    search_heading: f64,
    search_fov: f64,
    next_search_behind: Option<f64>,
    search_limit: Option<(f64, f64)>,
    search_dir: SearchDirection,
}

pub enum SearchDirection {
    Cw,
    Ccw,
}

impl Radar {
    pub fn new(search_fov: f64) -> Self {
        Self {
            next_track_id: 0,
            tracks: Vec::new(),
            track_idx: 0,
            tracked_id: None,
            search_heading: 0.0,
            search_fov,
            next_search_behind: None,
            search_limit: None,
            search_dir: SearchDirection::Ccw,
        }
    }
    pub fn set_limits(&mut self, search_limit: Option<(f64, f64)>) {
        self.search_limit = search_limit;
    }

    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    pub fn get_target(&self, mut prio: impl FnMut(&Track) -> f64) -> Option<&Track> {
        self.tracks.iter().max_by_key(|t| F64Ord(prio(t)))
    }

    pub fn tick(&mut self) {
        self.evict_tracks();
        self.fuse_tracks();
        self.debug_tracks();
        self.scan();

        if oa::current_tick() % 2 == 0 || self.tracks.is_empty() {
            let looking_at_known_track = || {
                self.tracks.iter().find(|t| {
                    oa::angle_diff(heading_to(t.pos_in(oa::TICK_LENGTH)), self.search_heading).abs()
                        < 0.5 * self.search_fov
                })
            };

            oa::set_radar_heading(self.search_heading);
            oa::set_radar_width(self.search_fov);
            if let Some(min_dist) = self.next_search_behind.take() {
                oa::set_radar_min_distance(min_dist);
                oa::set_radar_max_distance(f64::MAX);
                self.next_search_heading();
            } else if let Some(t) = looking_at_known_track() {
                oa::set_radar_min_distance(0.0);
                oa::set_radar_max_distance(
                    t.pos_in(oa::TICK_LENGTH).distance(oa::position())
                        - t.error_in(oa::TICK_LENGTH)
                        - class_radius(t.class()),
                );
                self.next_search_behind = Some(
                    t.pos_in(3.0 * oa::TICK_LENGTH).distance(oa::position())
                        + t.error_in(3.0 * oa::TICK_LENGTH)
                        + class_radius(t.class()),
                );
            } else {
                oa::set_radar_min_distance(0.0);
                oa::set_radar_max_distance(f64::MAX);
                self.next_search_heading();
            }
        } else {
            let t = &self.tracks[self.track_idx % self.tracks.len()];
            self.tracked_id = Some(t.id());
            self.track_idx += 1;

            let p = t.pos_in(oa::TICK_LENGTH);
            let e = t.error_in(oa::TICK_LENGTH);
            let d = distance_to(p);
            let r = class_radius(t.class());

            oa::set_radar_heading(heading_to(p));
            oa::set_radar_width(2.0 * (e + r) / d);
            oa::set_radar_min_distance(d - e - r);
            oa::set_radar_max_distance(d + e + r);
        }
    }

    fn next_search_heading(&mut self) {
        match self.search_dir {
            SearchDirection::Cw => self.search_heading -= self.search_fov,
            SearchDirection::Ccw => self.search_heading += self.search_fov,
        }
        let Some((ccw_lim, cw_lim)) = self.search_limit else {
            return;
        };
        if (self.search_heading + 0.5 * self.search_fov) >= ccw_lim {
            self.search_heading = ccw_lim - 0.5 * self.search_fov;
            self.search_dir = SearchDirection::Cw;
        } else if (self.search_heading - 0.5 * self.search_fov) <= cw_lim {
            self.search_heading = cw_lim + 0.5 * self.search_fov;
            self.search_dir = SearchDirection::Ccw;
        }
    }
    fn evict_tracks(&mut self) {
        self.tracks
            .retain(|t| !t.timed_out() || self.tracked_id.map_or(false, |id| t.id() == id));
    }
    fn fuse_tracks(&mut self) {
        for (i, t) in self.tracks.iter().enumerate() {
            if let Some((ot_i, _)) = self
                .tracks
                .iter()
                .enumerate()
                .skip(i + 1)
                .find(|(_, ot)| t.same_track(ot))
            {
                let ot = self.tracks.swap_remove(ot_i);
                self.tracks[i].track_update(&ot);
                return;
            }
        }
    }

    fn get_mut_by_id(&mut self, id: u64) -> Option<&mut Track> {
        self.tracks.iter_mut().find(|t| t.id() == id)
    }

    fn scan(&mut self) {
        if let Some(id) = self.tracked_id.take() {
            self.track_scan(id);
        } else {
            self.search_scan();
        }
    }

    fn track_scan(&mut self, id: u64) {
        let Some(track) = self.get_mut_by_id(id) else {
            return;
        };
        if let Some(contact) = oa::scan() {
            track.scan_update(&contact);
            //} else if elapsed(track.last_seen()) > 0.1 * class_timeout(track.class())
            //    && track.error() < 100.0
            //{
            //    let id = track.id();
            //    if let Some(idx) = self.tracks.iter().position(|t| t.id() == id) {
            //        self.tracks.swap_remove(idx);
            //    }
        }
    }

    fn search_scan(&mut self) {
        let Some(contact) = oa::scan() else {
            return;
        };

        let error = scan_error_heuristic(contact.rssi, contact.snr);

        let same_track = |track: &Track| {
            if track.class() != contact.class {
                return false;
            }
            let (tp, tv, _) = track.pva();
            let effective_dist = (tp.distance(contact.position) + tv.distance(contact.velocity))
                - (track.error() + error);
            effective_dist < class_radius(track.class())
        };

        if !self.tracks.iter().any(same_track) {
            self.tracks.push(Track::new(self.next_track_id, &contact));
            self.next_track_id += 1;
        }
    }

    fn debug_tracks(&self) {
        self.tracks.iter().for_each(Track::debug);
    }
}

pub fn scan_error_heuristic(rssi: f64, snr: f64) -> f64 {
    (rssi / snr).powi(2)
}
