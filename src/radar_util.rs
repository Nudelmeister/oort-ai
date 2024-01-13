use crate::{
    position_in,
    track::{class_radius, Track},
};
use oort_api::prelude::{
    angle_diff, radar_heading, set_radar_heading, set_radar_max_distance, set_radar_min_distance,
    set_radar_width, ScanResult, Vec2Extras, TICK_LENGTH,
};

pub fn scan_error_heuristic(contact: &ScanResult) -> f64 {
    (contact.rssi / contact.snr).powi(2)
}

pub fn set_to_track(track: &Track, max_width: f64) {
    let tp = track.pos_in(TICK_LENGTH);
    let sp = position_in(TICK_LENGTH);
    let e = track.error_in(TICK_LENGTH);
    let d = tp.distance(sp);
    let r = class_radius(track.class());

    set_radar_heading((tp - sp).angle());
    set_radar_width((2.0 * (e + r) / d).min(max_width));
    set_radar_min_distance(d - e - r);
    set_radar_max_distance(d + e + r);
}

fn next_heading(width: f64, rotation_dir: RotationDir) -> f64 {
    let heading = radar_heading();
    match rotation_dir {
        RotationDir::Ccw => heading + width,
        RotationDir::Cw => heading - width,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum RotationDir {
    #[default]
    Ccw,
    Cw,
}

impl RotationDir {
    pub fn reversed(self) -> Self {
        match self {
            Self::Ccw => Self::Cw,
            Self::Cw => Self::Ccw,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum RadarSearch {
    Circular(CircularSearch),
    Cone(ConeSearch),
}

impl RadarSearch {
    pub fn tick(&mut self, tracks: &[Track]) {
        match self {
            RadarSearch::Circular(p) => p.tick(tracks),
            RadarSearch::Cone(p) => p.tick(tracks),
        }
    }

    pub fn circular_mut(&mut self) -> Option<&mut CircularSearch> {
        match self {
            Self::Circular(inner) => Some(inner),
            _ => None,
        }
    }
    pub fn cone_mut(&mut self) -> Option<&mut ConeSearch> {
        match self {
            Self::Cone(inner) => Some(inner),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CircularSearch {
    width: f64,
}

impl CircularSearch {
    pub fn new(width: f64) -> Self {
        Self { width }
    }

    pub fn set_width(&mut self, width: f64) {
        self.width = width;
    }

    pub fn tick(&mut self, tracks: &[Track]) {
        set_radar_heading(next_heading(self.width, RotationDir::default()));
        set_radar_width(self.width);
        set_radar_min_distance(0.0);
        set_radar_max_distance(f64::MAX);
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ConeSearch {
    width: f64,
    rotation_dir: RotationDir,
    max_angle: f64,
    heading: f64,
}

impl ConeSearch {
    pub fn new(width: f64, max_angle: f64, heading: f64) -> Self {
        Self {
            width,
            rotation_dir: RotationDir::default(),
            max_angle,
            heading,
        }
    }

    pub fn set_width(&mut self, width: f64) {
        self.width = width;
    }
    pub fn set_max_angle(&mut self, max_angle: f64) {
        self.max_angle = max_angle;
    }
    pub fn set_heading(&mut self, heading: f64) {
        self.heading = heading;
    }

    pub fn tick(&mut self, tracks: &[Track]) {
        if angle_diff(radar_heading(), self.heading).abs() > self.max_angle {
            self.rotation_dir = self.rotation_dir.reversed();
        }

        set_radar_heading(next_heading(self.width, self.rotation_dir));
        set_radar_width(self.width);
        set_radar_min_distance(0.0);
        set_radar_max_distance(f64::MAX);
    }
}
