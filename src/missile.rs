use crate::{
    direction_to, distance_to, heading_to, position_in,
    radar::SearchDirection,
    track::{class_radius, Track},
    Aimbot,
};
use oort_api::{
    prelude::{
        self as oa,
        byteorder::{NativeEndian, ReadBytesExt, WriteBytesExt},
        maths_rs::prelude::Base,
        Vec2, Vec2Extras,
    },
    Ability, Class,
};
use std::{f64::consts::TAU, io};

pub const FRAG_LIFE: f64 = 10.0 * oa::TICK_LENGTH;
pub const FRAG_RANGE: f64 = 150.0;
pub const FRAG_SPEED: f64 = FRAG_RANGE / FRAG_LIFE;
pub const FRAG_LETHAL_RANGE: f64 = 50.0;

const MAX_FUEL: f64 = 2000.0;
const TERMINAL_FUEL: f64 = 1000.0;
const PROGRAM_FUEL: f64 = MAX_FUEL - TERMINAL_FUEL;

pub struct MissileRadar {
    search_dir: SearchDirection,
    track: Option<Track>,
}

impl MissileRadar {
    fn tick(&mut self, guessed_heading: Option<f64>, fov: f64, max_angle: f64) {
        if self.track.is_some() {
            self.tick_locked();
            return;
        } else if let Some(heading) = guessed_heading {
            oa::set_radar_max_distance(f64::MAX);
            oa::set_radar_min_distance(0.0);
            oa::set_radar_width(fov);
            self.next_search_heading(heading, fov, max_angle);
        } else {
            oa::set_radar_max_distance(f64::MAX);
            oa::set_radar_min_distance(0.0);
            oa::set_radar_heading(oa::radar_heading() + fov);
            oa::set_radar_width(fov);
        }
        if let Some(contact) = oa::scan() {
            if contact.class != Class::Missile {
                self.track = Some(Track::new(0, &contact));
            }
        }
    }
    fn next_search_heading(&mut self, heading: f64, fov: f64, max_angle: f64) {
        let mut search_heading = oa::radar_heading();
        match self.search_dir {
            SearchDirection::Cw => search_heading -= fov,
            SearchDirection::Ccw => search_heading += fov,
        }

        if (search_heading + 0.5 * fov) >= heading + max_angle {
            search_heading = heading + max_angle - 0.5 * fov;
            self.search_dir = SearchDirection::Cw;
        } else if (search_heading - 0.5 * fov) <= heading - max_angle {
            search_heading = heading - max_angle + 0.5 * fov;
            self.search_dir = SearchDirection::Ccw;
        }
        oa::set_radar_heading(search_heading);
    }
    fn tick_locked(&mut self) {
        let Some(track) = self.track.as_mut() else {
            return;
        };

        if let Some(contact) = oa::scan() {
            if contact.class == track.class() {
                track.scan_update(&contact);
            }
        }

        let p = track.pos_in(oa::TICK_LENGTH) - oa::velocity() * oa::TICK_LENGTH;
        let e = track.error_in(oa::TICK_LENGTH);
        let d = distance_to(p);
        let r = class_radius(track.class());

        oa::set_radar_heading(heading_to(p));
        oa::set_radar_width(2.0 * (e + r) / d);
        oa::set_radar_min_distance(d - e - r);
        oa::set_radar_max_distance(d + e + r);

        if track.error() > 1000.0 {
            self.track = None;
        }
    }
}

pub struct Missile {
    program: Option<MissileProgram>,
    terminal: bool,
    radar: MissileRadar,
    aimbot: Aimbot,
}

impl Missile {
    pub fn new() -> Self {
        Self {
            program: None,
            terminal: false,
            radar: MissileRadar {
                track: None,
                search_dir: SearchDirection::Ccw,
            },
            aimbot: Aimbot::new(),
        }
    }

    fn receive_program(&mut self) {
        self.program =
            oa::receive_bytes().and_then(|buffer| MissileProgram::from_message(buffer).ok());
    }

    pub fn tick(&mut self) {
        if self.program.is_none() {
            self.receive_program();
        }

        let Some(program) = self.program else {
            return;
        };

        match program {
            MissileProgram::Target { pos, vel } => todo!(),
            MissileProgram::TargetViaPoint { point, pos, vel } => todo!(),
            MissileProgram::Point(_) => todo!(),
            MissileProgram::Heading(heading) => self.tick_heading(heading),
        }
    }

    fn tick_heading(&mut self, heading: f64) {
        self.radar.tick(Some(heading), 0.001 * TAU, 0.025 * TAU);
        if let Some(track) = self.radar.track.as_ref() {
            let tti = || {
                track
                    .intercept(0.0, oa::max_forward_acceleration())
                    .unwrap_or(f64::MAX)
            };
            if self.terminal || oa::fuel() / (0.5 * oa::max_forward_acceleration()) > tti() {
                self.terminal = true;
                self.tick_terminal();
            }
        } else if MAX_FUEL - oa::fuel() < PROGRAM_FUEL {
            let heading_dir = Vec2::new(1.0, 0.0).rotate(heading);
            oa::torque(self.aimbot.aim_at_torque(oa::position() + heading_dir, 0.0));
            oa::accelerate(heading_dir * oa::max_forward_acceleration());
        }
    }
    fn tick_terminal(&mut self) {
        let Some(track) = self.radar.track.as_ref() else {
            return;
        };

        if distance_to(track.pos()) < FRAG_LETHAL_RANGE {
            oa::explode();
            return;
        }

        oa::activate_ability(Ability::Boost);
        let max_acc = oa::max_forward_acceleration();

        let Some(tti_top_fuel) = track.intercept(0.0, max_acc) else {
            oa::torque(self.aimbot.aim_at_torque(track.pos(), 0.0));
            oa::accelerate(max_acc * direction_to(track.pos()));
            return;
        };

        let Some(tti_on_fuel) = track.intercept(0.0, max_acc.min(oa::fuel() / tti_top_fuel)) else {
            oa::torque(self.aimbot.aim_at_torque(track.pos(), 0.0));
            oa::accelerate(max_acc * direction_to(track.pos()));
            return;
        };
        let Some(tti) = track.intercept(0.0, max_acc.min(oa::fuel() / tti_on_fuel)) else {
            oa::torque(self.aimbot.aim_at_torque(track.pos(), 0.0));
            oa::accelerate(max_acc * direction_to(track.pos()));
            return;
        };

        let impact_pos = track.pos_in(tti);

        let acc = 2.0 * (impact_pos - position_in(tti)) / tti.powi(2);
        oa::accelerate(acc);
        oa::torque(self.aimbot.aim_at_torque(oa::position() + acc, 0.0));

        if acc.length() > max_acc {
            oa::debug!("{:.1} m/s^2 over capability", acc.length() - max_acc);
        }

        if acc.length() * tti > oa::fuel() {
            oa::debug!("{:.1} m/s over budget", acc.length() * tti - oa::fuel());
        } else {
            oa::debug!("{:.1} m/s under budget", oa::fuel() - acc.length() * tti);
        }
        oa::debug!("Boom in {tti:.2} seconds");
        oa::draw_diamond(impact_pos, 25.0, 0xFF0000);

        //let correction = impact_pos
        //    - oa::position()
        //    - oa::velocity() * tti
        //    - 0.5 * oa::max_forward_acceleration() * direction_to(impact_pos) * tti.powi(2);
        //oa::draw_square(impact_pos + correction, 25.0, 0x00A0FF);
        //oa::draw_square(impact_pos + 20.0 * (correction / tti), 25.0, 0x00A0A0);
        //let p = 20.0 * (correction / tti) + impact_pos;
        //if distance_to(track.pos()) < 2.0 * FRAG_RANGE {
        //    if let Some((_, p)) = track.intercept_aim_pos(FRAG_SPEED, 0.0) {
        //        oa::draw_triangle(p, 10.0, 0xffffff);
        //        oa::torque(self.aimbot.aim_at_torque(p, 0.0));
        //    } else {
        //        oa::turn(10.0 * oa::angle_diff(oa::heading(), heading_to(p)));
        //    }
        //} else {
        //    oa::turn(10.0 * oa::angle_diff(oa::heading(), heading_to(p)));
        //}

        let dt = tti / 15.0;
        let mut x = oa::position();
        for i in 1..=15 {
            let t = dt * i as f64;
            let xnext = oa::position() + t * oa::velocity() + t.powi(2) * 0.5 * acc;
            oa::draw_line(x, xnext, 0xA0A0A0);
            x = xnext;
        }
    }
}

impl Default for Missile {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MissileProgram {
    Target { pos: Vec2, vel: Vec2 },
    TargetViaPoint { point: Vec2, pos: Vec2, vel: Vec2 },
    Point(Vec2),
    Heading(f64),
}

impl MissileProgram {
    fn variant(&self) -> u8 {
        match self {
            Self::Target { .. } => 0,
            Self::TargetViaPoint { .. } => 1,
            Self::Point(..) => 2,
            Self::Heading(..) => 3,
        }
    }
    pub fn to_message(self) -> [u8; 32] {
        let mut buffer = [0; 32];
        let mut b = buffer.as_mut_slice();

        b.write_u8(self.variant()).unwrap();

        match self {
            Self::Target { pos, vel } => Self::write_target(b, pos, vel).unwrap(),
            Self::TargetViaPoint { point, pos, vel } => {
                Self::write_target_via_point(b, point, pos, vel).unwrap()
            }
            Self::Point(point) => Self::write_point(b, point).unwrap(),
            Self::Heading(heading) => Self::write_heading(b, heading).unwrap(),
        }

        buffer
    }
    pub fn from_message(buffer: [u8; 32]) -> io::Result<Self> {
        let mut b = buffer.as_slice();
        Ok(match b.read_u8()? {
            0 => {
                let (pos, vel) = Self::read_target(b)?;
                Self::Target { pos, vel }
            }
            1 => {
                let (point, pos, vel) = Self::read_target_via_point(b)?;
                Self::TargetViaPoint { point, pos, vel }
            }
            2 => {
                let point = Self::read_point(b)?;
                Self::Point(point)
            }
            3 => {
                let heading = Self::read_heading(b)?;
                Self::Heading(heading)
            }
            _ => unimplemented!(),
        })
    }

    fn read_target(mut b: &[u8]) -> io::Result<(Vec2, Vec2)> {
        let mut pos = Vec2::zero();
        let mut vel = Vec2::zero();
        pos.x = b.read_f32::<NativeEndian>()? as f64;
        pos.y = b.read_f32::<NativeEndian>()? as f64;
        vel.x = b.read_f32::<NativeEndian>()? as f64;
        vel.y = b.read_f32::<NativeEndian>()? as f64;
        Ok((pos, vel))
    }
    fn read_target_via_point(mut b: &[u8]) -> io::Result<(Vec2, Vec2, Vec2)> {
        let mut point = Vec2::zero();
        let mut pos = Vec2::zero();
        let mut vel = Vec2::zero();
        point.x = b.read_f32::<NativeEndian>()? as f64;
        point.y = b.read_f32::<NativeEndian>()? as f64;
        pos.x = b.read_f32::<NativeEndian>()? as f64;
        pos.y = b.read_f32::<NativeEndian>()? as f64;
        vel.x = b.read_f32::<NativeEndian>()? as f64;
        vel.y = b.read_f32::<NativeEndian>()? as f64;
        Ok((point, pos, vel))
    }
    fn read_point(mut b: &[u8]) -> io::Result<Vec2> {
        let mut point = Vec2::zero();
        point.x = b.read_f32::<NativeEndian>()? as f64;
        point.y = b.read_f32::<NativeEndian>()? as f64;
        Ok(point)
    }
    fn read_heading(mut b: &[u8]) -> io::Result<f64> {
        let heading = b.read_f32::<NativeEndian>()? as f64;
        Ok(heading)
    }

    fn write_target(mut b: &mut [u8], pos: Vec2, vel: Vec2) -> io::Result<()> {
        b.write_f32::<NativeEndian>(pos.x as f32)?;
        b.write_f32::<NativeEndian>(pos.y as f32)?;
        b.write_f32::<NativeEndian>(vel.x as f32)?;
        b.write_f32::<NativeEndian>(vel.y as f32)
    }
    fn write_target_via_point(
        mut b: &mut [u8],
        point: Vec2,
        pos: Vec2,
        vel: Vec2,
    ) -> io::Result<()> {
        b.write_f32::<NativeEndian>(point.x as f32)?;
        b.write_f32::<NativeEndian>(point.y as f32)?;
        b.write_f32::<NativeEndian>(pos.x as f32)?;
        b.write_f32::<NativeEndian>(pos.y as f32)?;
        b.write_f32::<NativeEndian>(vel.x as f32)?;
        b.write_f32::<NativeEndian>(vel.y as f32)
    }
    fn write_point(mut b: &mut [u8], point: Vec2) -> io::Result<()> {
        b.write_f32::<NativeEndian>(point.x as f32)?;
        b.write_f32::<NativeEndian>(point.y as f32)
    }
    fn write_heading(mut b: &mut [u8], heading: f64) -> io::Result<()> {
        b.write_f32::<NativeEndian>(heading as f32)
    }
}
