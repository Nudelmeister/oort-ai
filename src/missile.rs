use crate::{
    direction_to, distance_to, elapsed, heading_to, position_in,
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
pub const FRAG_LETHAL_RANGE: f64 = 100.0;

const MAX_FUEL: f64 = 2000.0;
const TERMINAL_FUEL: f64 = 750.0;
const PROGRAM_FUEL: f64 = MAX_FUEL - TERMINAL_FUEL;

pub struct MissileRadar {
    search_dir: SearchDirection,
    track: Option<Track>,
}

impl MissileRadar {
    fn tick(&mut self, guessed_heading: Option<f64>, fov: f64, max_angle: f64) {
        if self.track.is_some() {
            self.tick_locked(fov);
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
            self.track = Some(Track::new(0, &contact));
            //if contact.class != Class::Missile {
            //    self.track = Some(Track::new(0, &contact));
            //}
        }
    }
    fn next_search_heading(&mut self, heading: f64, fov: f64, max_angle: f64) {
        let mut search_heading = oa::radar_heading();
        match self.search_dir {
            SearchDirection::Cw => search_heading -= fov,
            SearchDirection::Ccw => search_heading += fov,
        }

        if oa::angle_diff(search_heading + 0.5 * fov, heading).abs() >= max_angle {
            search_heading = heading + max_angle - 0.5 * fov;
            self.search_dir = SearchDirection::Cw;
        } else if oa::angle_diff(search_heading - 0.5 * fov, heading).abs() >= max_angle {
            search_heading = heading - max_angle + 0.5 * fov;
            self.search_dir = SearchDirection::Ccw;
        }
        oa::set_radar_heading(search_heading);
    }
    fn tick_locked(&mut self, max_fov: f64) {
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
        oa::set_radar_width((2.0 * (e + r) / d).min(max_fov));
        oa::set_radar_min_distance(d - e - r);
        oa::set_radar_max_distance(d + e + r);

        if track.error() > 1000.0 {
            self.track = None;
        }
    }
}

pub struct Missile {
    program: Option<MissileProgram>,
    program_receive_time: f64,
    terminal: bool,
    radar: MissileRadar,
    aimbot: Aimbot,
}

impl Missile {
    pub fn new() -> Self {
        Self {
            program: None,
            program_receive_time: 0.0,
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
            if self.program.is_some() {
                self.program_receive_time = oa::current_time();
            }
        }

        let Some(program) = self.program else {
            return;
        };

        match program {
            MissileProgram::Target { pos, vel } => self.tick_target(pos, vel),
            MissileProgram::TargetViaPoint { point, t, pos, vel } => {
                self.tick_target_via_point(point, t, pos, vel)
            }
            MissileProgram::Point(_) => todo!(),
            MissileProgram::Heading(heading) => self.tick_heading(heading),
        }
    }
    fn tick_target_via_point(&mut self, point: Vec2, t: f64, pos: Vec2, vel: Vec2) {
        let p = pos + vel * elapsed(self.program_receive_time);
        oa::draw_diamond(p, 250.0, 0xa0a0a0);
        self.radar
            .tick(Some(heading_to(p)), 0.001 * TAU, 0.02 * TAU);
        if oa::current_time() > t
            && self
                .radar
                .track
                .as_ref()
                .map_or(false, |t| t.error() < 10.0 * class_radius(t.class()))
        {
            self.tick_terminal();
        } else if MAX_FUEL - oa::fuel() < PROGRAM_FUEL {
            oa::draw_square(point, 250.0, 0xa0a0a0);
            let vel_towards = oa::velocity().dot(direction_to(point));
            let eta = distance_to(point) / vel_towards;
            let correction = oa::velocity().dot(direction_to(point).rotate(0.25 * TAU));
            oa::debug!("{vel_towards:?}");
            oa::debug!("{correction:?}");
            oa::debug!("{eta:?}");
            oa::debug!("{:?}", eta + oa::current_time());
            oa::debug!("{t:?}");
            let acc = (direction_to(point) - direction_to(point).rotate(0.25 * TAU) * correction)
                * oa::max_forward_acceleration();
            oa::debug!("{acc:?}");
            if eta < 0.0 || eta + oa::current_time() > t {
                oa::accelerate(acc);
            }

            oa::torque(self.aimbot.aim_at_torque(oa::position() + acc, 0.0));
        }
    }

    fn tick_target(&mut self, pos: Vec2, vel: Vec2) {
        let p = pos + vel * elapsed(self.program_receive_time);
        let dir = direction_to(p);
        let d = distance_to(p);
        let tti = d / ((vel - oa::velocity()).dot(-dir) + 0.5 * oa::fuel());
        let p_tti = p + vel * tti;
        self.radar
            .tick(Some(heading_to(p)), 0.001 * TAU, 0.02 * TAU);
        if self
            .radar
            .track
            .as_ref()
            .map_or(false, |t| t.error() < 10.0 * class_radius(t.class()))
        {
            self.tick_terminal();
        } else if MAX_FUEL - oa::fuel() < PROGRAM_FUEL {
            oa::draw_square(p_tti, 250.0, 0xa0a0a0);
            oa::torque(self.aimbot.aim_at_torque(p_tti, 0.0));
            oa::accelerate(direction_to(p_tti) * oa::max_forward_acceleration());
        }
    }

    fn tick_heading(&mut self, heading: f64) {
        self.radar.tick(Some(heading), 0.001 * TAU, 0.05 * TAU);
        if self
            .radar
            .track
            .as_ref()
            .map_or(false, |t| t.error() < 10.0 * class_radius(t.class()))
        {
            self.tick_terminal();
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

        if distance_to(track.pos_in(oa::TICK_LENGTH)) < FRAG_LETHAL_RANGE {
            oa::explode();
            return;
        }

        oa::activate_ability(Ability::Boost);

        let closing_speed = (track.vel() - oa::velocity()).dot(-direction_to(track.pos()));
        let approx_tti = distance_to(track.pos()) / (closing_speed + 0.5 * oa::fuel());
        let max_acc = oa::max_forward_acceleration();

        oa::debug!("Closing speed {closing_speed:.2} m/s");
        oa::debug!("Approx Intercept in {approx_tti:.2} seconds");
        oa::draw_diamond(track.pos_in(approx_tti), 50.0, 0xa00000);
        let tti = track
            .intercept(0.0, max_acc.min(oa::fuel() / approx_tti))
            .map_or(approx_tti, |tti| {
                if tti > 2.0 * approx_tti {
                    oa::debug!("Fudging numbers (real tti {tti:.2})");
                    let tti_f = (tti / approx_tti).recip();
                    tti_f * tti + (1.0 - tti_f) * approx_tti
                } else {
                    tti
                }
            });
        oa::debug!("Intercept in {tti:.2} seconds");

        let impact_pos = track.pos_in(tti);

        let acc = 2.0 * (impact_pos - position_in(tti)) / tti.powi(2);
        oa::accelerate(acc);

        if tti < 1.0 {
            let Some((frag_tti, frag_impact)) = track.intercept_aim_pos(FRAG_SPEED, 0.0) else {
                oa::explode();
                return;
            };
            let tte = frag_tti - FRAG_LETHAL_RANGE / FRAG_SPEED;
            oa::debug!("Prepare explode");
            oa::debug!("Boom in {tte:.2}");
            oa::draw_triangle(position_in(tte) + tte.powi(2) * 0.5 * acc, 25.0, 0xffffa0);
            oa::draw_line(
                oa::position(),
                Vec2::new(1000000.0, 0.0).rotate(heading_to(frag_impact)),
                0xa0a0ff,
            );
            oa::draw_line(
                oa::position(),
                oa::position() + Vec2::new(1000000.0, 0.0).rotate(oa::heading()),
                0xFFA0A0,
            );
            oa::torque(self.aimbot.aim_at_torque(frag_impact, 0.0));
            if tte <= 0.0 {
                oa::explode();
                return;
            }
        } else {
            oa::torque(self.aimbot.aim_at_torque(oa::position() + acc, 0.0));
        }

        if acc.length() > max_acc {
            oa::debug!("{:.1} m/s^2 over capability", acc.length() - max_acc);
        }

        if acc.length() * tti > oa::fuel() {
            oa::debug!("{:.1} m/s over budget", acc.length() * tti - oa::fuel());
        } else {
            oa::debug!("{:.1} m/s under budget", oa::fuel() - acc.length() * tti);
        }
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
    Target {
        pos: Vec2,
        vel: Vec2,
    },
    TargetViaPoint {
        point: Vec2,
        t: f64,
        pos: Vec2,
        vel: Vec2,
    },
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
            Self::TargetViaPoint { point, t, pos, vel } => {
                Self::write_target_via_point(b, point, t, pos, vel).unwrap()
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
                let (point, t, pos, vel) = Self::read_target_via_point(b)?;
                Self::TargetViaPoint { point, t, pos, vel }
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
    fn read_target_via_point(mut b: &[u8]) -> io::Result<(Vec2, f64, Vec2, Vec2)> {
        let mut point = Vec2::zero();
        let mut pos = Vec2::zero();
        let mut vel = Vec2::zero();
        point.x = b.read_f32::<NativeEndian>()? as f64;
        point.y = b.read_f32::<NativeEndian>()? as f64;
        let t = b.read_f32::<NativeEndian>()? as f64;
        pos.x = b.read_f32::<NativeEndian>()? as f64;
        pos.y = b.read_f32::<NativeEndian>()? as f64;
        vel.x = b.read_f32::<NativeEndian>()? as f64;
        vel.y = b.read_f32::<NativeEndian>()? as f64;
        Ok((point, t, pos, vel))
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
        t: f64,
        pos: Vec2,
        vel: Vec2,
    ) -> io::Result<()> {
        b.write_f32::<NativeEndian>(point.x as f32)?;
        b.write_f32::<NativeEndian>(point.y as f32)?;
        b.write_f32::<NativeEndian>(t as f32)?;
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
