use oort_api::prelude::{
    byteorder::{NetworkEndian, ReadBytesExt, WriteBytesExt},
    maths_rs::{prelude::Base, Vec2f},
    *,
};
use std::{f64::consts::SQRT_2, io::Cursor, ops::RangeInclusive};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MissileStage {
    Boost,
    Cruise,
    Terminal,
}

pub struct Missile {
    id: Option<u16>,
    radar: LockingRadar,
    paired: Option<u16>,
    paired_tti: Option<f64>,
    explode_dist: f64,
    stage: MissileStage,
}
impl Default for Missile {
    fn default() -> Self {
        Self::new()
    }
}

impl Missile {
    const MAX_TTI: f64 = 5.;
    const BOOST_BUDGET: f64 = 1000.;

    pub fn new() -> Self {
        Self {
            id: None,
            radar: LockingRadar::new(),
            paired: None,
            paired_tti: None,
            explode_dist: 50.,
            stage: MissileStage::Boost,
        }
    }

    pub fn receive(&mut self) {
        if let Some(message) = receive_bytes().and_then(Message::from_bytes) {
            match message {
                Message::MissileInit(msg) => self.receive_missile_init(msg),
                Message::TargetUpdate(msg) => self.receive_target_update(msg),
                Message::PotentialTarget(msg) => self.receive_potential_target(msg),
                Message::Pair(msg) => self.receive_pair(msg),
                Message::PairTti(msg) => self.receive_pair_tti(msg),
            }
        }
    }

    fn receive_pair(&mut self, msg: Pair) {
        let Some(my_id) = self.id else {
            return;
        };
        if self.paired.is_some() {
            return;
        }
        if msg.missile_id_a == my_id {
            self.paired = Some(msg.missile_id_b);
        } else if msg.missile_id_b == my_id {
            self.paired = Some(msg.missile_id_a);
        }
    }
    fn receive_pair_tti(&mut self, msg: PairTti) {
        let Some(paired_id) = self.paired else {
            return;
        };
        if msg.sender_missile_id == paired_id {
            self.paired_tti = Some(msg.time_to_impact.into());
        }
    }

    fn receive_potential_target(&mut self, msg: TargetUpdate) {
        if msg
            .missile_id
            .get()
            .zip(self.id)
            .map_or(true, |(m_id, s_id)| m_id == s_id)
        {
            if self.paired.is_some() {
                return;
            }
            let c = Track::with_uncertanty(
                0,
                msg.target.class,
                msg.target.pos.into(),
                msg.target.vel.into(),
                msg.target.uncertanty as f64,
            );
            let intercept_time = c.closest_intercept_time();
            if intercept_time < 0. {
                return;
            }
            if c.position_in(intercept_time)
                .distance(position_in(intercept_time))
                > 50.
            {
                return;
            }
            self.radar.track = Some(c);
            self.explode_dist = 25.;
            deactivate_ability(Ability::ShapedCharge);
        }
    }

    fn receive_target_update(&mut self, msg: TargetUpdate) {
        if msg
            .missile_id
            .get()
            .zip(self.id)
            .map_or(false, |(m_id, s_id)| m_id != s_id)
        {
            return;
        }

        if let Some(track) = self.radar.track.as_mut() {
            track.update_fixed_noise(
                msg.target.pos.into(),
                msg.target.vel.into(),
                msg.target.uncertanty.into(),
            );
        } else {
            self.radar.track = Some(Track::with_uncertanty(
                0,
                msg.target.class,
                msg.target.pos.into(),
                msg.target.vel.into(),
                msg.target.uncertanty.into(),
            ));
        }
        self.explode_dist = 250.;
        activate_ability(Ability::ShapedCharge);
    }

    fn receive_missile_init(&mut self, msg: MissileInit) {
        if self.id.is_some() {
            return;
        }
        self.id = Some(msg.missile_id);
        self.radar.track = Some(Track::with_uncertanty(
            0,
            msg.target.class,
            msg.target.pos.into(),
            msg.target.vel.into(),
            msg.target.uncertanty.into(),
        ))
    }

    fn send(&mut self) {
        let Some((my_id, _)) = self.id.zip(self.paired) else {
            return;
        };
        let Some(track) = self.radar.track.as_ref() else {
            return;
        };

        if rand(0., 10.) < 1. {
            let message = Message::PairTti(PairTti {
                sender_missile_id: my_id,
                time_to_impact: (track.distance() / track.closing_speed()) as f32,
            });
            send_bytes(&message.to_bytes());
            draw_triangle(position(), 1000., 0xFFA0_A0FF);
        } else if rand(0., 50.) < 1. {
            let message = Message::TargetUpdate(TargetUpdate {
                missile_id: PackedOptionU16::make(None),
                target: TargetMsg {
                    class: track.class,
                    uncertanty: track.uncertanty() as f32,
                    pos: track.position().into(),
                    vel: track.velocity().into(),
                },
            });
            send_bytes(&message.to_bytes());
            draw_triangle(position(), 1000., 0xFF50_50FF);
        }
    }

    fn tick(&mut self) {
        self.receive();
        self.radar.tick();
        self.send();

        if let Some(p_tti) = self.paired_tti.as_mut() {
            *p_tti -= TICK_LENGTH;
        }

        let Some(track) = self.radar.track.clone() else {
            turn(angle_diff(heading(), velocity().angle()));
            return;
        };
        track.debug();

        match self.stage {
            MissileStage::Boost => self.tick_boost(&track),
            MissileStage::Cruise => self.tick_cruise(&track),
            MissileStage::Terminal => self.tick_terminal(&track),
        }
    }

    fn tick_boost(&mut self, contact: &Track) {
        let boost_fuel = fuel() + Self::BOOST_BUDGET - 2000.;

        let tti =
            contact.distance() / (contact.closing_speed() + 0.5 * (fuel() - Self::BOOST_BUDGET));
        if boost_fuel < 0. || (tti.is_sign_positive() && tti < Self::MAX_TTI) {
            deactivate_ability(Ability::Boost);
            accelerate(Vec2::zero());
            self.stage = MissileStage::Cruise;
            return;
        }
        debug!("Boost");
        let lead_time = Self::MAX_TTI;

        let lead_pos = contact.position_in(lead_time);
        draw_line(position(), lead_pos, 0xFF00_FF00);

        let lead_dir = (lead_pos - position()).normalize();
        let side_dir = lead_dir.rotate(0.25 * TAU);
        let side_vel = side_dir * velocity().normalize().dot(side_dir);
        let acc_dir = lead_dir - side_vel;

        accelerate(acc_dir * max_forward_acceleration());
        turn(10. * angle_diff(heading(), acc_dir.angle()));
        activate_ability(Ability::Boost);
    }

    fn tick_cruise(&mut self, contact: &Track) {
        debug!("Cruise");
        let real_tti = contact.distance() / contact.closing_speed();
        let terminal_tti = contact.distance() / (contact.closing_speed() + 0.5 * fuel());
        debug!("tti {:.2} t-tti {:.2}", real_tti, terminal_tti);

        if terminal_tti < Self::MAX_TTI || max_forward_acceleration() * terminal_tti < fuel() {
            self.stage = MissileStage::Terminal;
            return;
        }

        let tti_diff = self.paired_tti.map_or(0., |p_tti| real_tti - p_tti);
        if tti_diff > 0. {
            debug!("Speeding up");
        } else if tti_diff < 0. {
            debug!("Slowing down");
        }
        debug!("tti_diff_metric {}", 100. * tti_diff / real_tti);
        let tti = real_tti.clamp(0., Self::MAX_TTI);

        let intercept_pos = contact.position_in(tti);
        let intercept_my_pos = position_in(tti);
        let intercept_pos_dir = (intercept_pos - position()).normalize();
        let intercept_pos_side_dir = intercept_pos_dir.rotate(0.25 * TAU);
        let miss_dv =
            (intercept_pos - intercept_my_pos).dot(intercept_pos_side_dir) * intercept_pos_side_dir;
        let miss_cont_acc = miss_dv / tti;
        draw_line(position(), intercept_pos, 0xFF00_FF00);

        let tti_adjust_acc = intercept_pos_dir * 100. * (tti_diff / real_tti);

        // We could correct our velocity within terminal time and have enough fuel to do so
        if miss_cont_acc.length() < max_forward_acceleration() * terminal_tti
            && fuel() > miss_dv.length()
        {
            debug!("Chillin");
            turn(10. * angle_diff(heading(), tti_adjust_acc.angle()));
            accelerate(tti_adjust_acc);
        } else {
            turn(10. * angle_diff(heading(), (miss_cont_acc + tti_adjust_acc).angle()));
            accelerate(miss_cont_acc + tti_adjust_acc);
        }

        debug!("Correction dv: {:.2}", miss_dv.length());
        draw_line(position(), intercept_pos, 0xFF00_FF00);
    }
    fn tick_terminal(&mut self, contact: &Track) {
        debug!("Terminal");
        let tti = contact.distance() / (contact.closing_speed() + 0.5 * fuel());
        let max_fuel_used = (tti * max_forward_acceleration()).min(fuel());
        let tti = contact.distance() / (contact.closing_speed() + 0.5 * max_fuel_used);

        let impact_pos = contact.position_in(tti);
        draw_line(position(), impact_pos, 0xFF00_FF00);

        let impact_dir = (impact_pos - position()).normalize();
        let impact_side_dir = impact_dir.rotate(0.25 * TAU);
        let side_vel = velocity().dot(impact_side_dir) * impact_side_dir;
        let side_acc = 10. * -side_vel / tti;

        let acc_length = (fuel() / tti).min(max_forward_acceleration());
        let toward_acc = if acc_length > side_acc.length() {
            impact_dir * (acc_length.powi(2) - side_acc.length().powi(2)).sqrt()
        } else {
            Vec2::zero()
        };

        let acc = side_acc + toward_acc;
        if acc.length() > max_forward_acceleration() {
            activate_ability(Ability::Boost);
        }
        turn(10. * angle_diff(heading(), acc.angle()));
        if fuel() > 10. {
            accelerate(acc);
        } else {
            deactivate_ability(Ability::Boost);
            accelerate(Vec2::zero());
        }
        debug!("tti {:.3}", tti);
        debug!("side_vel {:.3}", side_vel.length());
        debug!("side_acc {:.3}", side_acc.length());
        debug!("to_acc {:.3}", toward_acc.length());
        debug!("acc {:.3}", acc.length());
        debug!("acc l {:.3}", acc_length);

        if health() < 20. || tti <= TICK_LENGTH || contact.distance() < self.explode_dist {
            explode();
        }
    }
}

#[derive(Clone, Copy)]
enum Message {
    MissileInit(MissileInit),
    TargetUpdate(TargetUpdate),
    PotentialTarget(TargetUpdate),
    Pair(Pair),
    PairTti(PairTti),
}
impl Message {
    fn from_bytes(data: [u8; 32]) -> Option<Self> {
        let mut cursor = Cursor::new(data);
        let discriminant = cursor.read_u8().ok()?;
        match discriminant {
            0 => Some(Self::MissileInit(MissileInit::from_bytes(&mut cursor)?)),
            1 => Some(Self::TargetUpdate(TargetUpdate::from_bytes(&mut cursor)?)),
            2 => Some(Self::PotentialTarget(TargetUpdate::from_bytes(
                &mut cursor,
            )?)),
            3 => Some(Self::Pair(Pair::from_bytes(&mut cursor)?)),
            4 => Some(Self::PairTti(PairTti::from_bytes(&mut cursor)?)),
            _ => None,
        }
    }
    fn to_bytes(self) -> [u8; 32] {
        let mut data = [0; 32];
        let mut cursor = Cursor::new(data.as_mut_slice());
        match self {
            Message::MissileInit(msg) => {
                let _ = cursor.write_u8(0);
                msg.to_bytes(&mut cursor);
            }
            Message::TargetUpdate(msg) => {
                let _ = cursor.write_u8(1);
                msg.to_bytes(&mut cursor);
            }
            Message::PotentialTarget(msg) => {
                let _ = cursor.write_u8(2);
                msg.to_bytes(&mut cursor);
            }
            Message::Pair(msg) => {
                let _ = cursor.write_u8(3);
                msg.to_bytes(&mut cursor);
            }
            Message::PairTti(msg) => {
                let _ = cursor.write_u8(4);
                msg.to_bytes(&mut cursor);
            }
        };

        data
    }
}

#[derive(Clone, Copy)]
struct PackedOptionU16(u16);
impl PackedOptionU16 {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        Some(Self(data.read_u16::<NetworkEndian>().ok()?))
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        let _ = data.write_u16::<NetworkEndian>(self.0);
    }
    fn get(self) -> Option<u16> {
        (self.0 != u16::MAX).then_some(self.0)
    }
    fn make(value: Option<u16>) -> Self {
        Self(value.unwrap_or(u16::MAX))
    }
}
#[derive(Clone, Copy)]
struct PairTti {
    sender_missile_id: u16,
    time_to_impact: f32,
}
impl PairTti {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        let sender_missile_id = data.read_u16::<NetworkEndian>().ok()?;
        let time_to_impact = data.read_f32::<NetworkEndian>().ok()?;
        Some(Self {
            sender_missile_id,
            time_to_impact,
        })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        let _ = data.write_u16::<NetworkEndian>(self.sender_missile_id);
        let _ = data.write_f32::<NetworkEndian>(self.time_to_impact);
    }
}

#[derive(Clone, Copy)]
struct Pair {
    missile_id_a: u16,
    missile_id_b: u16,
}
impl Pair {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        let missile_id_a = data.read_u16::<NetworkEndian>().ok()?;
        let missile_id_b = data.read_u16::<NetworkEndian>().ok()?;
        Some(Self {
            missile_id_a,
            missile_id_b,
        })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        let _ = data.write_u16::<NetworkEndian>(self.missile_id_a);
        let _ = data.write_u16::<NetworkEndian>(self.missile_id_b);
    }
}

#[derive(Clone, Copy)]
struct MissileInit {
    missile_id: u16,
    target: TargetMsg,
}
impl MissileInit {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        let missile_id = data.read_u16::<NetworkEndian>().ok()?;
        let target = TargetMsg::from_bytes(data)?;
        Some(Self { missile_id, target })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        let _ = data.write_u16::<NetworkEndian>(self.missile_id);
        self.target.to_bytes(data);
    }
}
#[derive(Clone, Copy)]
struct TargetUpdate {
    missile_id: PackedOptionU16,
    target: TargetMsg,
}
impl TargetUpdate {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        let missile_id = PackedOptionU16::from_bytes(data)?;
        let target = TargetMsg::from_bytes(data)?;
        Some(Self { missile_id, target })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        self.missile_id.to_bytes(data);
        self.target.to_bytes(data);
    }
}

#[derive(Clone, Copy)]
struct TargetMsg {
    class: Class,
    uncertanty: f32,
    pos: Vec2f,
    vel: Vec2f,
}
impl TargetMsg {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        let class = match data.read_u8().ok()? {
            0 => Class::Fighter,
            1 => Class::Frigate,
            2 => Class::Cruiser,
            3 => Class::Asteroid,
            4 => Class::Target,
            5 => Class::Missile,
            6 => Class::Torpedo,
            7 => Class::Unknown,
            _ => return None,
        };

        let uncertanty = data.read_f32::<NetworkEndian>().ok()?;

        let px = data.read_f32::<NetworkEndian>().ok()?;
        let py = data.read_f32::<NetworkEndian>().ok()?;
        let vx = data.read_f32::<NetworkEndian>().ok()?;
        let vy = data.read_f32::<NetworkEndian>().ok()?;

        Some(Self {
            class,
            uncertanty,
            pos: Vec2f::new(px, py),
            vel: Vec2f::new(vx, vy),
        })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        let _ = data.write_u8(match self.class {
            Class::Fighter => 0,
            Class::Frigate => 1,
            Class::Cruiser => 2,
            Class::Asteroid => 3,
            Class::Target => 4,
            Class::Missile => 5,
            Class::Torpedo => 6,
            Class::Unknown => 7,
        });
        let _ = data.write_f32::<NetworkEndian>(self.uncertanty);
        let _ = data.write_f32::<NetworkEndian>(self.pos.x);
        let _ = data.write_f32::<NetworkEndian>(self.pos.y);
        let _ = data.write_f32::<NetworkEndian>(self.vel.x);
        let _ = data.write_f32::<NetworkEndian>(self.vel.y);
    }
}

pub struct Fighter {
    radar: UnifiedRadar,
    aimbot: AimBot,
    aim_id: u32,
    next_missile_id: u16,
}

impl Default for Fighter {
    fn default() -> Self {
        Self::new()
    }
}

impl Fighter {
    pub fn new() -> Fighter {
        Fighter {
            radar: UnifiedRadar::new(0.0..=40_000., 0., TAU, TAU * TICK_LENGTH * 2.),
            aimbot: AimBot::new(70., -70_000.),
            aim_id: 0,
            next_missile_id: 0,
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
            if c.class == Class::Fighter && rand(0., 30.) < 1. {
                let (p, v, _) = c.pva_in(TICK_LENGTH);
                send_bytes(
                    &Message::TargetUpdate(TargetUpdate {
                        missile_id: PackedOptionU16::make(None),
                        target: TargetMsg {
                            class: c.class,
                            uncertanty: c.uncertanty() as f32,
                            pos: p.into(),
                            vel: v.into(),
                        },
                    })
                    .to_bytes(),
                );
            }
            c.debug();
            self.radar.start_tracking(c.id);
            if reload_ticks(1) == 0 {
                fire(1);
                send_bytes(
                    &Message::MissileInit(MissileInit {
                        missile_id: 0,
                        target: TargetMsg {
                            class: c.class,
                            uncertanty: c.uncertanty() as f32,
                            pos: c.position().into(),
                            vel: c.velocity().into(),
                        },
                    })
                    .to_bytes(),
                )
            }
        }
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

fn position_in(time: f64) -> Vec2 {
    position() + velocity() * time
}

fn class_timeout(class: Class) -> f64 {
    match class {
        Class::Frigate => 5.,
        Class::Cruiser => 10.,
        Class::Asteroid => 30.,
        Class::Missile | Class::Torpedo => 0.25,
        Class::Target | Class::Fighter | Class::Unknown => 2.5,
    }
}

struct LockingRadar {
    track: Option<Track>,
}
impl LockingRadar {
    fn new() -> Self {
        Self { track: None }
    }
    fn tick(&mut self) {
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

struct UnifiedRadar {
    next_id: u32,
    scan_heading: f64,
    scan_distance_range: RangeInclusive<f64>,
    scan_direction: f64,
    scan_angle_max: f64,
    scan_fov: f64,
    contacts: Vec<Track>,
    tracking: Vec<u32>,
    check_behind: bool,
}

impl UnifiedRadar {
    const CONTACT_FUSE_PER_DIST: f64 = 0.1;
    const CONTACT_FUSE_MIN: f64 = 50.;

    fn new(
        scan_distance_range: RangeInclusive<f64>,
        scan_direction: f64,
        scan_angle_max: f64,
        scan_fov: f64,
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
        }
    }

    fn evict_contacts(&mut self) {
        self.contacts
            .retain(|c| elapsed(c.last_seen) <= class_timeout(c.class));
        self.tracking
            .retain(|id| self.contacts.iter().any(|c| c.id == *id));
    }

    fn prepare_track_tick(&mut self, track_idx: usize) {
        let track = self
            .contacts
            .iter()
            .find(|c| c.id == self.tracking[track_idx])
            .unwrap();

        let (h, a, d) = track.radar_track_look();
        let w = a / d;
        set_radar_width(w);
        set_radar_heading(h);
        set_radar_min_distance(d - 0.5 * a);
        set_radar_max_distance(d + 0.5 * a);
    }
    fn prepare_scan_tick(&mut self) {
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

    fn tick(&mut self) {
        if let Some(contact) = scan() {
            self.integrate_contact(&contact);
        }

        let select = current_tick() as usize % (self.tracking.len() + 1);
        if select > 0 {
            self.prepare_track_tick(select - 1);
        } else {
            self.prepare_scan_tick();
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
            self.next_id += 1;
        }
    }

    fn start_tracking(&mut self, id: u32) {
        if self.contacts.iter().any(|c| c.id == id) {
            self.tracking.push(id);
            self.tracking.sort_unstable();
            self.tracking.dedup();
        }
    }

    fn prio_track(&self) -> Option<&Track> {
        self.contacts
            .iter()
            .max_by_key(|t| (t.priority() * 1000.) as i64)
    }

    fn add_contact(&mut self, pos: Vec2, vel: Vec2, class: Class) -> u32 {
        let contact = Track::new(self.next_id, class, pos, vel);
        self.contacts.push(contact);
        self.next_id += 1;
        self.next_id - 1
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

fn class_priority(class: Class) -> f64 {
    match class {
        Class::Cruiser => 0.9,
        Class::Frigate => 0.7,
        Class::Fighter => 0.5,
        Class::Torpedo => 0.4,
        Class::Missile => 0.3,
        Class::Target => 0.2,
        Class::Unknown => 0.1,
        Class::Asteroid => 0.,
    }
}

fn inverse_prio(value: f64, half_point: f64) -> f64 {
    half_point / (value + half_point)
}

#[derive(Clone)]
struct Track {
    id: u32,
    class: Class,
    last_seen: f64,
    state: TrackState,
}

impl Track {
    fn priority(&self) -> f64 {
        let time = elapsed(self.last_seen);
        let dist = self.distance();
        let intercept_time = self.projectile_impact_time(0.);
        let intercept_dist = self
            .position_in(intercept_time)
            .distance(position_in(intercept_time));
        let m = if matches!(self.class, Class::Missile | Class::Torpedo) {
            10.
        } else {
            0.1
        };
        class_priority(self.class)
            + inverse_prio(dist, 5000.)
            + inverse_prio(time, 10.)
            + m * inverse_prio(intercept_dist, 100.) * m * inverse_prio(intercept_time, 5.).max(0.)
    }
    fn from_contact(id: u32, contact: &ScanResult) -> Self {
        let noise = Vec2::from(0.5 * contact.snr.recip() * -contact.rssi);
        Self {
            id,
            class: contact.class,
            last_seen: current_time(),
            state: TrackState {
                p: contact.position,
                v: contact.velocity,
                a: Vec2::zero(),
                p_c: noise,
                v_c: noise,
                a_c: noise,
            },
        }
    }
    fn with_uncertanty(id: u32, class: Class, pos: Vec2, vel: Vec2, uncertanty: f64) -> Self {
        Self {
            id,
            class,
            last_seen: current_time(),
            state: TrackState {
                p: pos,
                v: vel,
                a: Vec2::zero(),
                p_c: Vec2::one() * uncertanty / SQRT_2,
                v_c: Vec2::one() * uncertanty / SQRT_2,
                a_c: Vec2::one() * uncertanty / SQRT_2,
            },
        }
    }
    fn new(id: u32, class: Class, pos: Vec2, vel: Vec2) -> Self {
        Self {
            id,
            class,
            last_seen: current_time(),
            state: TrackState {
                p: pos,
                v: vel,
                a: Vec2::zero(),
                p_c: Vec2::one(),
                v_c: Vec2::one(),
                a_c: Vec2::one(),
            },
        }
    }
    fn debug(&self) {
        let (p, v, a) = self.state.predict_pva(elapsed(self.last_seen));
        let color = class_color(self.class);
        draw_square(self.state.p, 100., color);
        draw_diamond(p, 100., color);
        draw_line(p, p + v, color);
        draw_polygon(p + a, 25., 8, 0., color);
        draw_text!(
            p + Vec2::new(100., 100.),
            color,
            "{} {:.2}",
            self.id,
            self.priority()
        );
    }
    fn pva(&self) -> (Vec2, Vec2, Vec2) {
        self.pva_in(0.)
    }
    fn pva_in(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        self.state.predict_pva(elapsed(self.last_seen) + time)
    }
    fn position(&self) -> Vec2 {
        self.position_in(0.)
    }
    fn position_in(&self, time: f64) -> Vec2 {
        let (p, _, _) = self.state.predict_pva(elapsed(self.last_seen) + time);
        p
    }
    fn velocity(&self) -> Vec2 {
        let (_, v, _) = self.state.predict_pva(elapsed(self.last_seen));
        v
    }
    fn closing_speed(&self) -> f64 {
        let dir = self.direction();
        velocity().dot(dir) + self.velocity().dot(-dir)
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
    /// heading, area, distance
    fn radar_track_look(&self) -> (f64, f64, f64) {
        let (p, _, _) = self
            .state
            .predict_pva(elapsed(self.last_seen) + 2. * TICK_LENGTH);
        let towards = p - position();
        (towards.angle(), 150. * self.uncertanty(), towards.length())
    }
    fn projectile_impact_time(&self, projectile_speed: f64) -> f64 {
        let time = self.distance() / (self.closing_speed() + projectile_speed);

        let rel_vel_pos = self.position_in(time) - velocity() * time;
        let bullet_pos =
            position() + (rel_vel_pos - position()).normalize() * projectile_speed * time;
        let mut closest_dist = rel_vel_pos.distance(bullet_pos);
        let mut closest_time = time;

        for i in -1..=10 {
            let t = time + time * i as f64 / 5.;
            let rel_vel_pos = self.position_in(t) - velocity() * t;
            let bullet_pos =
                position() + (rel_vel_pos - position()).normalize() * projectile_speed * t;
            let dist = rel_vel_pos.distance(bullet_pos);
            if dist < closest_dist {
                closest_time = t;
                closest_dist = dist;
            }
        }
        let time = closest_time;
        for i in -5..=5 {
            let t = time + time * i as f64 / 50.;
            let rel_vel_pos = self.position_in(t) - velocity() * t;
            let bullet_pos =
                position() + (rel_vel_pos - position()).normalize() * projectile_speed * t;
            let dist = rel_vel_pos.distance(bullet_pos);
            if dist < closest_dist {
                closest_time = t;
                closest_dist = dist;
            }
        }
        let time = closest_time;
        for i in -5..=5 {
            let t = time + time * i as f64 / 500.;
            let rel_vel_pos = self.position_in(t) - velocity() * t;
            let bullet_pos =
                position() + (rel_vel_pos - position()).normalize() * projectile_speed * t;
            let dist = rel_vel_pos.distance(bullet_pos);
            if dist < closest_dist {
                closest_time = t;
                closest_dist = dist;
            }
        }
        let time = closest_time;
        for i in -5..=5 {
            let t = time + time * i as f64 / 5000.;
            let rel_vel_pos = self.position_in(t) - velocity() * t;
            let bullet_pos =
                position() + (rel_vel_pos - position()).normalize() * projectile_speed * t;
            let dist = rel_vel_pos.distance(bullet_pos);
            if dist < closest_dist {
                closest_time = t;
                closest_dist = dist;
            }
        }
        closest_time
    }
    fn closest_intercept_time(&self) -> f64 {
        self.projectile_impact_time(0.)
    }
    fn projectile_lead_heading(&self, projectile_speed: f64) -> f64 {
        let t = self.projectile_impact_time(projectile_speed);
        ((self.position_in(t) - velocity() * t) - position()).angle()
    }
    fn update(&mut self, observ: &ScanResult) {
        self.state.update(observ, elapsed(self.last_seen));
        self.class = observ.class;
        self.last_seen = current_time();
    }
    fn update_fixed_noise(&mut self, pos: Vec2, vel: Vec2, noise: f64) {
        self.state
            .update_fixed_noise(pos, vel, noise, elapsed(self.last_seen));
        self.last_seen = current_time();
    }

    fn uncertanty(&self) -> f64 {
        let (p_c, v_c, a_c) = self.state.predict_cov(elapsed(self.last_seen));
        p_c.length() + v_c.length() + a_c.length()
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
    fn predict_pva(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        let t = time;
        (
            self.p + self.v * t + self.a * t.powi(2) * 0.5,
            self.v + self.a * t,
            self.a,
        )
    }
    fn predict_cov(&self, time: f64) -> (Vec2, Vec2, Vec2) {
        let t = time;
        let p_noise = Vec2::from((1. / 6.) * t.powi(3));
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

    fn update_fixed_noise(&mut self, pos: Vec2, vel: Vec2, noise: f64, time: f64) {
        let time = if time == 0. { TICK_LENGTH / 2. } else { time };
        let acc = (vel - self.v) / time;
        let (p, v, a) = self.predict_pva(time);
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
    fn update(&mut self, observ: &ScanResult, time: f64) {
        let noise = observ.snr.recip() * -observ.rssi;
        self.update_fixed_noise(observ.position, observ.velocity, noise, time);
    }
}
