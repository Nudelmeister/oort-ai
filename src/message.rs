use oort_api::{
    prelude::{
        byteorder::{NetworkEndian, ReadBytesExt, WriteBytesExt},
        maths_rs::Vec2f,
    },
    Class,
};
use std::io::Cursor;

#[derive(Clone, Copy)]
pub enum MyMessage {
    MissileInit(MissileInit),
    TargetUpdate(TargetUpdate),
    PotentialTarget(TargetUpdate),
    Pair(Pair),
    PairTti(PairTti),
}
impl MyMessage {
    pub fn from_bytes(data: [u8; 32]) -> Option<Self> {
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
    pub fn to_bytes(self) -> [u8; 32] {
        let mut data = [0; 32];
        let mut cursor = Cursor::new(data.as_mut_slice());
        match self {
            MyMessage::MissileInit(msg) => {
                let _ = cursor.write_u8(0);
                msg.to_bytes(&mut cursor);
            }
            MyMessage::TargetUpdate(msg) => {
                let _ = cursor.write_u8(1);
                msg.to_bytes(&mut cursor);
            }
            MyMessage::PotentialTarget(msg) => {
                let _ = cursor.write_u8(2);
                msg.to_bytes(&mut cursor);
            }
            MyMessage::Pair(msg) => {
                let _ = cursor.write_u8(3);
                msg.to_bytes(&mut cursor);
            }
            MyMessage::PairTti(msg) => {
                let _ = cursor.write_u8(4);
                msg.to_bytes(&mut cursor);
            }
        };

        data
    }
}

#[derive(Clone, Copy)]
pub struct PairTti {
    pub sender_missile_id: u16,
    pub time_to_impact: f32,
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
pub struct Pair {
    pub missile_id_a: u16,
    pub missile_id_b: u16,
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
pub struct MissileInit {
    pub missile_id: u16,
    pub target: TargetMsg,
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
pub struct TargetUpdate {
    pub missile_id: Option<u16>,
    pub target: TargetMsg,
}
impl TargetUpdate {
    fn from_bytes(data: &mut Cursor<[u8; 32]>) -> Option<Self> {
        let missile_id = data.read_u16::<NetworkEndian>().ok()?;
        let missile_id = (missile_id != u16::MAX).then_some(missile_id);
        let target = TargetMsg::from_bytes(data)?;
        Some(Self { missile_id, target })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        let _ = data.write_u16::<NetworkEndian>(self.missile_id.unwrap_or(u16::MAX));
        self.target.to_bytes(data);
    }
}

#[derive(Clone, Copy)]
pub struct TargetMsg {
    pub class: Class,
    pub send_tick: u32,
    pub uncertanty: f32,
    pub pos: Vec2f,
    pub vel: Vec2f,
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

        let send_tick = data.read_u32::<NetworkEndian>().ok()?;
        let uncertanty = data.read_f32::<NetworkEndian>().ok()?;

        let px = data.read_f32::<NetworkEndian>().ok()?;
        let py = data.read_f32::<NetworkEndian>().ok()?;
        let vx = data.read_f32::<NetworkEndian>().ok()?;
        let vy = data.read_f32::<NetworkEndian>().ok()?;

        Some(Self {
            class,
            send_tick,
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
        let _ = data.write_u32::<NetworkEndian>(self.send_tick);
        let _ = data.write_f32::<NetworkEndian>(self.uncertanty);
        let _ = data.write_f32::<NetworkEndian>(self.pos.x);
        let _ = data.write_f32::<NetworkEndian>(self.pos.y);
        let _ = data.write_f32::<NetworkEndian>(self.vel.x);
        let _ = data.write_f32::<NetworkEndian>(self.vel.y);
    }
}
