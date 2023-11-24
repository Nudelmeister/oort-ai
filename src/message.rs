use super::track::Track;
use oort_api::{
    prelude::{
        byteorder::{ReadBytesExt, WriteBytesExt},
        current_tick, current_time, id,
        maths_rs::{prelude::Base, vec},
        receive_bytes, send_bytes, world_size, Vec2,
    },
    Class,
};
use std::io::Cursor;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Msg {
    /// `id() % 256`
    pub sender_id: u8,
    /// `current_tick() % 256`
    pub send_tick: u8,
    pub msg: MsgKind,
    pub chksum: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MsgKind {
    SenderPos(QuantState),
    SenderTgt(QuantTrack),
    Tracks([QuantTrack; 4]),
    SwitchChannel(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantTrack {
    pub class: Class,
    pub state: QuantState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantState {
    pub pos: vec::Vec2<i8>,
    pub vel: vec::Vec2<i8>,
    pub noise: i8,
}

impl Msg {
    pub fn receive() -> Option<Self> {
        let data = receive_bytes()?;
        let chksum = data.iter().fold(0u8, |s, e| s.wrapping_add(*e));

        let mut data = Cursor::new(data.as_slice());
        let sender_id = data.read_u8().ok()?;
        let send_tick = data.read_u8().ok()?;
        if send_tick != ((current_tick() - 1) % 256) as u8 {
            return None;
        }
        let msg = MsgKind::from_bytes(&mut data)?;
        let received_chksum = data.read_u8().ok()?;
        if received_chksum != chksum {
            return None;
        }

        Some(Self {
            sender_id,
            send_tick,
            msg,
            chksum,
        })
    }
    pub fn send(msg: MsgKind) {
        let sender_id = (id() % 256) as u8;
        let send_tick = (current_tick() % 256) as u8;

        let mut buf = [0; 32];
        let mut data = Cursor::new(buf.as_mut_slice());
        data.write_u8(sender_id).unwrap();
        data.write_u8(send_tick).unwrap();
        msg.to_bytes(&mut data);
        let chksum_idx = data.position();
        let mut chksum = buf.iter().fold(0u8, |s, e| s.wrapping_add(*e));
        chksum = chksum.wrapping_add(chksum);
        buf[chksum_idx as usize] = chksum;

        send_bytes(&buf);
    }
}

impl MsgKind {
    fn from_bytes(data: &mut Cursor<&[u8]>) -> Option<Self> {
        match data.read_u8().ok()? {
            0 => Some(Self::SenderPos(QuantState::from_bytes(data)?)),
            1 => Some(Self::SenderTgt(QuantTrack::from_bytes(data)?)),
            2 => Some(Self::Tracks(Self::read_tracks(data)?)),
            3 => Some(Self::SwitchChannel(data.read_u8().ok()?)),
            _ => None,
        }
    }
    fn read_tracks(data: &mut Cursor<&[u8]>) -> Option<[QuantTrack; 4]> {
        let mut arr = [QuantTrack {
            class: Class::Unknown,
            state: QuantState {
                pos: vec::Vec2::zero(),
                vel: vec::Vec2::zero(),
                noise: 0,
            },
        }; 4];

        for t in &mut arr {
            *t = QuantTrack::from_bytes(data)?;
        }

        Some(arr)
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        match self {
            MsgKind::SenderPos(state) => {
                data.write_u8(0).unwrap();
                state.to_bytes(data);
            }
            MsgKind::SenderTgt(track) => {
                data.write_u8(1).unwrap();
                track.to_bytes(data);
            }
            MsgKind::Tracks(tracks) => {
                data.write_u8(2).unwrap();
                tracks.into_iter().for_each(|t| t.to_bytes(data));
            }
            MsgKind::SwitchChannel(c) => {
                data.write_u8(3).unwrap();
                data.write_u8(c).unwrap();
            }
        }
    }
}

impl From<Track> for QuantTrack {
    fn from(value: Track) -> Self {
        Self {
            class: value.class,
            state: value.into(),
        }
    }
}

impl QuantTrack {
    fn from_bytes(data: &mut Cursor<&[u8]>) -> Option<Self> {
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
        let state = QuantState::from_bytes(data)?;

        Some(Self { class, state })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        data.write_u8(match self.class {
            Class::Fighter => 0,
            Class::Frigate => 1,
            Class::Cruiser => 2,
            Class::Asteroid => 3,
            Class::Target => 4,
            Class::Missile => 5,
            Class::Torpedo => 6,
            Class::Unknown => 7,
        })
        .unwrap();
        self.state.to_bytes(data);
    }
}

impl From<Track> for QuantState {
    fn from(value: Track) -> Self {
        let pos_conv = 64. * world_size();
        let vel_conv = 64. * 1000.;
        let unc_conv = 64. * 500.;

        let (p, v, _) = value.pva();
        let uncertanty = value.uncertanty();

        Self {
            pos: vec::Vec2::<i8>::new(
                (p.x / pos_conv).round() as i8,
                (p.y / pos_conv).round() as i8,
            ),
            vel: vec::Vec2::<i8>::new(
                (v.x / vel_conv).round() as i8,
                (v.y / vel_conv).round() as i8,
            ),
            noise: (uncertanty / unc_conv) as i8,
        }
    }
}

impl QuantState {
    fn from_bytes(data: &mut Cursor<&[u8]>) -> Option<Self> {
        let px = data.read_i8().ok()?;
        let py = data.read_i8().ok()?;
        let vx = data.read_i8().ok()?;
        let vy = data.read_i8().ok()?;
        let n = data.read_i8().ok()?;

        Some(Self {
            pos: vec::Vec2::new(px, py),
            vel: vec::Vec2::new(vx, vy),
            noise: n,
        })
    }
    fn to_bytes(self, data: &mut Cursor<&mut [u8]>) {
        data.write_i8(self.pos.x).unwrap();
        data.write_i8(self.pos.y).unwrap();
        data.write_i8(self.vel.x).unwrap();
        data.write_i8(self.vel.y).unwrap();
        data.write_i8(self.noise).unwrap();
    }
}
