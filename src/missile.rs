use super::{
    elapsed,
    message::{Msg, MsgKind, QuantTrack},
    position_in,
    radar::LockingRadar,
    track::Track,
};
use oort_api::prelude::{maths_rs::prelude::Base, *};

const SHRAPNEl_MAX_RANGE: f64 = 300.;
const SHRAPNEl_OPT_RANGE: f64 = 100.;
const SHRAPNEl_SPEED: f64 = 300. / (10. * TICK_LENGTH);

const SHRAPNEL_OPT_TTI: f64 = SHRAPNEl_OPT_RANGE / SHRAPNEl_SPEED;
const SHRAPNEL_MAX_TTI: f64 = SHRAPNEl_MAX_RANGE / SHRAPNEl_SPEED;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MissileStage {
    Boost,
    Cruise,
    Terminal,
}

pub struct Missile {
    radar: LockingRadar,
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
            radar: LockingRadar::new(),
            stage: MissileStage::Boost,
        }
    }

    pub fn receive(&mut self) {
        let Some(msg) = Msg::receive() else {
            return;
        };

        match msg.msg {
            MsgKind::SenderPos(_) => (),
            MsgKind::SenderTgt(track) => self.receive_target_update(track),
            MsgKind::Tracks(_) => (),
            MsgKind::SwitchChannel(channel) => set_radio_channel(channel as usize),
        }
    }

    fn receive_target_update(&mut self, t: QuantTrack) {
        let track = Track::from(t);
        draw_diamond(
            (msg.target.pos + msg.target.vel * elapsed(msg.target.send_time as f64) as f32).into(),
            1000.,
            0xFFFF_FFFF,
        );
        draw_square(msg.target.pos.into(), 1000., 0xFFFF_FFFF);

        if let Some(track) = self.radar.track.as_mut() {
            track.update_fixed_noise(
                (msg.target.pos + msg.target.vel * elapsed(msg.target.send_time as f64) as f32)
                    .into(),
                msg.target.vel.into(),
                msg.target.uncertanty.into(),
                elapsed(msg.target.send_time as f64),
            );
        } else {
            self.radar.track = Some(Track::with_uncertanty(
                0,
                msg.target.class,
                (msg.target.pos + msg.target.vel * elapsed(msg.target.send_time as f64) as f32)
                    .into(),
                msg.target.vel.into(),
                msg.target.uncertanty.into(),
            ));
        }
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
            let message = MyMessage::PairTti(PairTti {
                sender_missile_id: my_id,
                time_to_impact: (track.distance() / track.closing_speed()) as f32,
            });
            send_bytes(&message.to_bytes());
            draw_triangle(position(), 1000., 0xFFA0_A0FF);
        } else if rand(0., 50.) < 1. {
            let message = MyMessage::TargetUpdate(TargetUpdate {
                missile_id: None,
                target: TargetMsg {
                    class: track.class,
                    send_time: track.last_seen as f32,
                    uncertanty: track.uncertanty() as f32,
                    pos: track.position().into(),
                    vel: track.velocity().into(),
                },
            });
            send_bytes(&message.to_bytes());
            draw_triangle(position(), 1000., 0xFF50_50FF);
        }
    }

    pub fn tick(&mut self) {
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
        accelerate(acc);
        debug!("tti {:.3}", tti);

        let shrapnel_tti =
            contact.distance() / (contact.closing_speed() + 0.5 * max_fuel_used + SHRAPNEl_SPEED);
        debug!("s-tti {:.3}", shrapnel_tti);

        if shrapnel_tti - SHRAPNEL_OPT_TTI < 0.25 {
            let shrapnel_pos = contact.position_in(shrapnel_tti);
            let shrapnel_aim = (shrapnel_pos - position()).angle();
            draw_triangle(shrapnel_pos, 50., 0xFFFF_0000);
            turn(50. * angle_diff(heading(), shrapnel_aim));
        }

        if health() < 20. || shrapnel_tti <= SHRAPNEL_OPT_TTI {
            explode();
        }
    }
}
