use super::{
    message::{MissileInit, MyMessage, Pair, PairTti, TargetMsg, TargetUpdate},
    position_in,
    radar::LockingRadar,
    track::Track,
};
use oort_api::prelude::{maths_rs::prelude::Base, *};

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
        if let Some(message) = receive_bytes().and_then(MyMessage::from_bytes) {
            match message {
                MyMessage::MissileInit(msg) => self.receive_missile_init(msg),
                MyMessage::TargetUpdate(msg) => self.receive_target_update(msg),
                MyMessage::PotentialTarget(msg) => self.receive_potential_target(msg),
                MyMessage::Pair(msg) => self.receive_pair(msg),
                MyMessage::PairTti(msg) => self.receive_pair_tti(msg),
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
            if let Some(intercept_time) = c.impact_time(0.) {
                if c.position_in(intercept_time)
                    .distance(position_in(intercept_time))
                    > 50.
                {
                    return;
                }
                self.radar.track = Some(c);
                self.explode_dist = 25.;
            }
        }
    }

    fn receive_target_update(&mut self, msg: TargetUpdate) {
        if msg
            .missile_id
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
        self.explode_dist = 200.;
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
                    send_tick: current_tick(),
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
        if fuel() < 10. {
            turn(50. * angle_diff(heading(), impact_dir.angle()));
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
