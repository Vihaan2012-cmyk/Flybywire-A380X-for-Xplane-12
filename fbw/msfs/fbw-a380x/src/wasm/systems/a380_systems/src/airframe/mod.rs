use systems::{
    airframe::{CenterOfGravityData, WeightData},
    fuel::FuelPayload,
    payload::{CargoPayload, LoadsheetInfo, PassengerPayload},
    simulation::{InitContext, SimulationElement, SimulationElementVisitor, SimulatorWriter, VariableIdentifier, Write},
};
use uom::si::{
    f64::Mass,
    mass::{kilogram, pound},
};

#[cfg(test)]
mod test;

const FLIGHT_ENVELOPE_FWD_FLAT_CG_PERCENT: f64 = 28.0;
const FLIGHT_ENVELOPE_FWD_BREAK_KG: f64 = 375_000.0;
const FLIGHT_ENVELOPE_FWD_MAX_KG: f64 = 510_000.0;
const FLIGHT_ENVELOPE_FWD_MAX_CG_PERCENT: f64 = 35.0;
const FLIGHT_ENVELOPE_AFT_CG_PERCENT: f64 = 44.0;
const FWD_LIMIT_ANNUNCIATION_MARGIN_PERCENT: f64 = 0.5;
const EXCESS_AFT_CG_PERCENT: f64 = 50.0;
const ZFW_CG_DISAGREE_PERCENT: f64 = 8.0;
const WEIGHT_DISAGREE_LB: f64 = 198_400.0;

fn flight_envelope_fwd_cg_percent(gross_weight_kg: f64) -> f64 {
    if gross_weight_kg <= FLIGHT_ENVELOPE_FWD_BREAK_KG {
        FLIGHT_ENVELOPE_FWD_FLAT_CG_PERCENT
    } else if gross_weight_kg >= FLIGHT_ENVELOPE_FWD_MAX_KG {
        FLIGHT_ENVELOPE_FWD_MAX_CG_PERCENT
    } else {
        let frac = (gross_weight_kg - FLIGHT_ENVELOPE_FWD_BREAK_KG) / (FLIGHT_ENVELOPE_FWD_MAX_KG - FLIGHT_ENVELOPE_FWD_BREAK_KG);
        FLIGHT_ENVELOPE_FWD_FLAT_CG_PERCENT + frac * (FLIGHT_ENVELOPE_FWD_MAX_CG_PERCENT - FLIGHT_ENVELOPE_FWD_FLAT_CG_PERCENT)
    }
}

pub struct AirframeCgAlerts {
    weight_disagree_id: VariableIdentifier,
    zfw_cg_disagree_id: VariableIdentifier,
    cg_out_of_range_id: VariableIdentifier,
    cg_at_fwd_limit_id: VariableIdentifier,
    cg_excess_aft_id: VariableIdentifier,
    to_cg_out_of_range_id: VariableIdentifier,
    weight_disagree: bool,
    zfw_cg_disagree: bool,
    cg_out_of_range: bool,
    cg_at_fwd_limit: bool,
    cg_excess_aft: bool,
    to_cg_out_of_range: bool,
}
impl AirframeCgAlerts {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            weight_disagree_id: context.get_identifier("AIRFRAME_WEIGHT_DISAGREE".to_owned()),
            zfw_cg_disagree_id: context.get_identifier("AIRFRAME_ZFW_CG_DISAGREE".to_owned()),
            cg_out_of_range_id: context.get_identifier("AIRFRAME_CG_OUT_OF_RANGE".to_owned()),
            cg_at_fwd_limit_id: context.get_identifier("AIRFRAME_CG_AT_FWD_LIMIT".to_owned()),
            cg_excess_aft_id: context.get_identifier("AIRFRAME_CG_EXCESS_AFT".to_owned()),
            to_cg_out_of_range_id: context.get_identifier("AIRFRAME_TO_CG_OUT_OF_RANGE".to_owned()),
            weight_disagree: false,
            zfw_cg_disagree: false,
            cg_out_of_range: false,
            cg_at_fwd_limit: false,
            cg_excess_aft: false,
            to_cg_out_of_range: false,
        }
    }

    fn update(
        &mut self,
        gross_weight_kg: f64,
        target_gross_weight_kg: f64,
        zfw_cg_percent_mac: f64,
        target_zfw_cg_percent_mac: f64,
        gw_cg_percent_mac: f64,
        to_cg_percent_mac: f64,
        target_to_cg_percent_mac: f64,
    ) {
        let weight_disagree_limit_kg = Mass::new::<pound>(WEIGHT_DISAGREE_LB).get::<kilogram>();
        self.weight_disagree = (gross_weight_kg - target_gross_weight_kg).abs() > weight_disagree_limit_kg;
        self.zfw_cg_disagree = (zfw_cg_percent_mac - target_zfw_cg_percent_mac).abs() > ZFW_CG_DISAGREE_PERCENT;
        self.cg_excess_aft = gw_cg_percent_mac > EXCESS_AFT_CG_PERCENT;

        let fwd_edge = flight_envelope_fwd_cg_percent(gross_weight_kg);
        self.cg_at_fwd_limit = gw_cg_percent_mac < fwd_edge + FWD_LIMIT_ANNUNCIATION_MARGIN_PERCENT;
        self.cg_out_of_range = to_cg_percent_mac < fwd_edge || to_cg_percent_mac > FLIGHT_ENVELOPE_AFT_CG_PERCENT;

        let target_fwd_edge = flight_envelope_fwd_cg_percent(target_gross_weight_kg);
        self.to_cg_out_of_range = target_to_cg_percent_mac < target_fwd_edge || target_to_cg_percent_mac > FLIGHT_ENVELOPE_AFT_CG_PERCENT;
    }
}
impl SimulationElement for AirframeCgAlerts {
    fn write(&self, writer: &mut SimulatorWriter) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        writer.write(&self.weight_disagree_id, b(self.weight_disagree));
        writer.write(&self.zfw_cg_disagree_id, b(self.zfw_cg_disagree));
        writer.write(&self.cg_out_of_range_id, b(self.cg_out_of_range));
        writer.write(&self.cg_at_fwd_limit_id, b(self.cg_at_fwd_limit));
        writer.write(&self.cg_excess_aft_id, b(self.cg_excess_aft));
        writer.write(&self.to_cg_out_of_range_id, b(self.to_cg_out_of_range));
    }
}

pub struct A380Airframe {
    center_of_gravity: CenterOfGravityData,
    weight: WeightData,
    cg_alerts: AirframeCgAlerts,
    // trim_horizontal_stabiliser: f64,
}
impl A380Airframe {
    const LOADSHEET: LoadsheetInfo = LoadsheetInfo {
        operating_empty_weight_kg: 300007.12,
        operating_empty_position: (6.47, 0., 0.),
        per_pax_weight_kg: 84.,
        mean_aerodynamic_chord_size: 40.35,
        leading_edge_mean_aerodynamic_chord: 21.09,
    };

    pub fn new(context: &mut InitContext) -> Self {
        A380Airframe {
            center_of_gravity: CenterOfGravityData::new(context),
            weight: WeightData::new(context),
            cg_alerts: AirframeCgAlerts::new(context),
            // trim_horizontal_stabiliser: 0.,
        }
    }

    pub(crate) fn get_loadsheet() -> &'static LoadsheetInfo {
        &Self::LOADSHEET
    }

    #[cfg(test)]
    fn zero_fuel_weight_center_of_gravity(&self) -> f64 {
        self.center_of_gravity.zero_fuel_weight_center_of_gravity()
    }

    #[cfg(test)]
    fn gross_weight_center_of_gravity(&self) -> f64 {
        self.center_of_gravity.gross_weight_center_of_gravity()
    }

    #[cfg(test)]
    // TODO: To be iterated upon in the future
    fn take_off_center_of_gravity(&self) -> f64 {
        self.center_of_gravity.take_off_center_of_gravity()
    }

    #[cfg(test)]
    fn target_zero_fuel_weight_center_of_gravity(&self) -> f64 {
        self.center_of_gravity
            .target_zero_fuel_weight_center_of_gravity()
    }

    #[cfg(test)]
    fn target_gross_weight_center_of_gravity(&self) -> f64 {
        self.center_of_gravity
            .target_gross_weight_center_of_gravity()
    }

    #[cfg(test)]
    // TODO: To be iterated upon in the future
    fn target_take_off_center_of_gravity(&self) -> f64 {
        self.center_of_gravity.target_take_off_center_of_gravity()
    }

    fn convert_cg(cg: f64) -> f64 {
        -100. * (cg - Self::LOADSHEET.leading_edge_mean_aerodynamic_chord)
            / Self::LOADSHEET.mean_aerodynamic_chord_size
    }

    fn set_zero_fuel_weight_center_of_gravity(&mut self, zero_fuel_weight_cg: f64) {
        let zero_fuel_weight_center_of_gravity = Self::convert_cg(zero_fuel_weight_cg);
        self.center_of_gravity
            .set_zero_fuel_weight_center_of_gravity(zero_fuel_weight_center_of_gravity)
    }

    fn set_gross_weight_center_of_gravity(&mut self, gross_weight_cg: f64) {
        let gross_weight_center_of_gravity = Self::convert_cg(gross_weight_cg);
        self.center_of_gravity
            .set_gross_weight_center_of_gravity(gross_weight_center_of_gravity);
    }

    fn set_take_off_center_of_gravity(&mut self, to_cg: f64) {
        let take_off_center_of_gravity = Self::convert_cg(to_cg);
        self.center_of_gravity
            .set_take_off_center_of_gravity(take_off_center_of_gravity);
    }

    fn set_target_zero_fuel_weight_center_of_gravity(&mut self, target_zero_fuel_weight_cg: f64) {
        let target_zero_fuel_weight_cg_percent_mac = Self::convert_cg(target_zero_fuel_weight_cg);
        self.center_of_gravity
            .set_target_zero_fuel_weight_center_of_gravity(target_zero_fuel_weight_cg_percent_mac)
    }

    fn set_target_gross_weight_center_of_gravity(&mut self, target_gross_weight_cg: f64) {
        let target_gross_weight_cg_percent_mac = Self::convert_cg(target_gross_weight_cg);
        self.center_of_gravity
            .set_target_gross_weight_center_of_gravity(target_gross_weight_cg_percent_mac);
    }

    fn set_target_take_off_center_of_gravity(
        &mut self,
        target_take_off_weight_cg_percent_mac: f64,
    ) {
        let target_take_off_weight_cg_percent_mac =
            Self::convert_cg(target_take_off_weight_cg_percent_mac);
        self.center_of_gravity
            .set_target_take_off_center_of_gravity(target_take_off_weight_cg_percent_mac);
    }

    fn set_zero_fuel_weight(&mut self, zero_fuel_weight: Mass) {
        self.weight.set_zero_fuel_weight(zero_fuel_weight);
    }

    fn set_gross_weight(&mut self, gross_weight: Mass) {
        self.weight.set_gross_weight(gross_weight);
    }

    fn set_take_off_weight(&mut self, take_off_weight: Mass) {
        self.weight.set_take_off_weight(take_off_weight);
    }

    fn set_target_zero_fuel_weight(&mut self, target_zero_fuel_weight: Mass) {
        self.weight
            .set_target_zero_fuel_weight(target_zero_fuel_weight);
    }

    fn set_target_gross_weight(&mut self, target_gross_weight: Mass) {
        self.weight.set_target_gross_weight(target_gross_weight);
    }

    fn set_target_take_off_weight(&mut self, target_take_off_weight: Mass) {
        self.weight
            .set_target_take_off_weight(target_take_off_weight);
    }

    pub(crate) fn update(
        &mut self,
        fuel_payload: &impl FuelPayload,
        pax_payload: &impl PassengerPayload,
        cargo_payload: &impl CargoPayload,
    ) {
        let total_pax = pax_payload.total_passenger_load();
        let total_cargo = cargo_payload.total_cargo_load();

        let operating_empty_weight =
            Mass::new::<kilogram>(Self::LOADSHEET.operating_empty_weight_kg);

        let empty_moment = Self::LOADSHEET.operating_empty_position.0 * operating_empty_weight;
        let pax_moment = total_pax * pax_payload.fore_aft_center_of_gravity();
        let cargo_moment = total_cargo * cargo_payload.fore_aft_center_of_gravity();

        let zero_fuel_weight_moment = empty_moment + pax_moment + cargo_moment;
        let zero_fuel_weight = operating_empty_weight + total_pax + total_cargo;
        let zero_fuel_weight_cg =
            zero_fuel_weight_moment.get::<kilogram>() / zero_fuel_weight.get::<kilogram>();

        self.set_zero_fuel_weight(zero_fuel_weight);
        self.set_zero_fuel_weight_center_of_gravity(zero_fuel_weight_cg);

        let total_target_pax = pax_payload.total_target_passenger_load();
        let total_target_cargo = cargo_payload.total_target_cargo_load();

        let pax_target_moment = total_target_pax * pax_payload.target_fore_aft_center_of_gravity();
        let cargo_target_moment =
            total_target_cargo * cargo_payload.target_fore_aft_center_of_gravity();

        let target_zero_fuel_weight_moment = empty_moment + pax_target_moment + cargo_target_moment;
        let target_zero_fuel_weight =
            operating_empty_weight + total_target_pax + total_target_cargo;

        let target_zero_fuel_weight_cg = target_zero_fuel_weight_moment.get::<kilogram>()
            / target_zero_fuel_weight.get::<kilogram>();

        self.set_target_zero_fuel_weight(target_zero_fuel_weight);
        self.set_target_zero_fuel_weight_center_of_gravity(target_zero_fuel_weight_cg);

        let fuel = fuel_payload.total_load();
        let fuel_moment = fuel * fuel_payload.fore_aft_center_of_gravity();

        let gross_weight_moment = zero_fuel_weight_moment + fuel_moment;
        let gross_weight = zero_fuel_weight + fuel;
        let gross_weight_cg =
            gross_weight_moment.get::<kilogram>() / gross_weight.get::<kilogram>();

        self.set_gross_weight(gross_weight);
        self.set_gross_weight_center_of_gravity(gross_weight_cg);

        let target_gross_weight_moment = target_zero_fuel_weight_moment + fuel_moment;
        let target_gross_weight = target_zero_fuel_weight + fuel;
        let target_gross_weight_cg =
            target_gross_weight_moment.get::<kilogram>() / target_gross_weight.get::<kilogram>();

        self.set_target_gross_weight(target_gross_weight);
        self.set_target_gross_weight_center_of_gravity(target_gross_weight_cg);

        // TODO: Implement Taxi Fuel Input/Calculation

        let tow = gross_weight;
        let to_cg = gross_weight_cg;

        self.set_take_off_weight(tow);
        self.set_take_off_center_of_gravity(to_cg);

        let target_tow = target_gross_weight;
        let target_to_cg = target_gross_weight_cg;

        self.set_target_take_off_weight(target_tow);
        self.set_target_take_off_center_of_gravity(target_to_cg);

        self.cg_alerts.update(
            gross_weight.get::<kilogram>(),
            target_gross_weight.get::<kilogram>(),
            Self::convert_cg(zero_fuel_weight_cg),
            Self::convert_cg(target_zero_fuel_weight_cg),
            Self::convert_cg(gross_weight_cg),
            Self::convert_cg(to_cg),
            Self::convert_cg(target_to_cg),
        );
    }
}
impl SimulationElement for A380Airframe {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.center_of_gravity.accept(visitor);
        self.weight.accept(visitor);
        self.cg_alerts.accept(visitor);

        visitor.visit(self);
    }
}
