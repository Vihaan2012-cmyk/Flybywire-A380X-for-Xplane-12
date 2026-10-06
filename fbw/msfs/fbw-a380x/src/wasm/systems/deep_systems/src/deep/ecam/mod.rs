pub mod cond_json;
#[cfg(test)]
mod dump_published;
pub mod codegen;
pub mod fbw;
pub mod fbw_codegen;
pub mod generated_alerts;
pub mod wave5_w_ata21_1_alerts;
pub mod wave5_t02_hydraulic_tests_alerts;
pub mod wave5_f02_fuel_indication_alerts;
pub mod wave5_w_ata34_1_alerts;
pub mod wave5_e02_fan_damage_alerts;
pub mod ids;
pub mod patches;

#[cfg(test)]
mod fbw_tests;

#[cfg(test)]
#[cfg(feature = "js")]
mod tests;
