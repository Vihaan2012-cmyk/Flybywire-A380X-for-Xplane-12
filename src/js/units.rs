//! MSFS simulation variable units and the conversions between them.
//!
//! MSFS converts a simulator variable into whatever unit a script asks for
//! (`SimVar.GetSimVarValue('AIRSPEED INDICATED', 'meters per second')`).
//! Units are matched without regard to case or repeated spaces, and in the
//! singular or plural, as MSFS accepts them. Each unit belongs to a
//! dimension; a conversion within a dimension is linear (a scale and, for
//! temperatures, an offset) except for the BCD frequency encodings.
//!
//! `number`, `enum`, `mask`, `flags` and the boolean units carry no dimension:
//! reading in them gives the variable in the unit it is kept in.

/// What a unit measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dimension {
    Length,
    Speed,
    Acceleration,
    Angle,
    AngularVelocity,
    AngularAcceleration,
    Time,
    Mass,
    MassFlow,
    VolumeFlow,
    Force,
    Torque,
    Pressure,
    Temperature,
    Volume,
    Density,
    Frequency,
    Ratio,
    Voltage,
    Current,
    Power,
    MomentOfInertia,
    Mach,
    Area,
}

/// A unit: `value_in_base = value * scale + offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Unit {
    pub dimension: Dimension,
    pub scale: f64,
    pub offset: f64,
    pub encoding: Encoding,
}

/// How a frequency is written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Encoding {
    Plain,
    /// "Frequency BCD16": four BCD digits of MHz × 100 without the leading 1
    /// (0x0850 is 108.50 MHz).
    Bcd16,
    /// "Frequency BCD32": eight BCD digits of MHz × 100 000 (0x10850000 is
    /// 108.5 MHz).
    Bcd32,
    /// "Frequency ADF BCD32": five BCD digits of kHz × 10, shifted up 12 bits
    /// (0x03500000 is 350.0 kHz).
    AdfBcd32,
}

/// How a script's unit relates to values: no conversion, a boolean, a struct
/// or string (not numbers), or a unit with a dimension.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    /// `number`, `enum` and similar: the value as it is kept.
    Plain,
    Bool,
    String,
    /// `latlonalt`, `latlonaltpbh`, `pbh`, `pid_struct`, `xyz`.
    Struct,
    Measure(Unit),
}

const FT: f64 = 0.3048;
const NM: f64 = 1852.;
const LB: f64 = 0.453_592_37;
const G0: f64 = 9.806_65;
const INHG: f64 = 3386.389;
const GALLON: f64 = 0.003_785_411_784;
const DEG: f64 = std::f64::consts::PI / 180.;
const SLUG: f64 = 14.593_902_937;

/// A unit name in the form units are looked up by: lower case, single spaces,
/// no surrounding space.
pub fn normalise(unit: &str) -> String {
    unit.split_whitespace().collect::<Vec<_>>().join(" ").to_ascii_lowercase()
}

/// What a unit name means, or `None` if MSFS has no such unit.
pub fn kind(unit: &str) -> Option<Kind> {
    let name = normalise(unit);
    match name.as_str() {
        "" | "number" | "numbers" | "enum" | "enums" | "mask" | "flags" | "part" | "scalar" | "keyframe" | "keyframes"
        | "bco16" | "sint32" | "uint32" => return Some(Kind::Plain),
        "bool" | "boolean" => return Some(Kind::Bool),
        "string" => return Some(Kind::String),
        "latlonalt" | "latlonaltpbh" | "pbh" | "pid_struct" | "xyz" => return Some(Kind::Struct),
        _ => {}
    }
    measure(&name).map(Kind::Measure)
}

fn unit(dimension: Dimension, scale: f64) -> Unit {
    Unit { dimension, scale, offset: 0., encoding: Encoding::Plain }
}

/// The plural and "per" spellings MSFS accepts reduce to these singulars.
fn singular(name: &str) -> String {
    name.split(' ')
        .map(|word| match word {
            "feet" => "foot",
            "inches" => "inch",
            "knots" => "knot",
            "miles" => "mile",
            "meters" | "metres" | "metre" => "meter",
            "kilometers" | "kilometres" | "kilometre" => "kilometer",
            "centimeters" | "centimetres" | "centimetre" => "centimeter",
            "millimeters" | "millimetres" | "millimetre" => "millimeter",
            "yards" => "yard",
            "degrees" => "degree",
            "radians" => "radian",
            "grads" => "grad",
            "seconds" => "second",
            "minutes" => "minute",
            "hours" => "hour",
            "days" => "day",
            "pounds" => "pound",
            "kilograms" => "kilogram",
            "grams" => "gram",
            "slugs" => "slug",
            "newtons" => "newton",
            "pascals" => "pascal",
            "hectopascals" => "hectopascal",
            "kilopascals" => "kilopascal",
            "millibars" => "millibar",
            "bars" => "bar",
            "atmospheres" => "atmosphere",
            "gallons" => "gallon",
            "liters" | "litres" | "litre" => "liter",
            "volts" => "volt",
            "amperes" | "amps" | "amp" => "ampere",
            "watts" => "watt",
            "hertz" => "hertz",
            "rotations" => "rotation",
            "revolutions" => "revolution",
            other => other,
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn measure(name: &str) -> Option<Unit> {
    use Dimension::*;
    let name = singular(name);
    let u = match name.as_str() {
        // Length, in metres.
        "meter" => unit(Length, 1.),
        "centimeter" | "cm" => unit(Length, 0.01),
        "millimeter" | "mm" => unit(Length, 0.001),
        "kilometer" | "km" => unit(Length, 1000.),
        "foot" => unit(Length, FT),
        "inch" => unit(Length, 0.0254),
        "yard" => unit(Length, 0.9144),
        "mile" => unit(Length, 1609.344),
        "nautical mile" | "nmile" | "nmiles" => unit(Length, NM),
        "decinmile" | "decinmiles" => unit(Length, NM / 10.),
        // Speed, in metres per second.
        "meter per second" | "m/s" => unit(Speed, 1.),
        "meter per minute" => unit(Speed, 1. / 60.),
        "kilometer per hour" | "kph" | "km/h" => unit(Speed, 1000. / 3600.),
        "knot" | "kt" | "kts" => unit(Speed, NM / 3600.),
        "mile per hour" | "mph" => unit(Speed, 1609.344 / 3600.),
        "foot per second" | "ft/s" | "feet/second" => unit(Speed, FT),
        "foot per minute" | "ft/min" | "feet/minute" | "fpm" => unit(Speed, FT / 60.),
        "nautical mile per hour" => unit(Speed, NM / 3600.),
        // Acceleration, in metres per second squared.
        "meter per second squared" | "meters/second squared" => unit(Acceleration, 1.),
        "foot per second squared" | "feet/second squared" => unit(Acceleration, FT),
        "foot per minute per second" => unit(Acceleration, FT / 60.),
        "g force" | "gforce" => unit(Acceleration, G0),
        // Angle, in radians.
        "radian" | "rad" => unit(Angle, 1.),
        "degree" | "deg" | "degree latitude" | "degree longitude" | "degree angl16" | "degree angl32" => {
            unit(Angle, DEG)
        }
        "grad" => unit(Angle, std::f64::consts::PI / 200.),
        "radian latitude" | "radian longitude" => unit(Angle, 1.),
        // Angular velocity, in radians per second.
        "radian per second" => unit(AngularVelocity, 1.),
        "degree per second" | "deg/s" => unit(AngularVelocity, DEG),
        "degree per minute" => unit(AngularVelocity, DEG / 60.),
        "rpm" | "rotation per minute" | "revolution per minute" => unit(AngularVelocity, 2. * std::f64::consts::PI / 60.),
        // Angular acceleration, in radians per second squared.
        "radian per second squared" => unit(AngularAcceleration, 1.),
        "degree per second squared" => unit(AngularAcceleration, DEG),
        // Time, in seconds.
        "second" | "sec" => unit(Time, 1.),
        "minute" | "min" => unit(Time, 60.),
        "hour" | "hr" => unit(Time, 3600.),
        "day" => unit(Time, 86_400.),
        "hour over 10" => unit(Time, 360.),
        "year" => unit(Time, 31_536_000.),
        // Mass, in kilograms.
        "kilogram" | "kg" => unit(Mass, 1.),
        "gram" => unit(Mass, 0.001),
        "pound" | "lbs" | "lb" => unit(Mass, LB),
        "slug" => unit(Mass, SLUG),
        "tonne" | "metric ton" => unit(Mass, 1000.),
        "ton" | "short ton" => unit(Mass, 907.184_74),
        // Mass flow, in kilograms per second.
        "kilogram per second" => unit(MassFlow, 1.),
        "kilogram per hour" => unit(MassFlow, 1. / 3600.),
        "pound per hour" | "pph" => unit(MassFlow, LB / 3600.),
        // Volume flow, in cubic metres per second.
        "gallon per hour" | "gph" => unit(VolumeFlow, GALLON / 3600.),
        "liter per hour" => unit(VolumeFlow, 0.001 / 3600.),
        // Force, in newtons.
        "newton" => unit(Force, 1.),
        "pound-force" | "pound force" | "lbf" => unit(Force, LB * G0),
        "kilogram force" => unit(Force, G0),
        // Torque, in newton metres.
        "newton meter" => unit(Torque, 1.),
        "foot pound" | "foot-pound" | "ft-lb" | "ft lb" => unit(Torque, FT * LB * G0),
        "foot pound per second" => unit(Power, FT * LB * G0),
        // Pressure, in pascals.
        "pascal" | "pa" => unit(Pressure, 1.),
        "kilopascal" | "kpa" => unit(Pressure, 1000.),
        "hectopascal" | "hpa" => unit(Pressure, 100.),
        "millibar" | "mbar" | "mb" | "millibars scaler 16" => unit(Pressure, 100.),
        "bar" => unit(Pressure, 100_000.),
        "atmosphere" | "atm" => unit(Pressure, 101_325.),
        "psi" | "pound per square inch" | "pound-force per square inch" => unit(Pressure, LB * G0 / 0.0254 / 0.0254),
        "psf" | "pound per square foot" | "pound-force per square foot" => unit(Pressure, LB * G0 / FT / FT),
        "inhg" | "inch of mercury" | "in hg" => unit(Pressure, INHG),
        "mmhg" | "millimeter of mercury" => unit(Pressure, 133.322_387),
        "millimeter of water" | "mmh2o" => unit(Pressure, G0),
        "inch of water" => unit(Pressure, 249.088_91),
        "kilogram force per square centimeter" | "kgf meter squared" => unit(Pressure, G0 * 10_000.),
        // Temperature, in kelvin.
        "kelvin" => unit(Temperature, 1.),
        "celsius" | "degree celsius" => Unit { dimension: Temperature, scale: 1., offset: 273.15, encoding: Encoding::Plain },
        "fahrenheit" | "farenheit" | "degree fahrenheit" => {
            Unit { dimension: Temperature, scale: 5. / 9., offset: 273.15 - 32. * 5. / 9., encoding: Encoding::Plain }
        }
        "rankine" => unit(Temperature, 5. / 9.),
        // Volume, in cubic metres.
        "cubic meter" | "m^3" => unit(Volume, 1.),
        "liter" => unit(Volume, 0.001),
        "gallon" | "gal" => unit(Volume, GALLON),
        "cubic inch" | "cubic inches" | "in^3" => unit(Volume, 0.0254 * 0.0254 * 0.0254),
        "cubic foot" | "ft^3" => unit(Volume, FT * FT * FT),
        "cubic centimeter" | "cc" => unit(Volume, 1e-6),
        // Area, in square metres.
        "square meter" | "m^2" => unit(Area, 1.),
        "square foot" | "ft^2" => unit(Area, FT * FT),
        "square inch" | "in^2" => unit(Area, 0.0254 * 0.0254),
        // Density, in kilograms per cubic metre.
        "kilogram per cubic meter" | "kg/m^3" => unit(Density, 1.),
        "slug per cubic foot" | "slug per cubic feet" | "slug/ft^3" => unit(Density, SLUG / (FT * FT * FT)),
        "pound per gallon" => unit(Density, LB / GALLON),
        // Moment of inertia, in kilogram square metres.
        "kilogram meter squared" => unit(MomentOfInertia, 1.),
        "slug foot squared" | "slug feet squared" => unit(MomentOfInertia, SLUG * FT * FT),
        "pound square foot" => unit(MomentOfInertia, LB * FT * FT),
        // Frequency, in hertz.
        "hertz" | "hz" => unit(Frequency, 1.),
        "kilohertz" | "khz" => unit(Frequency, 1e3),
        "megahertz" | "mhz" => unit(Frequency, 1e6),
        "per second" => unit(Frequency, 1.),
        "per minute" => unit(Frequency, 1. / 60.),
        "per hour" => unit(Frequency, 1. / 3600.),
        "frequency bcd16" => Unit { encoding: Encoding::Bcd16, ..unit(Frequency, 1.) },
        "frequency bcd32" => Unit { encoding: Encoding::Bcd32, ..unit(Frequency, 1.) },
        "frequency adf bcd32" => Unit { encoding: Encoding::AdfBcd32, ..unit(Frequency, 1.) },
        // Ratios.
        "percent over 100" | "ratio" => unit(Ratio, 1.),
        "percent" | "percentage" => unit(Ratio, 0.01),
        "position" => unit(Ratio, 1.),
        "position 16k" => unit(Ratio, 1. / 16_384.),
        "position 32k" => unit(Ratio, 1. / 32_768.),
        "position 128" => unit(Ratio, 1. / 128.),
        "per mille" => unit(Ratio, 0.001),
        // Electricity and power.
        "volt" => unit(Voltage, 1.),
        "ampere" => unit(Current, 1.),
        "watt" => unit(Power, 1.),
        "kilowatt" | "kw" => unit(Power, 1000.),
        "horsepower" | "hp" => unit(Power, 745.699_872),
        "mach" | "machs" => unit(Mach, 1.),
        _ => return None,
    };
    Some(u)
}

/// Convert `value` from `from` into `to`, both of one dimension. `None` if
/// the dimensions differ.
pub fn convert(value: f64, from: &Unit, to: &Unit) -> Option<f64> {
    if from.dimension != to.dimension {
        return None;
    }
    let base = match from.encoding {
        Encoding::Plain => value * from.scale + from.offset,
        Encoding::Bcd16 => 1e6 * (100. + bcd_decode(value as u32) as f64 / 100.),
        Encoding::Bcd32 => 1e6 * bcd_decode(value as u32) as f64 / 100_000.,
        Encoding::AdfBcd32 => 100. * bcd_decode((value as u32) >> 12) as f64,
    };
    Some(match to.encoding {
        Encoding::Plain => (base - to.offset) / to.scale,
        Encoding::Bcd16 => bcd_encode((((base / 1e6) - 100.) * 100.).round().max(0.) as u64) as f64,
        Encoding::Bcd32 => bcd_encode((base / 1e6 * 100_000.).round().max(0.) as u64) as f64,
        Encoding::AdfBcd32 => ((bcd_encode((base / 100.).floor().max(0.) as u64) as u64) << 12) as u32 as f64,
    })
}

/// A conversion reduced to `value * scale + offset`, when it is linear.
pub fn linear(from: &Unit, to: &Unit) -> Option<(f64, f64)> {
    if from.dimension != to.dimension || from.encoding != Encoding::Plain || to.encoding != Encoding::Plain {
        return None;
    }
    Some((from.scale / to.scale, (from.offset - to.offset) / to.scale))
}

fn bcd_decode(bcd: u32) -> u64 {
    let mut out = 0u64;
    for shift in (0..32).step_by(4).rev() {
        out = out * 10 + ((bcd >> shift) & 0xf).min(9) as u64;
    }
    out
}

fn bcd_encode(mut value: u64) -> u32 {
    let mut out = 0u32;
    for shift in (0..32).step_by(4) {
        out |= ((value % 10) as u32) << shift;
        value /= 10;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(name: &str) -> Unit {
        match kind(name) {
            Some(Kind::Measure(u)) => u,
            other => panic!("{name}: {other:?}"),
        }
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6 * b.abs().max(1.)
    }

    #[test]
    fn names_match_regardless_of_case_spacing_and_number() {
        assert_eq!(m("Knots"), m("knot"));
        assert_eq!(m("feet  per   minute"), m("Foot per minute"));
        assert_eq!(m("Slugs per cubic feet"), m("slug per cubic foot"));
        assert_eq!(kind("Bool"), Some(Kind::Bool));
        assert_eq!(kind("SimVarValueType.Nope"), None);
    }

    #[test]
    fn linear_units_convert() {
        assert!(close(convert(100., &m("knots"), &m("meters per second")).unwrap(), 51.444_444));
        assert!(close(convert(1., &m("inHg"), &m("millibars")).unwrap(), 33.863_89));
        assert!(close(convert(15., &m("celsius"), &m("fahrenheit")).unwrap(), 59.));
        assert!(close(convert(0., &m("celsius"), &m("rankine")).unwrap(), 491.67));
        assert!(close(convert(50., &m("percent"), &m("percent over 100")).unwrap(), 0.5));
        assert!(close(convert(180., &m("degrees"), &m("radians")).unwrap(), std::f64::consts::PI));
        assert!(close(convert(1., &m("gallons"), &m("liters")).unwrap(), 3.785_411_784));
        assert!(convert(1., &m("feet"), &m("knots")).is_none());
    }

    #[test]
    fn linear_reduces_a_conversion_to_scale_and_offset() {
        let (scale, offset) = linear(&m("celsius"), &m("fahrenheit")).unwrap();
        assert!(close(100. * scale + offset, 212.));
        assert!(linear(&m("mhz"), &m("frequency bcd32")).is_none());
    }

    #[test]
    fn frequencies_convert_to_and_from_bcd() {
        assert_eq!(convert(108.5, &m("MHz"), &m("Frequency BCD32")).unwrap(), 0x1085_0000 as f64);
        assert_eq!(convert(108.5, &m("MHz"), &m("Frequency BCD16")).unwrap(), 0x0850 as f64);
        assert!(close(convert(0x1109_5000 as f64, &m("Frequency BCD32"), &m("Hz")).unwrap(), 110_950_000.));
        assert_eq!(convert(350., &m("kHz"), &m("Frequency ADF BCD32")).unwrap(), 0x0350_0000 as f64);
    }
}
