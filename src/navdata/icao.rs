//! MSFS facility identifiers.
//!
//! MSFS names every facility by an ICAO value: a type letter (`A` airport,
//! `W` waypoint, `V` VHF navaid, `N` NDB, `R` runway), a two letter region,
//! the airport a terminal facility belongs to, and the ident. Scripts see it
//! two ways: the `JS_ICAO` object (`{__Type, type, region, airport, ident}`)
//! and the 12 character "V1" string: type, region (blank for airports),
//! airport padded to 4, ident padded to 5 (msfs-sdk `ICAO.valueToStringV1`).

use super::json::{self, Obj, Value};

/// A short upper-case code (ident, airport, region), zero padded.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct Code(pub [u8; 8]);

impl Code {
    pub const EMPTY: Code = Code([0; 8]);

    /// The code for `s`, or `None` if it is longer than eight bytes.
    pub fn new(s: &str) -> Option<Code> {
        let s = s.trim();
        if s.len() > 8 {
            return None;
        }
        let mut c = [0u8; 8];
        c[..s.len()].copy_from_slice(s.as_bytes());
        Some(Code(c))
    }

    /// Like `new`, but cuts longer text short.
    pub fn lossy(s: &str) -> Code {
        let s = s.trim();
        let mut c = [0u8; 8];
        let n = s.len().min(8);
        c[..n].copy_from_slice(&s.as_bytes()[..n]);
        Code(c)
    }

    pub fn as_str(&self) -> &str {
        let n = self.0.iter().position(|&b| b == 0).unwrap_or(8);
        std::str::from_utf8(&self.0[..n]).unwrap_or("")
    }

    pub fn is_empty(&self) -> bool {
        self.0[0] == 0
    }

    pub fn starts_with(&self, prefix: &Code) -> bool {
        let p = prefix.as_str().as_bytes();
        self.0.starts_with(p)
    }
}

impl std::fmt::Debug for Code {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

/// A two letter region code.
pub fn region(s: &str) -> [u8; 2] {
    let b = s.trim().as_bytes();
    let mut r = [0u8; 2];
    for (i, c) in b.iter().take(2).enumerate() {
        r[i] = *c;
    }
    r
}

pub fn region_str(r: &[u8; 2]) -> &str {
    let n = r.iter().position(|&b| b == 0).unwrap_or(2);
    std::str::from_utf8(&r[..n]).unwrap_or("")
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Default, Debug)]
pub struct Icao {
    /// `A`, `W`, `V`, `N`, `R`, or 0 for the empty ICAO.
    pub kind: u8,
    pub region: [u8; 2],
    pub airport: Code,
    pub ident: Code,
}

impl Icao {
    pub const EMPTY: Icao = Icao { kind: 0, region: [0; 2], airport: Code::EMPTY, ident: Code::EMPTY };

    pub fn new(kind: u8, region: [u8; 2], airport: Code, ident: Code) -> Icao {
        Icao { kind, region, airport, ident }
    }

    pub fn is_empty(&self) -> bool {
        self.kind == 0 && self.ident.is_empty()
    }

    /// The legacy 12 character form. Airports drop their region, as the sim does.
    pub fn v1(&self) -> String {
        let mut s = String::with_capacity(12);
        self.write_v1(&mut s);
        s
    }

    pub fn write_v1(&self, s: &mut String) {
        if self.is_empty() {
            s.push_str("            ");
            return;
        }
        s.push(self.kind as char);
        let region = if self.kind == b'A' { "" } else { region_str(&self.region) };
        s.push_str(&format!("{region:<2}{:<4}{:<5}", self.airport.as_str(), self.ident.as_str()));
    }

    /// Reads a V1 string: type, region, airport, ident by column.
    pub fn from_v1(text: &str) -> Icao {
        let get = |a: usize, b: usize| text.get(a.min(text.len())..b.min(text.len())).unwrap_or("").trim();
        let kind = text.as_bytes().first().copied().filter(|b| *b != b' ').unwrap_or(0);
        Icao {
            kind,
            region: region(get(1, 3)),
            airport: Code::lossy(get(3, 7)),
            ident: Code::lossy(text.get(7.min(text.len())..).unwrap_or("").trim()),
        }
    }

    /// Reads either form: a `JS_ICAO` object or a V1 string.
    pub fn from_value(v: &Value) -> Option<Icao> {
        match v {
            Value::Str(s) => Some(Icao::from_v1(s)),
            Value::Obj(_) => {
                let field = |k: &str| v.get(k).and_then(Value::as_str).unwrap_or("");
                Some(Icao {
                    kind: field("type").bytes().next().unwrap_or(0),
                    region: region(field("region")),
                    airport: Code::lossy(field("airport")),
                    ident: Code::lossy(field("ident")),
                })
            }
            _ => None,
        }
    }

    /// Writes the `JS_ICAO` object.
    pub fn write_struct(&self, out: &mut String) {
        let kind = if self.kind == 0 { String::new() } else { (self.kind as char).to_string() };
        let mut o = Obj::new(out);
        o.str("__Type", "JS_ICAO")
            .str("type", &kind)
            .str("region", region_str(&self.region))
            .str("airport", self.airport.as_str())
            .str("ident", self.ident.as_str());
        o.end();
    }

    /// Writes both forms under the conventional names: `name` for the V1
    /// string and `name` + `Struct` for the object.
    pub fn write_fields(&self, o: &mut Obj<'_>, name: &str) {
        json::string(o.key(name), &self.v1());
        self.write_struct(o.key(&format!("{name}Struct")));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v1_strings_match_the_sdk_layout() {
        let apt = Icao::new(b'A', region("EG"), Code::EMPTY, Code::lossy("EGLL"));
        assert_eq!(apt.v1(), "A      EGLL ");
        let wpt = Icao::new(b'W', region("EG"), Code::lossy("EGLL"), Code::lossy("CF27L"));
        assert_eq!(wpt.v1(), "WEGEGLLCF27L");
        let vor = Icao::new(b'V', region("EG"), Code::EMPTY, Code::lossy("BIG"));
        assert_eq!(vor.v1(), "VEG    BIG  ");
        assert_eq!(Icao::EMPTY.v1(), "            ");
        assert_eq!(Icao::from_v1("VEG    BIG  "), vor);
        // Scripts often skip the padding.
        assert_eq!(Icao::from_v1("A      EGLL").ident.as_str(), "EGLL");
        let mut s = String::new();
        wpt.write_struct(&mut s);
        assert_eq!(s, r#"{"__Type":"JS_ICAO","type":"W","region":"EG","airport":"EGLL","ident":"CF27L"}"#);
        assert_eq!(Icao::from_value(&json::parse(&s).unwrap()), Some(wpt));
    }
}
