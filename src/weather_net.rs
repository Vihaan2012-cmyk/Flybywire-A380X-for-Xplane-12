//! METAR and TAF in the shape FlyByWire's API answers (`{icao, source, metar}`,
//! `{icao, source, taf}`), fetched from the services that API used to proxy:
//! its own `/metar` is blocked for many users and its `/taf` finds nothing.

const RECEIVE_MS: u32 = 15_000;

fn icao_code(icao: &str) -> Option<String> {
    let code = icao.trim().to_ascii_uppercase();
    (code.len() == 4 && code.chars().all(|c| c.is_ascii_alphanumeric())).then_some(code)
}

fn upstream(kind: &str, icao: &str, source: &str) -> Option<String> {
    match (kind, source.to_ascii_lowercase().as_str()) {
        ("metar", "vatsim") => Some(format!("https://metar.vatsim.net/{icao}")),
        ("metar", _) => Some(format!("https://aviationweather.gov/api/data/metar?ids={icao}&format=raw")),
        ("taf", _) => Some(format!("https://aviationweather.gov/api/data/taf?ids={icao}&format=raw")),
        _ => None,
    }
}

pub fn report(kind: &str, icao: &str, raw: &str) -> Option<String> {
    let line = raw.split_whitespace().collect::<Vec<_>>().join(" ");
    let line = match kind {
        "metar" => line.strip_prefix("METAR ").or_else(|| line.strip_prefix("SPECI ")).unwrap_or(&line).to_string(),
        _ => line,
    };
    line.contains(icao).then_some(line)
}

fn source_name(kind: &str, source: &str) -> String {
    match (kind, source.to_ascii_lowercase().as_str()) {
        ("metar", "vatsim") => "VATSIM".into(),
        _ => "NOAA".into(),
    }
}

pub fn answer(kind: &str, icao: &str, source: &str) -> Option<(u16, String)> {
    let kind = kind.to_ascii_lowercase();
    let url = upstream(&kind, "XXXX", source)?;
    let Some(code) = icao_code(icao) else {
        return Some((400, serde_json::json!({ "statusCode": 400, "message": "an ICAO code has four letters" }).to_string()));
    };
    let url = url.replace("XXXX", &code);
    let text = crate::net::request_with("GET", &url, "", RECEIVE_MS).filter(|(status, _)| *status == 200).and_then(|(_, body)| report(&kind, &code, &body));
    Some(match text {
        Some(text) => (200, serde_json::json!({ "icao": code, "source": source_name(&kind, source), kind.as_str(): text }).to_string()),
        None => (
            404,
            serde_json::json!({ "statusCode": 404, "message": format!("{} not available for ICAO: {code}", kind.to_ascii_uppercase()) }).to_string(),
        ),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reports_are_trimmed_to_what_flybywires_api_returned() {
        assert_eq!(
            report("metar", "EGLL", "METAR EGLL 081150Z AUTO 32012KT 9999 NCD 14/02 Q1022\n").as_deref(),
            Some("EGLL 081150Z AUTO 32012KT 9999 NCD 14/02 Q1022")
        );
        assert_eq!(report("metar", "EGLL", "EGLL 081150Z AUTO 32012KT").as_deref(), Some("EGLL 081150Z AUTO 32012KT"));
        assert_eq!(
            report("taf", "EGLL", "TAF EGLL 081102Z 0812/0918 32010KT 9999 FEW045\n  BECMG 0815/0818 23004KT\n").as_deref(),
            Some("TAF EGLL 081102Z 0812/0918 32010KT 9999 FEW045 BECMG 0815/0818 23004KT")
        );
        assert_eq!(report("metar", "ZZZZ", ""), None, "an unknown airport answers nothing");
    }

    #[test]
    fn each_source_goes_where_it_should() {
        assert_eq!(upstream("metar", "EGLL", "vatsim").unwrap(), "https://metar.vatsim.net/EGLL");
        assert!(upstream("metar", "EGLL", "ms").unwrap().starts_with("https://aviationweather.gov/api/data/metar?ids=EGLL"));
        assert!(upstream("taf", "EGLL", "aviationweather").unwrap().starts_with("https://aviationweather.gov/api/data/taf?ids=EGLL"));
        assert_eq!(upstream("atis", "EGLL", "ivao"), None);
        assert_eq!(answer("metar", "EG", "vatsim").map(|(s, _)| s), Some(400));
        assert_eq!(answer("atis", "EGLL", ""), None);
    }
}
