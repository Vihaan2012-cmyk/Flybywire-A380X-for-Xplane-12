//! Just enough JSON for Coherent call arguments and facility objects: a
//! parser for the argument arrays scripts pass, and a writer that builds
//! results straight into a string.

use std::fmt::Write as _;

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Value>),
    Obj(Vec<(String, Value)>),
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Num(n) => Some(*n),
            Value::Bool(b) => Some(*b as i32 as f64),
            Value::Str(s) => s.trim().parse().ok(),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn get(&self, key: &str) -> Option<&Value> {
        match self {
            Value::Obj(fields) => fields.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
}

pub fn parse(text: &str) -> Result<Value, String> {
    let mut p = Parser { s: text.as_bytes(), i: 0 };
    p.ws();
    let v = p.value()?;
    p.ws();
    if p.i != p.s.len() {
        return Err(format!("trailing characters at {}", p.i));
    }
    Ok(v)
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn ws(&mut self) {
        while self.i < self.s.len() && matches!(self.s[self.i], b' ' | b'\t' | b'\n' | b'\r') {
            self.i += 1;
        }
    }

    fn eat(&mut self, word: &str) -> Result<(), String> {
        if self.s[self.i..].starts_with(word.as_bytes()) {
            self.i += word.len();
            Ok(())
        } else {
            Err(format!("expected {word} at {}", self.i))
        }
    }

    fn value(&mut self) -> Result<Value, String> {
        match self.s.get(self.i) {
            None => Err("unexpected end".into()),
            Some(b'{') => {
                self.i += 1;
                let mut fields = Vec::new();
                self.ws();
                if self.s.get(self.i) == Some(&b'}') {
                    self.i += 1;
                    return Ok(Value::Obj(fields));
                }
                loop {
                    self.ws();
                    let key = self.string()?;
                    self.ws();
                    self.eat(":")?;
                    self.ws();
                    fields.push((key, self.value()?));
                    self.ws();
                    match self.s.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b'}') => {
                            self.i += 1;
                            return Ok(Value::Obj(fields));
                        }
                        _ => return Err(format!("expected , or }} at {}", self.i)),
                    }
                }
            }
            Some(b'[') => {
                self.i += 1;
                let mut items = Vec::new();
                self.ws();
                if self.s.get(self.i) == Some(&b']') {
                    self.i += 1;
                    return Ok(Value::Arr(items));
                }
                loop {
                    self.ws();
                    items.push(self.value()?);
                    self.ws();
                    match self.s.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b']') => {
                            self.i += 1;
                            return Ok(Value::Arr(items));
                        }
                        _ => return Err(format!("expected , or ] at {}", self.i)),
                    }
                }
            }
            Some(b'"') => Ok(Value::Str(self.string()?)),
            Some(b't') => self.eat("true").map(|_| Value::Bool(true)),
            Some(b'f') => self.eat("false").map(|_| Value::Bool(false)),
            Some(b'n') => self.eat("null").map(|_| Value::Null),
            Some(_) => {
                let start = self.i;
                while self.i < self.s.len() && matches!(self.s[self.i], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9') {
                    self.i += 1;
                }
                std::str::from_utf8(&self.s[start..self.i])
                    .ok()
                    .and_then(|t| t.parse().ok())
                    .map(Value::Num)
                    .ok_or_else(|| format!("bad value at {start}"))
            }
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.eat("\"")?;
        let mut out = String::new();
        loop {
            let start = self.i;
            while self.i < self.s.len() && self.s[self.i] != b'"' && self.s[self.i] != b'\\' {
                self.i += 1;
            }
            out.push_str(std::str::from_utf8(&self.s[start..self.i]).map_err(|e| e.to_string())?);
            match self.s.get(self.i) {
                Some(b'"') => {
                    self.i += 1;
                    return Ok(out);
                }
                Some(b'\\') => {
                    let c = *self.s.get(self.i + 1).ok_or("bad escape")?;
                    self.i += 2;
                    match c {
                        b'n' => out.push('\n'),
                        b't' => out.push('\t'),
                        b'r' => out.push('\r'),
                        b'b' => out.push('\u{8}'),
                        b'f' => out.push('\u{c}'),
                        b'u' => {
                            let hex = std::str::from_utf8(self.s.get(self.i..self.i + 4).ok_or("bad escape")?)
                                .map_err(|e| e.to_string())?;
                            let code = u32::from_str_radix(hex, 16).map_err(|e| e.to_string())?;
                            self.i += 4;
                            out.push(char::from_u32(code).unwrap_or('\u{fffd}'));
                        }
                        other => out.push(other as char),
                    }
                }
                _ => return Err("unterminated string".into()),
            }
        }
    }
}

/// Appends `s` as a JSON string literal.
pub fn string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

/// Appends a number. JSON has no NaN or infinity; those become 0.
pub fn number(out: &mut String, v: f64) {
    if !v.is_finite() {
        out.push('0');
    } else if v.fract() == 0. && v.abs() < 1e15 {
        let _ = write!(out, "{}", v as i64);
    } else {
        let _ = write!(out, "{v}");
    }
}

/// An object being written: fields are added in order, then `end` closes it.
pub struct Obj<'a> {
    pub out: &'a mut String,
    first: bool,
}

impl<'a> Obj<'a> {
    pub fn new(out: &'a mut String) -> Self {
        out.push('{');
        Obj { out, first: true }
    }

    /// Starts a field; the caller writes its value into `self.out`.
    pub fn key(&mut self, key: &str) -> &mut String {
        if !self.first {
            self.out.push(',');
        }
        self.first = false;
        string(self.out, key);
        self.out.push(':');
        self.out
    }

    pub fn str(&mut self, key: &str, v: &str) -> &mut Self {
        let out = self.key(key);
        string(out, v);
        self
    }

    pub fn num(&mut self, key: &str, v: f64) -> &mut Self {
        let out = self.key(key);
        number(out, v);
        self
    }

    pub fn bool(&mut self, key: &str, v: bool) -> &mut Self {
        self.key(key).push_str(if v { "true" } else { "false" });
        self
    }

    pub fn null(&mut self, key: &str) -> &mut Self {
        self.key(key).push_str("null");
        self
    }

    pub fn end(self) {
        self.out.push('}');
    }
}

/// Writes `items` as an array, each through `f`.
pub fn array<T>(out: &mut String, items: impl IntoIterator<Item = T>, mut f: impl FnMut(&mut String, T)) {
    out.push('[');
    for (i, item) in items.into_iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        f(out, item);
    }
    out.push(']');
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_coherent_argument_arrays() {
        let v = parse(r#"[{"__Type":"JS_ICAO","type":"A","region":"","airport":"","ident":"EGLL"}, 127, "A      EGLL ", [1, 2.5e1], null, true]"#).unwrap();
        let Value::Arr(items) = v else { panic!() };
        assert_eq!(items[0].get("ident").and_then(Value::as_str), Some("EGLL"));
        assert_eq!(items[1].as_f64(), Some(127.));
        assert_eq!(items[2].as_str(), Some("A      EGLL "));
        assert_eq!(items[3], Value::Arr(vec![Value::Num(1.), Value::Num(25.)]));
        assert_eq!(items[4], Value::Null);
        assert_eq!(items[5], Value::Bool(true));
    }

    #[test]
    fn writes_escaped_objects() {
        let mut out = String::new();
        let mut o = Obj::new(&mut out);
        o.str("name", "A \"B\"\\").num("n", 3.).num("f", -0.5).bool("b", false);
        array(o.key("a"), [1., 2.], |out, v| number(out, v));
        o.end();
        assert_eq!(out, r#"{"name":"A \"B\"\\","n":3,"f":-0.5,"b":false,"a":[1,2]}"#);
        assert_eq!(parse(&out).unwrap().get("f").and_then(Value::as_f64), Some(-0.5));
    }
}
