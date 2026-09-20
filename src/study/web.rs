//! JSON for the XPHFBW app's Study tab (docs/briefs/xphfbw-app.md): the same
//! page, group, topology, failures and breaker data the XPLM Study windows
//! draw from, generated from the exact same Rust definitions (`pages.rs`,
//! `depth.rs`, `elec.rs`, `hyd.rs`, `engine.rs`, `failures.rs`,
//! `services.rs`) so the web page and the in-sim windows can never drift
//! apart. `panel.rs` serves these at `/study/pages`, `/study/failures`,
//! `/study/breakers`, and applies `/study/action` posts through
//! [`apply_action`] onto the same request queues the XPLM windows use.
//!
//! Nothing here draws anything or touches X-Plane's own API; it only reads
//! the snapshot and the study modules' declarative data, so it is safe to
//! call from the panel's own thread (panel.rs's own doc comment explains
//! why that is safe).

use serde_json::{json, Value};

use super::canvas::{self, Field, Group};
use super::{depth, elec, engine, failures, hyd, pages, services, PageKind, ITEMS};

// ---------------------------------------------------------------------
// Shared JSON for a Field/Group, the box-page data type every field-list
// page (and the physics boxes `depth.rs` adds) is built from.

fn field_json(f: &Field) -> Value {
    let (kind, decimals) = match f.show {
        canvas::Show::Num(d) => ("num", d),
        canvas::Show::Lamp => ("lamp", 0),
        canvas::Show::Arinc(d) => ("arinc", d),
    };
    json!({ "label": f.label, "name": f.name, "unit": f.unit, "kind": kind, "decimals": decimals })
}

fn group_json(g: &Group) -> Value {
    json!({
        "title": g.title,
        "theme": canvas::theme_name(&g.theme),
        "hideMissing": g.hide_missing,
        "fields": g.fields.iter().map(field_json).collect::<Vec<_>>(),
    })
}

fn groups_json(groups: &[Group]) -> Value {
    Value::Array(groups.iter().map(group_json).collect())
}

// ---------------------------------------------------------------------
// Topology JSON (elec.rs/hyd.rs): nodes and links on the design sheet, with
// each link's liveness as a small boolean-gate tree the web page evaluates
// against its own `/vars` poll.

fn topo_field_json(f: &elec::TopoField) -> Value {
    json!({ "label": f.label, "name": f.name, "unit": f.unit, "decimals": f.decimals, "kind": f.kind, "flagText": f.flag_text })
}

fn topo_node_json(n: &elec::TopoNode) -> Value {
    json!({
        "id": n.id, "title": n.title, "x": n.x, "y": n.y, "w": n.w, "kind": n.kind,
        "fields": n.fields.iter().map(topo_field_json).collect::<Vec<_>>(),
    })
}

fn gate_json(g: &elec::Gate) -> Value {
    match g {
        elec::Gate::On(name) => json!({ "op": "on", "name": name }),
        elec::Gate::Off(name) => json!({ "op": "off", "name": name }),
        elec::Gate::Gt(name, v) => json!({ "op": "gt", "name": name, "value": v }),
        elec::Gate::AbsGt(name, v) => json!({ "op": "absGt", "name": name, "value": v }),
        elec::Gate::All(gates) => json!({ "op": "all", "gates": gates.iter().map(gate_json).collect::<Vec<_>>() }),
        elec::Gate::Any(gates) => json!({ "op": "any", "gates": gates.iter().map(gate_json).collect::<Vec<_>>() }),
    }
}

fn topo_link_json(l: &elec::TopoLink) -> Value {
    json!({ "points": l.points, "kind": l.kind, "gate": gate_json(&l.gate) })
}

fn topology_json(t: &elec::Topology) -> Value {
    json!({
        "designW": t.design_w, "designH": t.design_h,
        "nodes": t.nodes.iter().map(topo_node_json).collect::<Vec<_>>(),
        "links": t.links.iter().map(topo_link_json).collect::<Vec<_>>(),
    })
}

// ---------------------------------------------------------------------
// Per-page JSON.

/// The Study tree's own grouping ([`super::build_menu`]), so the web page's
/// left-hand list can section itself the same way the aircraft menu does.
fn menu_group(kind: PageKind) -> &'static str {
    match kind {
        PageKind::Engine(_) | PageKind::Apu | PageKind::Bleed | PageKind::Fuel => "Engines",
        PageKind::AirConditioning | PageKind::Pressurisation => "Environmental",
        PageKind::Hydraulics | PageKind::FlightControls => "Hydraulics",
        PageKind::GearBrakes => "Landing Gear",
        PageKind::Fire => "Fire",
        _ => "",
    }
}

fn engine_station_json(s: &engine::Station) -> Value {
    json!({
        "title": s.title, "x": s.x, "y": s.y, "w": s.w, "theme": canvas::theme_name(&s.theme),
        "fields": s.fields.iter().map(field_json).collect::<Vec<_>>(),
        "leader": s.to.map(|(tx, ty, label)| json!([tx, ty, label])),
    })
}

/// The engine cutaway's shapes ([`engine::cutaway`]), colours as CSS.
fn cutaway_json() -> Value {
    let css = |c: [f32; 4]| format!("rgb({},{},{})", (c[0] * 255.) as u8, (c[1] * 255.) as u8, (c[2] * 255.) as u8);
    Value::Array(
        engine::cutaway()
            .into_iter()
            .map(|shape| match shape {
                engine::Shape::Poly(points, c) => json!({ "type": "poly", "points": points, "fill": css(c) }),
                engine::Shape::Rect(x, y, w, h, c) => json!({ "type": "rect", "x": x, "y": y, "w": w, "h": h, "fill": css(c) }),
                engine::Shape::Line(x1, y1, x2, y2, c, width) => json!({ "type": "line", "x1": x1, "y1": y1, "x2": x2, "y2": y2, "stroke": css(c), "width": width }),
                engine::Shape::Arrow(x, y, len, half, c) => json!({ "type": "arrow", "x": x, "y": y, "len": len, "half": half, "fill": css(c) }),
                engine::Shape::Text(x, y, text) => json!({ "type": "text", "x": x, "y": y, "text": text }),
            })
            .collect(),
    )
}

fn engine_summary_json(entry: &(&'static str, canvas::Theme, Vec<Field>)) -> Value {
    let (title, theme, fields) = entry;
    json!({ "title": title, "theme": canvas::theme_name(theme), "fields": fields.iter().map(field_json).collect::<Vec<_>>() })
}

fn flight_controls_json() -> Value {
    let columns = pages::flight_controls_columns();
    Value::Array(
        columns
            .iter()
            .map(|sections| {
                Value::Array(
                    sections
                        .iter()
                        .map(|(heading, gauges)| {
                            json!({
                                "heading": heading,
                                "gauges": gauges.iter().map(|g| json!({
                                    "label": g.label, "name": g.name, "low": g.low, "high": g.high, "unit": g.unit,
                                })).collect::<Vec<_>>(),
                            })
                        })
                        .collect::<Vec<_>>(),
                )
            })
            .collect(),
    )
}

fn fuel_json() -> Value {
    let tanks: Vec<Value> = pages::tanks()
        .iter()
        .map(|&(n, label, x, w, cap)| json!({ "number": n, "label": label, "x": x, "w": w, "capacityGal": cap, "name": format!("A32NX_FUEL_TANK_QUANTITY_{n}") }))
        .collect();
    let stations: Vec<Value> = pages::fuel_extra_stations()
        .into_iter()
        .map(|(x, y, w, title, theme, fields)| {
            json!({ "x": x, "y": y, "w": w, "title": title, "theme": canvas::theme_name(&theme), "fields": fields.iter().map(field_json).collect::<Vec<_>>() })
        })
        .collect();
    json!({ "tanks": tanks, "stations": stations })
}

fn buttons_json(buttons: &[(&'static str, &'static str)]) -> Value {
    Value::Array(buttons.iter().map(|&(label, command)| json!({ "label": label, "command": command })).collect())
}

fn page_json(index: usize, title: &str, kind: PageKind) -> Value {
    let mut obj = json!({
        "index": index,
        "title": title,
        "menuGroup": menu_group(kind),
        "hasPhysics": depth::has_physics(kind),
    });
    let map = obj.as_object_mut().expect("object literal");
    match kind {
        PageKind::Apu | PageKind::Bleed | PageKind::AirConditioning | PageKind::Pressurisation | PageKind::GearBrakes | PageKind::AirData | PageKind::Fire => {
            let groups = pages::groups(kind);
            map.insert("kind".into(), json!("topology"));
            map.insert("topology".into(), topology_json(&elec::auto_topology(&groups)));
            map.insert("groups".into(), groups_json(&groups));
        }
        PageKind::Electrical => {
            map.insert("kind".into(), json!("topology"));
            map.insert("topology".into(), topology_json(&elec::topology()));
            map.insert("groups".into(), groups_json(&depth::extra(kind)));
        }
        PageKind::Hydraulics => {
            map.insert("kind".into(), json!("topology"));
            map.insert("topology".into(), topology_json(&hyd::topology()));
            map.insert("groups".into(), groups_json(&depth::extra(kind)));
        }
        PageKind::Fuel => {
            map.insert("kind".into(), json!("fuel"));
            let fuel = fuel_json();
            map.insert("tanks".into(), fuel["tanks"].clone());
            map.insert("stations".into(), fuel["stations"].clone());
            map.insert("topology".into(), topology_json(&pages::fuel_topology()));
            map.insert("groups".into(), groups_json(&depth::extra(kind)));
        }
        PageKind::FlightControls => {
            map.insert("kind".into(), json!("flightControls"));
            map.insert("columns".into(), flight_controls_json());
        }
        PageKind::Engine(n) => {
            map.insert("kind".into(), json!("engine"));
            map.insert("engine".into(), json!(n));
            map.insert("stations".into(), Value::Array(engine::stations(n).iter().map(engine_station_json).collect()));
            map.insert("summaries".into(), Value::Array(engine::summaries(n).iter().map(engine_summary_json).collect()));
            map.insert("topology".into(), topology_json(&engine::topology(n)));
            map.insert("cutaway".into(), cutaway_json());
            map.insert("groups".into(), groups_json(&depth::extra(kind)));
        }
        PageKind::Radios => {
            let groups = pages::radios_groups();
            map.insert("kind".into(), json!("topology"));
            map.insert("topology".into(), topology_json(&elec::auto_topology(&groups)));
            map.insert("groups".into(), groups_json(&groups));
        }
        PageKind::Failures => {
            map.insert("kind".into(), json!("failures"));
        }
        PageKind::Breakers => {
            map.insert("kind".into(), json!("breakers"));
        }
        PageKind::GroundServices => {
            map.insert("kind".into(), json!("groundServices"));
            map.insert("buttons".into(), buttons_json(&services::ground_buttons()));
            map.insert("tug".into(), buttons_json(&services::tug_buttons()));
            map.insert("groups".into(), groups_json(&services::ground_groups()));
        }
        PageKind::All => {
            map.insert("kind".into(), json!("all"));
        }
    }
    obj
}

/// `GET /study/pages`: the page list and each page's structure, generated
/// fresh from the same Rust the XPLM windows draw from (no caching — these
/// are cheap to build and the plugin can gain fields between requests).
pub(crate) fn pages_json() -> String {
    let pages: Vec<Value> = ITEMS.iter().enumerate().map(|(i, (title, kind))| page_json(i, title, *kind)).collect();
    json!({ "designW": canvas::DESIGN_W, "designH": canvas::DESIGN_H, "pages": pages }).to_string()
}

/// `GET /study/failures`: every failure id the systems and computers know
/// (the same catalogue [`failures::draw`] chapters), with whether it is
/// currently armed.
pub(crate) fn failures_json() -> String {
    let active = crate::failures::active_ids();
    // Every catalogued failure, the extra catalogue's (component, exotic
    // and hook-driven) included: all arm through the same active set.
    let ids = crate::failures::all_ids();
    let list: Vec<Value> = ids
        .iter()
        .map(|&id| {
            let remaining = crate::mel::deferred_state(id);
            // "state": inactive (never armed), active (armed, not under the
            // MEL), deferred (armed and dispatch-legal under the MEL, with
            // hoursRemaining), expired (its MEL interval ran out -- still
            // active, no longer dispatch-legal). Additive: "active" (bool)
            // above is kept for any existing client.
            let state = match (active.contains(&id), remaining) {
                (true, Some(h)) if h > 0.0 => "deferred",
                (true, Some(_)) => "expired",
                (true, None) => "active",
                (false, _) => "inactive",
            };
            json!({
                "id": id,
                "name": crate::failures::any_failure_name(id),
                "ata": failures::ata_of(id),
                "chapter": failures::chapter(failures::ata_of(id)),
                "active": active.contains(&id),
                "cause": crate::failures::cause_description(id),
                "affectedComponents": crate::failures::affected_components(id),
                "triggerCondition": crate::failures::trigger_condition(id),
                "state": state,
                "melHoursRemaining": remaining,
                // Continuous physical fraction in 0.0..=1.0
                // (docs/physics/failures.md's continuous-magnitude
                // contract): 0.0 for an inactive id, else whatever
                // `failures::set_magnitude` last set (1.0 -- "fully
                // lost" -- for a plain `toggleFailure`/legacy activation
                // that never called it). Consumers read this as the
                // physical perturbation fraction itself, never a
                // severity bucket.
                "magnitude": crate::failures::magnitude(id),
            })
        })
        .collect();
    json!({ "failures": list }).to_string()
}

fn mel_sub_json(item: &crate::mel_catalog::Item, sub: &crate::mel_catalog::SubItem) -> Value {
    json!({
        "id": sub.id,
        "ata": item.ata,
        "title": item.title,
        "category": sub.category,
        "intervalDays": sub.repair_interval_days,
        "installed": sub.installed,
        "required": sub.required,
        "placard": sub.placard,
        "conditions": sub.conditions,
        "opsProcedureRequired": sub.ops_procedure_required,
        "maintenanceProcedureRequired": sub.maintenance_procedure_required,
        "opsProcedure": sub.ops_procedure,
        "amm": sub.amm,
    })
}

/// `GET /study/maintenance`: the MEL & Maintenance tab. Active defects with
/// their deferral (and the MEL item it was made under) or, if not deferred,
/// the MEL items that may cover them; whether the aircraft may be
/// dispatched; component wear; the technical log.
///
/// Dispatch here follows the MEL's own rule: every active defect must be
/// deferred with time remaining. An active defect not on the MEL, or one
/// whose interval has run out, blocks dispatch.
pub(crate) fn maintenance_json() -> String {
    let (deferred, hours, tech_log) = crate::mel::latest();
    let active = crate::failures::active_ids();
    let mut blocking = Vec::new();
    let defects: Vec<Value> = active
        .iter()
        .map(|&id| {
            let name = crate::failures::any_failure_name(id);
            let deferral = deferred.iter().find(|d| d.id == id);
            let remaining = deferral.map(|d| d.expires_at_hours - hours);
            match remaining {
                Some(h) if h > 0.0 => {}
                Some(_) => blocking.push(format!("{name}: MEL interval expired")),
                None => blocking.push(format!("{name}: not deferred")),
            }
            let mel_item = deferral
                .and_then(|d| d.mel_ref.as_deref())
                .and_then(crate::mel_catalog::sub_item)
                .map(|(it, s)| mel_sub_json(it, s));
            let candidates: Vec<Value> = if deferral.is_none() {
                crate::mel_catalog::candidates(id, 12).into_iter().map(|(it, s)| mel_sub_json(it, s)).collect()
            } else {
                Vec::new()
            };
            json!({
                "id": id,
                "name": name,
                "ata": failures::ata_of(id),
                "magnitude": crate::failures::magnitude(id),
                "cause": crate::failures::cause_description(id),
                "genericCategory": crate::failures::mel_item(id).map(|c| c.label()),
                "deferral": deferral.map(|d| json!({
                    "melRef": d.mel_ref,
                    "deferredAtHours": d.deferred_at_hours,
                    "hoursRemaining": d.expires_at_hours - hours,
                    "melItem": mel_item,
                })),
                "candidates": candidates,
            })
        })
        .collect();
    let wear = crate::wear::snapshot();
    let components: Vec<Value> = wear
        .ids()
        .map(|c| {
            let w = wear.get(c);
            json!({
                "id": c,
                "hotHours": w.hot_hours,
                "cycles": w.cycles,
                "thermalStress": w.thermal_stress_integral,
                "degradation": w.degradation_fraction,
            })
        })
        .collect();
    let log: Vec<Value> = tech_log
        .iter()
        .rev()
        .map(|e| json!({ "airframeHours": e.airframe_hours, "unixTime": e.unix_time, "text": e.text }))
        .collect();
    json!({
        "melLoaded": crate::mel_catalog::catalog().is_some(),
        "melPath": crate::mel_catalog::path().map(|p| p.display().to_string()),
        "airframeHours": hours,
        "dispatch": { "legal": blocking.is_empty(), "blocking": blocking },
        // Every deferral: which failure (unit), under which MEL sub-item.
        "deferrals": deferred.iter().map(|d| json!({ "id": d.id, "melRef": d.mel_ref })).collect::<Vec<_>>(),
        // The operator MEL's ATA chapters, for the chapter listing.
        "melChapters": crate::mel_catalog::catalog().map(|items| {
            let mut chapters: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
            for it in items.iter().filter(|it| !crate::mel_catalog::failures_for(&it.ata).is_empty()) {
                let c: String = it.ata.chars().take(2).collect();
                *chapters.entry(c).or_insert(0) += it.subitems.len();
            }
            chapters
                .into_iter()
                .map(|(c, n)| json!({ "ata": c, "name": failures::chapter(c.parse().unwrap_or(0)), "count": n }))
                .collect::<Vec<_>>()
        }).unwrap_or_default(),
        "defects": defects,
        "components": components,
        "techLog": log,
    })
    .to_string()
}

/// `GET /study/components`: every damageable component's physical
/// parameters, their current (combined) values and any direct setting.
pub(crate) fn components_json() -> String {
    let list: Vec<Value> = crate::components::list()
        .into_iter()
        .map(|c| {
            json!({
                "component": c.component,
                "param": c.spec.name,
                "unit": c.spec.unit,
                "healthy": c.spec.healthy,
                "min": c.spec.min,
                "max": c.spec.max,
                "combine": format!("{:?}", c.spec.combine),
                "description": c.spec.description,
                "value": c.value,
                "direct": c.direct,
                "ratePerHour": c.rate_per_hour,
            })
        })
        .collect();
    json!({ "components": list }).to_string()
}

/// `GET /study/mel?q=...`: free-text search of the operator MEL.
pub(crate) fn mel_search_json(query: &str) -> String {
    let q = query
        .split('&')
        .find_map(|kv| kv.strip_prefix("q="))
        .map(|v| v.replace('+', " ").replace("%20", " "))
        .unwrap_or_default();
    // `ata=NN`: every item of that ATA chapter, for the chapter listing.
    let ata = query.split('&').find_map(|kv| kv.strip_prefix("ata=")).unwrap_or("");
    // Only MEL items the simulation models (mel_catalog::MEL_FAILURES),
    // each with its units: the failure every INOP button arms.
    let modelled = |(it, _): &(&crate::mel_catalog::Item, &crate::mel_catalog::SubItem)| !crate::mel_catalog::failures_for(&it.ata).is_empty();
    let with_failure = |(it, s): (&crate::mel_catalog::Item, &crate::mel_catalog::SubItem)| {
        let mut v = mel_sub_json(it, s);
        v["units"] = crate::mel_catalog::failures_for(&it.ata)
            .iter()
            .map(|&id| json!({ "failureId": id, "name": crate::failures::any_failure_name(id) }))
            .collect::<Vec<_>>()
            .into();
        v
    };
    let found: Vec<Value> = crate::mel_catalog::catalog()
        .map(|items| {
            if ata.is_empty() {
                crate::mel_catalog::search_in(items, &q, usize::MAX).into_iter().filter(modelled).take(60).map(with_failure).collect()
            } else {
                items
                    .iter()
                    .filter(|it| it.ata.starts_with(ata))
                    .flat_map(|it| it.subitems.iter().map(move |s| (it, s)))
                    .filter(modelled)
                    .map(with_failure)
                    .collect()
            }
        })
        .unwrap_or_default();
    json!({ "items": found }).to_string()
}

/// `GET /study/breakers`: `breakers.rs`'s catalogue, `"source":"catalogue"`,
/// keyed by its own string `id` -- docs/analysis/cockpit-study-cbs.md
/// CB-001/CB-002/STUDY-001. Every real `systems.cfg` circuit (fuel pumps/
/// valves, lights, wipers) is now *absorbed* into the catalogue as its own
/// entry rather than listed separately (docs/physics/breakers.md
/// "systems.cfg audit"); a circuit with no consumer anywhere is dropped
/// entirely instead of listed as a breaker that does nothing.
/// `"systemsCfgAbsorbed": true` at the top level flags this shape to a
/// client that cached the older split-source response. Every entry carries
/// `id`/`name`/`ata`/`chapter`/`bus`/`ratingA`/`currentA`/`closed`/`trip`/
/// `consumers`/`basis`/`gates` (the last, new, is `"pluginVar"`|`"circuit"`|
/// `"failurePower"`|`"failureSoft"`|`"none"` -- see `BreakerDef::gate_kind`).
pub(crate) fn breakers_json() -> String {
    let Ok(snap) = crate::snapshot().lock() else {
        return json!({ "breakers": [], "systemsCfgAbsorbed": true }).to_string();
    };
    let read = |name: &str| snap.find(name).map(|i| snap.values[i]);
    // Every systems.cfg circuit that really gates something (fuel pumps/
    // valves, lights, the two wipers) is absorbed into the catalogue below
    // with a real A380 name/ATA chapter/rating (`breakers::
    // absorbed_systems_cfg`); every other circuit gates nothing anywhere in
    // the plugin or FlyByWire and is not listed at all (docs/physics/
    // breakers.md "systems.cfg audit" -- the user's "either make them work
    // or remove them"). `"systemsCfgAbsorbed": true` tells a client this
    // response no longer carries a separate `"source":"systemsCfg"` array;
    // every entry below is `"source":"catalogue"`, including the absorbed
    // ones.
    let list: Vec<Value> = crate::breakers::catalog()
        .iter()
        .map(|def| {
            let closed = read(&format!("CIRCUIT BREAKER CLOSED:{}", breaker_number(def))) != Some(0.);
            let current = read(&format!("CIRCUIT CURRENT:{}", breaker_number(def)));
            let trip = match read(&format!("CIRCUIT TRIP CAUSE:{}", breaker_number(def))) {
                Some(v) if v == 1. => "thermal",
                Some(v) if v == 2. => "magnetic",
                _ => "none",
            };
            json!({
                "source": "catalogue",
                "id": def.id,
                "name": def.name,
                "ata": def.ata,
                "chapter": failures::chapter(def.ata as u64),
                "bus": def.bus_label(),
                "ratingA": def.rating_a,
                "currentA": current,
                "closed": closed,
                "trip": trip,
                "consumers": def.consumers,
                "basis": def.basis,
                "gates": def.gate_kind(),
            })
        })
        .collect();
    json!({ "breakers": list, "systemsCfgAbsorbed": true }).to_string()
}

/// This entry's live `CIRCUIT BREAKER CLOSED/CURRENT/TRIP CAUSE:n` number:
/// its panel node's existing number, or its synthetic `breakers::EXTRA_BASE`
/// one. Mirrors `breakers::number_for`, which is private to that module
/// (constructed once at startup, not reachable from here); this is the same
/// lookup done the read-only way the Study panel's own thread needs.
fn breaker_number(def: &crate::breakers::BreakerDef) -> usize {
    if let Some(node) = def.panel_node {
        if let Some(n) = crate::circuits::panel_cb_number(node) {
            return n;
        }
    }
    crate::breakers::catalog().iter().filter(|d| d.panel_node.is_none()).position(|d| d.id == def.id).map_or(crate::breakers::EXTRA_BASE, |i| crate::breakers::EXTRA_BASE + i)
}

/// `POST /study/action`: apply one Study window action, the same way a
/// click in the XPLM windows does — onto the plugin's own request queues
/// (`failures::set_active`/`toggle`, `circuits::request_toggle`,
/// `xp::command_once`, `oxygen::request_service`), drained on the next
/// flight loop tick, so there is no race with the systems thread.
///
/// `{"kind":"toggleFailure","id":21000}`,
/// `{"kind":"setFailureMagnitude","id":21000,"magnitude":0.4}` (continuous
/// physical fraction, `failures::set_magnitude` -- docs/physics/
/// failures.md), `{"kind":"toggleBreaker","id":12}`,
/// `{"kind":"deferFailure","id":21000}`/`{"kind":"repairFailure","id":21000}`
/// (`mel.rs`'s deferral/repair, by failure id -- `Mel::apply_requests` on
/// the next tick),
/// `{"kind":"pullBreaker","id":"tr-1"}`/`{"kind":"resetBreaker","id":"tr-1"}`
/// (`breakers.rs`'s catalogue, by its own string id -- `apply_requests` on
/// the next tick), `{"kind":"resetAllBreakers"}`,
/// `{"kind":"command","command":"fbw/efb/ground/gpu"}`,
/// `{"kind":"serviceOxygen"}`. There is no `"kind":"physics"`: the
/// PHYSICS/SCHEMATIC toggle is purely which fields a page shows (already in
/// `/study/pages`' `groups` vs the page's own schematic/topology), the same
/// way the XPLM window's `Action::TogglePhysics` only flips local window
/// state and never touches a queue.
static QUEUED_WRITES: std::sync::Mutex<Vec<(&'static str, f64)>> = std::sync::Mutex::new(Vec::new());

fn queue_write(name: &'static str, value: f64) {
    QUEUED_WRITES.lock().unwrap_or_else(std::sync::PoisonError::into_inner).push((name, value));
}

/// Aircraft variable writes the panel asked for (names without the
/// aircraft prefix), for the plugin's tick to apply on its own thread.
pub(crate) fn take_writes() -> Vec<(&'static str, f64)> {
    std::mem::take(&mut *QUEUED_WRITES.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
}

pub(crate) fn apply_action(body: &str) -> Result<(), String> {
    let v: Value = serde_json::from_str(body).map_err(|e| format!("bad JSON: {e}"))?;
    let kind = v.get("kind").and_then(Value::as_str).ok_or("missing \"kind\"")?;
    match kind {
        "toggleFailure" => {
            let id = v.get("id").and_then(Value::as_u64).ok_or("missing \"id\"")?;
            crate::failures::toggle(id);
        }
        // Continuous physical fraction in 0.0..=1.0 (docs/physics/
        // failures.md): a partial failure, not a severity tier. `0.0`
        // clears the failure the same as `toggleFailure` off; anything
        // else arms it at that fraction (`failures::set_magnitude`).
        "setFailureMagnitude" => {
            let id = v.get("id").and_then(Value::as_u64).ok_or("missing \"id\"")?;
            let magnitude = v.get("magnitude").and_then(Value::as_f64).ok_or("missing \"magnitude\"")?;
            crate::failures::set_magnitude(id, magnitude);
        }
        // MEL: defer a currently-active, deferrable failure under its MMEL
        // category (queued; `mel::Mel::apply_requests` on the plugin's own
        // thread validates and applies it next tick, logging either
        // outcome). "repairFailure" is the Ground Services maintenance
        // action clearing a failure and any deferral on it -- also used to
        // repair a random-failure or damage.rs-armed id, since `mel::
        // Mel::repair` clears the failure regardless of how it was armed.
        "deferFailure" => {
            let id = v.get("id").and_then(Value::as_u64).ok_or("missing \"id\"")?;
            // Optional operator MEL sub-item ("49-10-01A"), mel_catalog.rs.
            let mel_ref = v.get("melRef").and_then(Value::as_str).map(str::to_owned);
            if let Some(r) = &mel_ref {
                if crate::mel_catalog::catalog().is_some() && crate::mel_catalog::sub_item(r).is_none() {
                    return Err(format!("no MEL item {r}"));
                }
            }
            crate::mel::request_defer(id, mel_ref);
        }
        // Maintenance: replace a worn component (wear.rs id), its wear back
        // to new, logged in the tech log.
        // Any damageable component's physical parameter (components.rs),
        // optionally worsening at `ratePerHour` in the parameter's own unit.
        "setComponentParam" => {
            let component = v.get("component").and_then(Value::as_str).ok_or("missing \"component\"")?;
            let param = v.get("param").and_then(Value::as_str).ok_or("missing \"param\"")?;
            let value = v.get("value").and_then(Value::as_f64).ok_or("missing \"value\"")?;
            let rate = v.get("ratePerHour").and_then(Value::as_f64).unwrap_or(0.0);
            crate::components::set_direct(component, param, value, rate)?;
        }
        // Placard one unit of an operator MEL item INOP (mel.rs `placard`):
        // its catalogued failure armed and deferred under the item.
        "placardMelItem" => {
            let r = v.get("melRef").and_then(Value::as_str).ok_or("missing \"melRef\"")?;
            let failure = v.get("failureId").and_then(Value::as_u64).ok_or("missing \"failureId\"")?;
            if crate::mel_catalog::sub_item(r).is_none() {
                return Err(format!("no MEL item {r}"));
            }
            if !crate::mel_catalog::failures_for(r).contains(&failure) {
                return Err(format!("MEL {r} does not cover failure {failure}"));
            }
            crate::mel::request_placard(r.to_owned(), failure);
        }
        "replaceComponent" => {
            let component = v.get("component").and_then(Value::as_str).ok_or("missing \"component\"")?;
            if !crate::wear::snapshot().ids().any(|c| c == component) {
                return Err(format!("no worn component {component}"));
            }
            crate::mel::request_replace(component.to_owned());
        }
        "repairFailure" => {
            let id = v.get("id").and_then(Value::as_u64).ok_or("missing \"id\"")?;
            crate::mel::request_repair(id);
        }
        "toggleBreaker" => {
            let number = v.get("id").and_then(Value::as_u64).ok_or("missing \"id\"")?;
            crate::circuits::request_toggle(number as usize);
        }
        // `breakers.rs`'s wider catalogue, keyed by its own string `id`
        // (unlike "toggleBreaker"'s numeric `systems.cfg` id above) --
        // docs/analysis/cockpit-study-cbs.md STUDY-001's "pull and reset"
        // UI actions, queued the same request-then-apply-on-tick way as
        // every other action here (`breakers::Breakers::apply_requests`
        // drains these on the plugin's own thread).
        "pullBreaker" => {
            let id = v.get("id").and_then(Value::as_str).ok_or("missing \"id\"")?;
            if !crate::breakers::catalog().iter().any(|d| d.id == id) {
                return Err(format!("no catalogue breaker {id}"));
            }
            crate::breakers::request_pull(id.to_owned());
        }
        "resetBreaker" => {
            let id = v.get("id").and_then(Value::as_str).ok_or("missing \"id\"")?;
            if !crate::breakers::catalog().iter().any(|d| d.id == id) {
                return Err(format!("no catalogue breaker {id}"));
            }
            crate::breakers::request_reset(id.to_owned());
        }
        "resetAllBreakers" => crate::breakers::request_reset_all(),
        "command" => {
            let command = v.get("command").and_then(Value::as_str).ok_or("missing \"command\"")?;
            crate::xp::command_once(command);
        }
        "serviceOxygen" => crate::oxygen::request_service(),
        // FlyByWire's aircraft presets (extra_backend/aircraft_presets.rs):
        // 1 cold & dark .. 5 ready for takeoff, 0 cancels.
        "loadAircraftPreset" => {
            let id = v.get("id").and_then(Value::as_u64).ok_or("missing \"id\"")?;
            if id > 5 {
                return Err(format!("no aircraft preset {id}"));
            }
            queue_write("AIRCRAFT_PRESET_LOAD", id as f64);
        }
        "setPresetExpedite" => {
            let on = v.get("on").and_then(Value::as_bool).ok_or("missing \"on\"")?;
            queue_write("AIRCRAFT_PRESET_LOAD_EXPEDITE", if on { 1. } else { 0. });
        }
        other => return Err(format!("unknown action kind: {other}")),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Writes `/study/pages` to `STUDY_PAGES_OUT`, for previewing the web
    /// pages without X-Plane or the app.
    #[test]
    #[ignore]
    fn dump_pages() {
        if let Ok(path) = std::env::var("STUDY_PAGES_OUT") {
            std::fs::write(path, pages_json()).unwrap();
        }
        if let Ok(path) = std::env::var("STUDY_FAILURES_OUT") {
            std::fs::write(path, failures_json()).unwrap();
        }
        if let Ok(path) = std::env::var("STUDY_BREAKERS_OUT") {
            std::fs::write(path, breakers_json()).unwrap();
        }
    }

    #[test]
    fn every_menu_item_becomes_one_json_page() {
        let v: Value = serde_json::from_str(&pages_json()).unwrap();
        let pages = v["pages"].as_array().unwrap();
        assert_eq!(pages.len(), ITEMS.len());
        for (i, (title, _)) in ITEMS.iter().enumerate() {
            assert_eq!(pages[i]["title"], *title);
            assert_eq!(pages[i]["index"], i);
        }
    }

    #[test]
    fn box_pages_carry_their_fields() {
        let v: Value = serde_json::from_str(&pages_json()).unwrap();
        let pages = v["pages"].as_array().unwrap();
        let apu = &pages[super::super::item_of(PageKind::Apu)];
        assert_eq!(apu["kind"], "topology");
        let groups = apu["groups"].as_array().unwrap();
        assert!(!groups.is_empty());
        let rotor = groups.iter().find(|g| g["title"] == "Rotor").expect("Rotor box");
        let fields = rotor["fields"].as_array().unwrap();
        assert!(fields.iter().any(|f| f["name"] == "A32NX_APU_N"));
    }

    #[test]
    fn electrical_and_hydraulics_carry_a_topology() {
        let v: Value = serde_json::from_str(&pages_json()).unwrap();
        let pages = v["pages"].as_array().unwrap();
        for kind in [PageKind::Electrical, PageKind::Hydraulics] {
            let page = &pages[super::super::item_of(kind)];
            assert_eq!(page["kind"], "topology");
            let nodes = page["topology"]["nodes"].as_array().unwrap();
            let links = page["topology"]["links"].as_array().unwrap();
            assert!(!nodes.is_empty() && !links.is_empty(), "{kind:?} has an empty topology");
            // Every link's gate names a real node field or a plain variable;
            // at minimum it must always carry an "op".
            for link in links {
                assert!(link["gate"]["op"].is_string());
            }
        }
    }

    #[test]
    fn engine_pages_carry_stations_and_summaries() {
        let v: Value = serde_json::from_str(&pages_json()).unwrap();
        let pages = v["pages"].as_array().unwrap();
        let e2 = &pages[super::super::item_of(PageKind::Engine(2))];
        assert_eq!(e2["kind"], "engine");
        assert_eq!(e2["engine"], 2);
        let stations = e2["stations"].as_array().unwrap();
        assert!(stations.iter().any(|s| s["title"] == "2: Fan"));
        let summaries = e2["summaries"].as_array().unwrap();
        assert!(summaries.iter().any(|s| s["title"] == "LP Rotor"));
    }

    #[test]
    fn fuel_page_carries_all_eleven_tanks() {
        let v: Value = serde_json::from_str(&pages_json()).unwrap();
        let pages = v["pages"].as_array().unwrap();
        let fuel = &pages[super::super::item_of(PageKind::Fuel)];
        assert_eq!(fuel["kind"], "fuel");
        assert_eq!(fuel["tanks"].as_array().unwrap().len(), 11);
    }

    #[test]
    fn failures_and_breakers_pages_defer_to_their_own_endpoint() {
        let v: Value = serde_json::from_str(&pages_json()).unwrap();
        let pages = v["pages"].as_array().unwrap();
        assert_eq!(pages[super::super::item_of(PageKind::Failures)]["kind"], "failures");
        assert_eq!(pages[super::super::item_of(PageKind::Breakers)]["kind"], "breakers");
    }

    #[test]
    fn failures_json_lists_every_registered_id_once() {
        let v: Value = serde_json::from_str(&failures_json()).unwrap();
        let list = v["failures"].as_array().unwrap();
        // FlyByWire's, the computers' and the extra catalogue's.
        let expect = crate::failures::all_ids();
        assert_eq!(list.len(), expect.len());
        for entry in list {
            assert!(entry["name"].as_str().is_some_and(|s| !s.is_empty() && !s.starts_with("failure ")), "unnamed: {entry}");
            assert!(entry["chapter"].as_str().is_some());
        }
    }

    #[test]
    fn breakers_json_absorbs_systems_cfg_instead_of_listing_it_separately() {
        let v: Value = serde_json::from_str(&breakers_json()).unwrap();
        assert_eq!(v["systemsCfgAbsorbed"], true);
        let list = v["breakers"].as_array().unwrap();
        assert_eq!(list.len(), crate::breakers::catalog().len());
        assert!(list.iter().all(|b| b["source"] == "catalogue"), "no raw systemsCfg entry should be emitted any more");
        assert!(list.iter().all(|b| b["gates"].is_string()), "every entry must report how it gates its consumer");
    }

    #[test]
    fn apply_action_rejects_bad_input_but_accepts_every_known_kind() {
        assert!(apply_action("not json").is_err());
        assert!(apply_action(r#"{"kind":"nonsense"}"#).is_err());
        assert!(apply_action(r#"{"kind":"toggleFailure"}"#).is_err(), "missing id");
        assert!(apply_action(r#"{"kind":"toggleFailure","id":21000}"#).is_ok());
        assert!(apply_action(r#"{"kind":"setFailureMagnitude"}"#).is_err(), "missing id/magnitude");
        assert!(apply_action(r#"{"kind":"setFailureMagnitude","id":21000,"magnitude":0.4}"#).is_ok());
        assert!(apply_action(r#"{"kind":"toggleBreaker","id":1}"#).is_ok());
        assert!(apply_action(r#"{"kind":"command","command":"fbw/efb/ground/gpu"}"#).is_ok());
        assert!(apply_action(r#"{"kind":"serviceOxygen"}"#).is_ok());
    }

    #[test]
    fn pull_and_reset_breaker_actions_validate_the_id_against_the_catalogue() {
        assert!(apply_action(r#"{"kind":"pullBreaker","id":"tr-1"}"#).is_ok());
        assert!(apply_action(r#"{"kind":"resetBreaker","id":"tr-1"}"#).is_ok());
        assert!(apply_action(r#"{"kind":"resetAllBreakers"}"#).is_ok());
        assert!(apply_action(r#"{"kind":"pullBreaker","id":"not-a-real-breaker"}"#).is_err());
        assert!(apply_action(r#"{"kind":"resetBreaker"}"#).is_err(), "missing id");
    }
}
