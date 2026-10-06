//! Legends, FCU, covers and RMP knobs on FlyByWire's and MSFS's own XML:
//! excerpts copied from the A380X package (behaviour/overhead/cargo-air-cond.xml,
//! legacy/generated/A32NX_Interior_Generics.xml, generic/buttons.xml,
//! A380_COCKPIT.xml, rmp.xml), FlyByWire's source tree (fcu.xml,
//! legacy/generated/A32NX_Interior_Autopilot.xml) and MSFS's
//! fs-base-aircraft-common ModelBehaviorDefs (Generic/Emissive.xml,
//! Generic/Complex/PushButton.xml), cut to the parts these controls use.

use std::collections::HashMap;

use super::bind::{self, Click, KEvents};
use super::emissive::{self, Drive};
use super::expand::{self, LightKind};
use super::rpn;
use super::sim;
use super::xml::{self, Library};

/// MSFS's Generic/Visibility.xml (ASOBO_GT_Visibility_Code),
/// Generic/Emissive.xml (ASOBO_GT_Material_Emissive_Code,
/// ASOBO_GT_Emissive_Gauge) and Generic/Complex/PushButton.xml
/// (ASOBO_GT_Push_Button_Airliner and the legend components of its
/// subtemplate).
const ASOBO: &str = r###"<ModelBehaviors>
	<Template Name = "ASOBO_GT_Visibility_Code">
		<DefaultTemplateParameters>
			<ONCE>False</ONCE>
		</DefaultTemplateParameters>

		<Visibility Once="#ONCE#">
			<Parameter>
				<Code>#VISIBILITY_CODE#</Code>
			</Parameter>
		</Visibility>
	</Template>
<Template Name = "ASOBO_GT_Material_Emissive_Code">
	<DefaultTemplateParameters>
		<FAILURE>1</FAILURE>
	</DefaultTemplateParameters>
	<OverrideTemplateParameters>
		<Condition NotEmpty="EMISSIVE_CODE_FACTOR">
			<True>
				<EMISSIVE_CODE_FACTOR>#EMISSIVE_CODE_FACTOR# *</EMISSIVE_CODE_FACTOR>
			</True>
			<False>
				<EMISSIVE_CODE_FACTOR></EMISSIVE_CODE_FACTOR>
			</False>
		</Condition>
		<Condition Valid="DONT_OVERRIDE_BASE_EMISSIVE">
			<True>
				<OVERRIDE_BASE_EMISSIVE>FALSE</OVERRIDE_BASE_EMISSIVE>
			</True>
			<False>
				<OVERRIDE_BASE_EMISSIVE>TRUE</OVERRIDE_BASE_EMISSIVE>
			</False>
		</Condition>
		<Condition NotEmpty="OVERRIDE_EMISSIVE_CODE">
			<True>
				<EMISSIVE_CODE>#OVERRIDE_EMISSIVE_CODE#</EMISSIVE_CODE>
			</True>
		</Condition>
	</OverrideTemplateParameters>
	<Material>
		<EmissiveFactor>
			<Parameter>
				<Code>#EMISSIVE_CODE# #FAILURE# * #EMISSIVE_CODE_FACTOR#</Code>
			</Parameter>
			<OverrideBaseEmissive>#OVERRIDE_BASE_EMISSIVE#</OverrideBaseEmissive>
		</EmissiveFactor>
	</Material>
</Template>
<Template Name = "ASOBO_GT_Emissive_Gauge">
	<Parameters Type="Default">
		<Condition Check="FAILURE_CIRCUIT">
			<False>
				<FAILURE>(A:CIRCUIT GENERAL PANEL ON, Bool)</FAILURE>
			</False>
			<True>
				<FAILURE>(A:CIRCUIT ON:#FAILURE_CIRCUIT#, Bool)</FAILURE>
			</True>
		</Condition>
	</Parameters>
	<Parameters Type="Override">
		<Condition Check="ALT_CODE">
			<EMISSIVE_CODE>#EMISSIVE_CODE# #ALT_CODE# max</EMISSIVE_CODE>
		</Condition>
	</Parameters>
	<Condition Valid="EMISSIVE_DRIVES_VISIBILITY">
		<True>
			<UseTemplate Name="ASOBO_GT_Visibility_Code">
				<VISIBILITY_CODE>#EMISSIVE_CODE# 0 &gt; #FAILURE# and</VISIBILITY_CODE>
			</UseTemplate>
		</True>
	</Condition>
	<UseTemplate Name="ASOBO_GT_Material_Emissive_Code">
	</UseTemplate>
</Template>
<Template Name = "ASOBO_GT_Push_Button_Airliner">
	<DefaultTemplateParameters>
		<SEQ1_SUFFIX>_SEQ1</SEQ1_SUFFIX>
		<SEQ2_SUFFIX>_SEQ2</SEQ2_SUFFIX>
	</DefaultTemplateParameters>
	<UseTemplate Name="ASOBO_GT_Push_Button_Airliner_SubTemplate">
	</UseTemplate>
</Template>
<Template Name = "ASOBO_GT_Push_Button_Airliner_SubTemplate">
	<Parameters Type="Default">
		<SEQ1_EMISSIVE_CODE>1</SEQ1_EMISSIVE_CODE>
		<SEQ2_EMISSIVE_CODE>1</SEQ2_EMISSIVE_CODE>
		<SEQ1_NODE_ID>#NODE_ID##SEQ1_SUFFIX#</SEQ1_NODE_ID>
		<SEQ2_NODE_ID>#NODE_ID##SEQ2_SUFFIX#</SEQ2_NODE_ID>
		<Condition Check="NOT_DIMMABLE">
			<True>
				<SEQ1_NOT_DIMMABLE>#NOT_DIMMABLE#</SEQ1_NOT_DIMMABLE>
				<SEQ2_NOT_DIMMABLE>#NOT_DIMMABLE#</SEQ2_NOT_DIMMABLE>
			</True>
			<False>
				<SEQ1_NOT_DIMMABLE>False</SEQ1_NOT_DIMMABLE>
				<SEQ2_NOT_DIMMABLE>False</SEQ2_NOT_DIMMABLE>
			</False>
		</Condition>
	</Parameters>
	<Condition Check="NO_SEQ1">
		<False>
			<Component ID="#SEQ1_NODE_ID#_S1" Node="#SEQ1_NODE_ID#">
				<UseTemplate Name ="ASOBO_GT_Emissive_Gauge">
					<Switch>
						<Case Valid="SEQ1_NOT_DIMMABLE">
							<EMISSIVE_CODE>#SEQ1_EMISSIVE_CODE#</EMISSIVE_CODE>
						</Case>
						<Case NotEmpty="POTENTIOMETER_SEQ1">
							<EMISSIVE_CODE>#SEQ1_EMISSIVE_CODE# (A:LIGHT POTENTIOMETER:#POTENTIOMETER_SEQ1#, Percent over 100) *</EMISSIVE_CODE>
						</Case>
						<Default>
							<EMISSIVE_CODE>#SEQ1_EMISSIVE_CODE#</EMISSIVE_CODE>
						</Default>
					</Switch>
				</UseTemplate>
			</Component>
		</False>
	</Condition>
	<Condition Check="NO_SEQ2">
		<False>
			<Component ID="#SEQ2_NODE_ID#_S2" Node="#SEQ2_NODE_ID#">
				<UseTemplate Name ="ASOBO_GT_Emissive_Gauge">
					<Switch>
						<Case Valid="SEQ2_NOT_DIMMABLE">
							<EMISSIVE_CODE>#SEQ2_EMISSIVE_CODE#</EMISSIVE_CODE>
						</Case>
						<Case NotEmpty="POTENTIOMETER_SEQ2">
							<EMISSIVE_CODE>#SEQ2_EMISSIVE_CODE# (A:LIGHT POTENTIOMETER:#POTENTIOMETER_SEQ2#, Percent over 100) *</EMISSIVE_CODE>
						</Case>
						<Default>
							<EMISSIVE_CODE>#SEQ2_EMISSIVE_CODE#</EMISSIVE_CODE>
						</Default>
					</Switch>
				</UseTemplate>
			</Component>
		</False>
	</Condition>
</Template>
</ModelBehaviors>"###;

/// A32NX_Interior_Generics.xml: FBW_Push_Toggle (the parts setting the
/// click and legend codes) and FBW_Covered_Push_Toggle.
const FBW_PUSH_TOGGLE: &str = r###"
    <Template Name="FBW_Push_Toggle">
        <DefaultTemplateParameters>
            <SEQ_POWERED>1</SEQ_POWERED>
            <SEQ1_POWERED>1</SEQ1_POWERED>
            <SEQ2_POWERED>1</SEQ2_POWERED>
            <EMISSIVE_DIM>(L:A32NX_OVHD_INTLT_ANN, number) 2 == if{ 0.1 } els{ 1 }</EMISSIVE_DIM>
        </DefaultTemplateParameters>
        <UseTemplate Name="ASOBO_GT_Push_Button_Airliner">
            <NODE_ID>#NODE_ID#</NODE_ID>
            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
            <Condition Check="LEFT_SINGLE_CODE">
                <True>
                    <Condition Check="MOMENTARY">
                        <True>
                            <LEFT_SINGLE_CODE>#LEFT_SINGLE_CODE#</LEFT_SINGLE_CODE>
                        </True>
                        <False>
                            <LEFT_SINGLE_CODE>
                                #LEFT_SINGLE_CODE#

                                (L:XMLVAR_Momentary_#NODE_ID#_Pressed) ! (&gt;L:XMLVAR_Momentary_#NODE_ID#_Pressed)
                            </LEFT_SINGLE_CODE>
                        </False>
                    </Condition>
                </True>
            </Condition>
            <Condition Check="TOGGLE_SIMVAR">
                <True>
                    <LEFT_SINGLE_CODE>(#TOGGLE_SIMVAR#, Bool) ! (&gt;#TOGGLE_SIMVAR#)</LEFT_SINGLE_CODE>
                    <Condition Check="MOMENTARY">
                        <False>
                            <Condition Check="DOWN_CODE">
                                <True>
                                    <DOWN_STATE_CODE>#DOWN_CODE#</DOWN_STATE_CODE>
                                </True>
                                <False>
                                    <Condition Check="INVERTED_ANIMATION">
                                        <True>
                                            <DOWN_STATE_CODE>(#TOGGLE_SIMVAR#) !</DOWN_STATE_CODE>
                                        </True>
                                        <False>
                                            <DOWN_STATE_CODE>(#TOGGLE_SIMVAR#)</DOWN_STATE_CODE>
                                        </False>
                                    </Condition>
                                </False>
                            </Condition>
                        </False>
                    </Condition>
                </True>
            </Condition>
            <Condition Check="DISABLE_SEQ1">
                <False>
                    <Condition Check="SEQ1_CODE">
                        <True>
                            <SEQ1_EMISSIVE_CODE>#SEQ1_CODE# (L:A32NX_OVHD_INTLT_ANN) 0 == or #SEQ1_POWERED# and #SEQ_POWERED# and #EMISSIVE_DIM# *</SEQ1_EMISSIVE_CODE>
                        </True>
                        <False>
                            <SEQ1_EMISSIVE_CODE>(L:A32NX_OVHD_INTLT_ANN) 0 == #SEQ1_POWERED# and #SEQ_POWERED# and #EMISSIVE_DIM# *</SEQ1_EMISSIVE_CODE>
                        </False>
                    </Condition>
                </False>
                <True>
                    <SEQ1_EMISSIVE_CODE>0</SEQ1_EMISSIVE_CODE>
                </True>
            </Condition>
            <Condition Check="DISABLE_SEQ2">
                <False>
                    <Condition Check="SEQ2_CODE">
                        <True>
                            <SEQ2_EMISSIVE_CODE>#SEQ2_CODE# (L:A32NX_OVHD_INTLT_ANN) 0 == or #SEQ2_POWERED# and #SEQ_POWERED# and #EMISSIVE_DIM# *</SEQ2_EMISSIVE_CODE>
                        </True>
                        <False>
                            <SEQ2_EMISSIVE_CODE>(L:A32NX_OVHD_INTLT_ANN) 0 == #SEQ2_POWERED# and #SEQ_POWERED# and #EMISSIVE_DIM# *</SEQ2_EMISSIVE_CODE>
                        </False>
                    </Condition>
                </False>
                <True>
                    <SEQ2_EMISSIVE_CODE>0</SEQ2_EMISSIVE_CODE>
                </True>
            </Condition>
        </UseTemplate>
    </Template>

    <Template Name="FBW_Covered_Push_Toggle">
        <UseTemplate Name="ASOBO_GT_Switch_Dummy">
            <NODE_ID>#LOCK_NODE_ID#</NODE_ID>
            <ANIM_NAME>#LOCK_NODE_ID#</ANIM_NAME>
            <WWISE_EVENT_1/>
            <WWISE_EVENT_2/>
            <LEFT_SINGLE_CODE>(#TOGGLE_SIMVAR#_LOCK) ! (&gt;#TOGGLE_SIMVAR#_LOCK)</LEFT_SINGLE_CODE>
        </UseTemplate>

        <UseTemplate Name="FBW_Push_Toggle"/>
    </Template>
"###;

/// generic/buttons.xml: FBW_A380X_BacklightIndicator_Button_Template.
const BACKLIGHT_INDICATOR: &str = r###"
    <Template Name="FBW_A380X_BacklightIndicator_Button_Template">
        <DefaultTemplateParameters>
            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
            <INDICATOR_NODE_ID>#NODE_ID#_SEQ1</INDICATOR_NODE_ID>
            <BACKLIGHT_NODE_ID>#NODE_ID#_SEQ2</BACKLIGHT_NODE_ID>
            <MINIMUM_INDICATOR_BRIGHTNESS>0</MINIMUM_INDICATOR_BRIGHTNESS>
        </DefaultTemplateParameters>
        <OverrideTemplateParameters>
            <Condition Check="TOGGLE_SIMVAR">
                <BUTTON_CODE>(#TOGGLE_SIMVAR#, Boolean) ! (&gt;#TOGGLE_SIMVAR#, Boolean)</BUTTON_CODE>
                <INDICATOR_CODE>(#TOGGLE_SIMVAR#, Boolean)</INDICATOR_CODE>
            </Condition>
            <Condition Check="INOP">
                <INDICATOR_POWERED>0</INDICATOR_POWERED>
            </Condition>
            <Condition Check="NODE_IS_INDICATOR">
                <INDICATOR_NODE_ID>#NODE_ID#</INDICATOR_NODE_ID>
                <MINIMUM_INDICATOR_BRIGHTNESS>0.004</MINIMUM_INDICATOR_BRIGHTNESS>
            </Condition>
            <Condition Check="NODE_IS_BACKLIGHT">
                <BACKLIGHT_NODE_ID>#NODE_ID#</BACKLIGHT_NODE_ID>
            </Condition>
        </OverrideTemplateParameters>
        <Component ID="#NODE_ID#" Node="#NODE_ID#">
            <Component ID="#NODE_ID#_BUTTON" Node="#NODE_ID#">
                <UseTemplate Name="ASOBO_GT_Push_Button">
                    <LEFT_SINGLE_CODE>
                        #INDICATOR_POWERED# if{
                            #BUTTON_CODE#
                        }
                    </LEFT_SINGLE_CODE>
                </UseTemplate>
            </Component>
        </Component>
    </Template>
"###;

/// FlyByWire's source fcu.xml (FBW_A380X_AP_PushButton, the FCU's global
/// parameters) and A32NX_Interior_Autopilot.xml (the speed knob).
const FCU_SOURCE: &str = r###"
    <Template Name="FBW_A380X_AP_PushButton">
        <DefaultTemplateParameters>
            <NODE_ID>PUSH_FCU_AP#ID#</NODE_ID>
            <INDICATOR_CODE>(L:A32NX_FCU_AP_#ID#_LIGHT_ON, Bool)</INDICATOR_CODE>
            <BUTTON_CODE>'A32NX.FCU_AP_#ID#_PUSH' (&gt;F:KeyEvent)</BUTTON_CODE>
            <NODE_IS_INDICATOR />
        </DefaultTemplateParameters>
        <UseTemplate Name="FBW_A380X_BacklightIndicator_Button_Template" />
    </Template>
    <Template Name="FBW_AUTOPILOT_Knob_SpeedMach_Template">
        <DefaultTemplateParameters>
            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
            <PART_ID>#NODE_ID#</PART_ID>
            <ANIM_NAME_PUSHPULL>#NODE_ID#_PUSHPULL</ANIM_NAME_PUSHPULL>
        </DefaultTemplateParameters>
        <Component ID="#NODE_ID#" Node="#NODE_ID#">
            <UseTemplate Name="ASOBO_GT_Knob_Infinite_PushPull">
                <ANIM_NAME_KNOB>#ANIM_NAME#</ANIM_NAME_KNOB>
                <ANTICLOCKWISE_CODE>
                    'A32NX.FCU_SPD_DEC' (&gt;F:KeyEvent)
                </ANTICLOCKWISE_CODE>
                <CLOCKWISE_CODE>
                    'A32NX.FCU_SPD_INC' (&gt;F:KeyEvent)
                </CLOCKWISE_CODE>
                <PULL_CODE>
                    'A32NX.FCU_SPD_PULL' (&gt;F:KeyEvent)
                </PULL_CODE>
                <PUSH_CODE>
                    'A32NX.FCU_SPD_PUSH' (&gt;F:KeyEvent)
                </PUSH_CODE>
            </UseTemplate>
        </Component>
    </Template>
"###;

fn library(templates: &[&str], components: &str) -> Library {
    Library::from_text(&format!("<ModelBehaviors>{}{components}</ModelBehaviors>", templates.concat())).unwrap()
}

/// The package's lights over MSFS's own templates, with the general panel
/// circuit always on (sim::sim_state's constant).
fn lights(lib: &Library) -> emissive::Lights {
    let base = Library::templates_from_text(ASOBO);
    let e = expand::expand_with(lib, Some(&base));
    let consts = HashMap::from([("A:CIRCUIT GENERAL PANEL ON".to_string(), 1.0)]);
    emissive::resolve(&e.lights, &consts)
}

fn value(code: &str, state: &[(&str, f64)]) -> f64 {
    let mut st: HashMap<String, f64> = state.iter().map(|(k, v)| (k.to_string(), *v)).collect();
    st.insert("A:CIRCUIT GENERAL PANEL ON".into(), 1.0);
    match rpn::run(code, &st, &KEvents).stack.last() {
        Some(rpn::Val::Num(n)) => *n,
        _ => f64::NAN,
    }
}

#[test]
fn cargo_isol_valves_legends_follow_fault_on_power_and_ann_lt() {
    // overhead/cargo-air-cond.xml, FWD CARGO ISOL VALVES.
    let lib = library(
        &[FBW_PUSH_TOGGLE],
        r###"<Component ID="OVHD_CARGO_AIR_COND">
        <UseTemplate Name="FBW_Push_Toggle">
            <NODE_ID>PUSH_OVHD_AIRCOND_FWDCARGO</NODE_ID>
            <PART_ID>PUSH_OVHD_AIRCOND_FWDCARGO</PART_ID>
            <TOGGLE_SIMVAR>L:A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_IS_ON</TOGGLE_SIMVAR>
            <SEQ_POWERED>(L:A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED, Bool)</SEQ_POWERED> <!-- TODO verify this -->
            <SEQ1_CODE>(L:A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_HAS_FAULT, Bool)</SEQ1_CODE>
            <SEQ2_CODE>(L:A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_IS_ON, Bool) !</SEQ2_CODE>
            <SEQ2_EMISSIVE_DRIVES_VISIBILITY>False</SEQ2_EMISSIVE_DRIVES_VISIBILITY>
            <SEQ1_CODE_DRIVES_VISIBILITY>False</SEQ1_CODE_DRIVES_VISIBILITY>
            <SEQ2_CODE_DRIVES_VISIBILITY>False</SEQ2_CODE_DRIVES_VISIBILITY>
        </UseTemplate>
        </Component>"###,
    );
    let l = lights(&lib);
    let get = |node: &str| l.lights.iter().find(|x| x.node == node && x.kind == LightKind::Emissive).unwrap_or_else(|| panic!("{node}: {:?}", l.lights));
    let (fault, off) = (get("PUSH_OVHD_AIRCOND_FWDCARGO_SEQ1"), get("PUSH_OVHD_AIRCOND_FWDCARGO_SEQ2"));
    assert_eq!(fault.template, "FBW_Push_Toggle");
    // Bus power, the fault and the ANN LT switch together: SASL.
    let (Ok(Drive::Lua(i1)), Ok(Drive::Lua(i2))) = (&fault.drive, &off.drive) else { panic!("{:?} {:?}", fault.drive, off.drive) };
    let lua = &l.lua[*i1];
    assert!(lua.contains("rd(\"fbw/A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED\")") && lua.contains("rd(\"fbw/A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_HAS_FAULT\")"), "{lua}");
    assert!(!lua.contains("CIRCUIT_GENERAL_PANEL"), "the always-powered panel circuit is a constant: {lua}");
    assert!(l.lua[*i2].contains("fbw/A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_IS_ON"));
    assert!(fault.reads.contains("fbw/A32NX_OVHD_INTLT_ANN"));
    // What MSFS shows (ANN LT: 0 test, 1 bright, 2 dim).
    let (bus, flt, on, ann) = ("L:A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED", "L:A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_HAS_FAULT", "L:A32NX_OVHD_CARGO_AIR_ISOL_VALVES_FWD_PB_IS_ON", "L:A32NX_OVHD_INTLT_ANN");
    assert_eq!(value(&fault.code, &[(bus, 1.0), (flt, 1.0), (ann, 1.0)]), 1.0);
    assert_eq!(value(&fault.code, &[(bus, 0.0), (flt, 1.0), (ann, 1.0)]), 0.0, "no power, no legend");
    assert_eq!(value(&fault.code, &[(bus, 1.0), (flt, 0.0), (ann, 1.0)]), 0.0);
    assert_eq!(value(&fault.code, &[(bus, 1.0), (flt, 0.0), (ann, 0.0)]), 1.0, "annunciator test");
    assert!((value(&fault.code, &[(bus, 1.0), (flt, 1.0), (ann, 2.0)]) - 0.1).abs() < 1e-12, "dim");
    assert_eq!(value(&off.code, &[(bus, 1.0), (on, 0.0), (ann, 1.0)]), 1.0, "OFF legend while the valves are off");
    assert_eq!(value(&off.code, &[(bus, 1.0), (on, 1.0), (ann, 1.0)]), 0.0);
}

#[test]
fn a_single_variable_legend_is_a_light_level_on_its_dataref() {
    // MSFS's ASOBO_GT_Emissive_Gauge on a node, as FlyByWire's templates use it.
    let lib = library(
        &[],
        r###"<Component ID="c">
            <Component ID="A" Node="LIGHT_A"><UseTemplate Name="ASOBO_GT_Emissive_Gauge"><EMISSIVE_CODE>(L:A32NX_X, Bool)</EMISSIVE_CODE></UseTemplate></Component>
            <Component ID="B" Node="LIGHT_B"><UseTemplate Name="ASOBO_GT_Emissive_Gauge"><EMISSIVE_CODE>(L:A32NX_X, Bool) !</EMISSIVE_CODE><EMISSIVE_DRIVES_VISIBILITY>True</EMISSIVE_DRIVES_VISIBILITY></UseTemplate></Component>
            <Component ID="C" Node="LIGHT_C"><UseTemplate Name="ASOBO_GT_Emissive_Gauge"><EMISSIVE_CODE>(A:LIGHT POTENTIOMETER:84, Percent over 100)</EMISSIVE_CODE></UseTemplate></Component>
        </Component>"###,
    );
    let l = lights(&lib);
    let drive = |node: &str, kind| l.lights.iter().find(|x| x.node == node && x.kind == kind).map(|x| x.drive.clone());
    assert_eq!(drive("LIGHT_A", LightKind::Emissive), Some(Ok(Drive::Direct { dref: "fbw/A32NX_X".into(), v1: 0.0, v2: 1.0 })));
    assert_eq!(drive("LIGHT_B", LightKind::Emissive), Some(Ok(Drive::Direct { dref: "fbw/A32NX_X".into(), v1: 1.0, v2: 0.0 })));
    // Shown while the variable is not 1.
    assert_eq!(drive("LIGHT_B", LightKind::Visibility), Some(Ok(Drive::Direct { dref: "fbw/A32NX_X".into(), v1: 1.0, v2: 0.0 })));
    // The potentiometer's dataref holds a ratio: full brightness at 1.
    assert_eq!(drive("LIGHT_C", LightKind::Emissive), Some(Ok(Drive::Direct { dref: "fbw/LIGHT_POTENTIOMETER_84".into(), v1: 0.0, v2: 1.0 })));
}

#[test]
fn fcu_buttons_and_knobs_fire_the_systems_plugin_commands() {
    // FlyByWire's source fcu.xml: the AP1 pushbutton and the speed knob.
    let lib = library(
        &[BACKLIGHT_INDICATOR, FCU_SOURCE],
        r###"<Component ID="FCU">
            <DefaultTemplateParameters>
                <INDICATOR_POWERED>(L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED, Bool) (L:A32NX_ELEC_DC_2_BUS_IS_POWERED, Bool) or</INDICATOR_POWERED>
                <BACKLIGHT_POWERED>(L:A32NX_ELEC_DC_1_BUS_IS_POWERED, Bool)</BACKLIGHT_POWERED>
                <BACKLIGHT_POTENTIOMETER>84</BACKLIGHT_POTENTIOMETER>
            </DefaultTemplateParameters>
            <UseTemplate Name="FBW_AUTOPILOT_Knob_SpeedMach_Template">
                <NODE_ID>KNOB_FCU_SPEED</NODE_ID>
                <ANIM_NAME>KNOB_FCU_SPEED</ANIM_NAME>
                <ANIM_NAME_PUSHPULL>PUSH_KNOB_FCU_SPEED</ANIM_NAME_PUSHPULL>
                <TYPE>AIRBUS</TYPE>
            </UseTemplate>
            <UseTemplate Name="FBW_A380X_AP_PushButton">
                <ID>1</ID>
            </UseTemplate>
        </Component>"###,
    );
    let r = bind::resolve_all(&expand::expand(&lib).leaves);
    let ap = r.binding("PUSH_FCU_AP1").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    // Powered FCU only: SASL runs the gate and fires the command.
    let Click::Script(sc) = &ap.click else { panic!("{:?}", ap.click) };
    let press = sc.press.as_deref().unwrap();
    assert!(press.contains("rd(\"fbw/A32NX_ELEC_DC_ESS_BUS_IS_POWERED\")") && press.contains("CMD(\"fbw/event/A32NX_FCU_AP_1_PUSH\")"), "{press}");
    assert!(ap.commands.contains("fbw/event/A32NX_FCU_AP_1_PUSH"));
    let knob = r.binding("KNOB_FCU_SPEED").unwrap();
    assert_eq!(knob.click, Click::CommandKnob { up: "fbw/event/A32NX_FCU_SPD_INC".into(), down: "fbw/event/A32NX_FCU_SPD_DEC".into() });
    // Push and pull share the knob's part.
    assert!(r.unresolved.iter().any(|u| u.anim == "PUSH_KNOB_FCU_SPEED" && u.reason.contains("A32NX.FCU_SPD_PUSH")), "{:?}", r.unresolved);

    // The package's FCU fires H: events its instrument forwards unchanged
    // (fcu.js AutopilotManager): the same command.
    let lua = bind::to_lua("(L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED, Bool) if{ (>H:A320_Neo_FCU_AP_1_PUSH) }", "c").unwrap();
    assert!(lua.contains("CMD(\"fbw/event/A32NX_FCU_AP_1_PUSH\")"), "{lua}");
    // ... and the speed knob's own H: events (whose selection that
    // JavaScript would otherwise keep, X-Plane never running it) go straight
    // to the same fbw/event command the JS would have ended up sending.
    assert!(bind::to_lua("(>H:A320_Neo_FCU_SPEED_INC)", "c").unwrap().contains("CMD(\"fbw/event/A32NX_FCU_SPD_INC\")"));
    // A FlyByWire event the plugin has no command for says so.
    let e = bind::to_lua("'A32NX.FCU_EFIS_L_LS_PUSH' (>F:KeyEvent)", "c").unwrap_err();
    assert!(e.contains("no fbw/event command"), "{e}");
}

#[test]
fn a_covered_button_without_a_variable_clicks_only_with_its_cover_open() {
    // A380_COCKPIT.xml, IDG 1 (FBW_Covered_Push_Toggle without TOGGLE_SIMVAR).
    let lib = library(
        &[FBW_PUSH_TOGGLE],
        r###"<Component ID="Overhead_Electricals">
                <UseTemplate Name="FBW_Covered_Push_Toggle">
                    <NODE_ID>PUSH_OVHD_ELEC_IDG1</NODE_ID>
                    <LOCK_NODE_ID>LOCK_OVHD_ELEC_IDG1</LOCK_NODE_ID>
                    <LEFT_SINGLE_CODE>1 (&gt;L:A32NX_OVHD_ELEC_IDG_1_PB_IS_RELEASED)</LEFT_SINGLE_CODE>
                    <SEQ_POWERED>(L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED, Bool)</SEQ_POWERED>
                    <SEQ1_CODE>(L:A32NX_OVHD_ELEC_IDG_1_PB_HAS_FAULT)</SEQ1_CODE>
                    <SEQ2_CODE>(L:A32NX_OVHD_ELEC_IDG_1_PB_IS_DISC)</SEQ2_CODE>
                    <MOMENTARY />
                </UseTemplate>
            </Component>"###,
    );
    let r = bind::resolve_all(&expand::expand(&lib).leaves);
    let cover = r.binding("LOCK_OVHD_ELEC_IDG1").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    assert_eq!(cover.click, Click::Toggle { dref: "fbw/cockpit/cover/LOCK_OVHD_ELEC_IDG1".into(), on: 1.0, off: 0.0 });
    assert!(cover.look.as_deref().is_some_and(|l| l.contains("fbw/cockpit/cover/LOCK_OVHD_ELEC_IDG1")));
    let button = r.binding("PUSH_OVHD_ELEC_IDG1").unwrap();
    let Click::Script(sc) = &button.click else { panic!("{:?}", button.click) };
    let press = sc.press.as_deref().unwrap();
    assert!(press.contains("rd(\"fbw/cockpit/cover/LOCK_OVHD_ELEC_IDG1\")") && press.contains("wr(\"fbw/A32NX_OVHD_ELEC_IDG_1_PB_IS_RELEASED\""), "{press}");
    // The gate, run as MSFS code: closed does nothing, open releases the IDG.
    let code = "(L:FBW_COCKPIT_COVER_LOCK_OVHD_ELEC_IDG1, Bool) if{ 1 (>L:A32NX_OVHD_ELEC_IDG_1_PB_IS_RELEASED) }";
    let closed = rpn::run(code, &HashMap::new(), &KEvents);
    assert!(closed.writes.is_empty());
    let open = rpn::run(code, &HashMap::from([("L:FBW_COCKPIT_COVER_LOCK_OVHD_ELEC_IDG1".to_string(), 1.0)]), &KEvents);
    assert_eq!(open.writes, vec![("L:A32NX_OVHD_ELEC_IDG_1_PB_IS_RELEASED".to_string(), 1.0)]);
}

#[test]
fn a_push_button_with_a_leave_code_resets_its_pressed_var_on_release() {
    // mip.xml PUSH_RTO_ARM: unlike the IDG button above, FBW pairs this
    // one's LEFT_SINGLE_CODE with a LEFT_LEAVE_CODE that resets the
    // momentary "pressed" simvar a380_systems' PressSingleSignalButton
    // edge-detects (fbw-common overhead/mod.rs: `is_pressed = current &&
    // !last`). Losing LEFT_LEAVE_CODE left the button working once, then
    // permanently stuck.
    let lib = library(
        &[FBW_PUSH_TOGGLE],
        r###"<Component ID="Autobrake">
                <UseTemplate Name="FBW_Push_Toggle">
                    <NODE_ID>PUSH_RTO_ARM</NODE_ID>
                    <SEQ2_CODE>(L:A32NX_AUTOBRAKES_RTO_ARMED)</SEQ2_CODE>
                    <LEFT_SINGLE_CODE>1 (&gt;L:A32NX_OVHD_AUTOBRK_RTO_ARM_IS_PRESSED)</LEFT_SINGLE_CODE>
                    <LEFT_LEAVE_CODE>0 (&gt;L:A32NX_OVHD_AUTOBRK_RTO_ARM_IS_PRESSED)</LEFT_LEAVE_CODE>
                    <MOMENTARY />
                </UseTemplate>
            </Component>"###,
    );
    let r = bind::resolve_all(&expand::expand(&lib).leaves);
    let button = r.binding("PUSH_RTO_ARM").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    // Once LEFT_LEAVE_CODE is captured, `resolve()`'s own pre-existing
    // "Held: one variable, set on press and reset on release" pattern
    // (bind.rs, matching `press`/`release` against the same dataref at two
    // different values) recognizes this exact press=1/release=0 pair and
    // resolves it to a native `Click::Hold` -- an `ATTR_manip_push` that
    // writes 1 on mouse-down and 0 on mouse-up directly, with no SASL
    // polling loop at all. That is a stronger fix than a `Click::Script`
    // with press/release Lua bodies would have been: X-Plane itself
    // guarantees the reset on release, not a Lua callback that could be
    // skipped. Before this fix (LEFT_LEAVE_CODE dropped, release always
    // `None`), this same "Held" pattern could never match (it requires
    // both a press and a release value), so the button fell through to a
    // bare `Click::Command`/`Click::Script` with no release at all --
    // exactly the "stuck at 1 after the first press" bug.
    assert_eq!(
        button.click,
        Click::Hold { dref: "fbw/A32NX_OVHD_AUTOBRK_RTO_ARM_IS_PRESSED".to_string(), down: 1.0, up: 0.0 },
        "{:?}",
        button.click
    );
    // Same RPN, run directly: mouse-up must write the var back to 0 so the
    // next press is a fresh 0->1 edge.
    let leave_code = "0 (>L:A32NX_OVHD_AUTOBRK_RTO_ARM_IS_PRESSED)";
    let out = rpn::run(leave_code, &HashMap::new(), &KEvents);
    assert_eq!(out.writes, vec![("L:A32NX_OVHD_AUTOBRK_RTO_ARM_IS_PRESSED".to_string(), 0.0)]);
}

#[test]
fn an_rmp_volume_knob_steps_the_rmp_variables_by_one_percent() {
    // rmp.xml FBW_A380X_RMP_Audio_Knob, as expanded for RMP 1 VHF 1 (the
    // package's own template calls MSFS's ASOBO_Interaction_Base_Template).
    let lib = library(
        &[],
        r###"<Component ID="RMP_1">
            <UseTemplate Name="ASOBO_Interaction_Base_Template">
                <NODE_ID>KNOB_RMP_1_VHF1</NODE_ID>
                <USE_INPUT_EVENT_ID>A380X_PED_RMP_1</USE_INPUT_EVENT_ID>
                <INPUT_EVENT_ID_SOURCE>A380X_PED_RMP_1</INPUT_EVENT_ID_SOURCE>
                <IE_NAME>VHF_VOL_1_KNOB</IE_NAME>
                <INTERACTION_TYPE>Knob</INTERACTION_TYPE>
                <TEMPLATE_VARIANT>Switch</TEMPLATE_VARIANT>
                <ANIM_NAME_KNOB>KNOB_RMP_1_VHF1_ROTATE_ANIM</ANIM_NAME_KNOB>
                <GET_STATE_EXTERNAL>(L:A380X_RMP_1_VHF_VOL_1)</GET_STATE_EXTERNAL>
                <SET_STATE_EXTERNAL>
                p0 100 min 0 max (&gt;L:A380X_RMP_1_VHF_VOL_1)
                (L:A32NX_FO_SYNC_EFIS_ENABLED, Bool) if{
                    p0 100 min 0 max (&gt;L:A380X_RMP_2_VHF_VOL_1)
                    p0 100 min 0 max (&gt;L:A380X_RMP_3_VHF_VOL_1)
                }
            </SET_STATE_EXTERNAL>
            </UseTemplate>
            <UseTemplate Name="ASOBO_Interaction_Push_Event_Base_Template">
                <NODE_ID>KNOB_RMP_1_VHF1</NODE_ID>
                <USE_INPUT_EVENT_ID>A380X_PED_RMP_1</USE_INPUT_EVENT_ID>
                <IE_NAME>VHF_VOL_1_PUSH</IE_NAME>
                <ANIM_NAME_SWITCH>KNOB_RMP_1_VHF1_PUSH_ANIM</ANIM_NAME_SWITCH>
                <GET_STATE_EXTERNAL>(L:A380X_RMP_1_VHF_VOL_RX_SWITCH_1) sp0</GET_STATE_EXTERNAL>
                <SET_STATE_EXTERNAL>p0 (&gt;L:A380X_RMP_1_VHF_VOL_RX_SWITCH_1)</SET_STATE_EXTERNAL>
            </UseTemplate>
        </Component>"###,
    );
    let (controls, _) = bind::controls(&expand::expand(&lib).leaves);
    let knob = controls.iter().find(|c| c.anim == "KNOB_RMP_1_VHF1_ROTATE_ANIM").unwrap();
    let Ok(bind::Action::Rotary { cw, ccw }) = &knob.action else { panic!("{:?}", knob.action) };
    let at = |code: &str, vol: f64, sync: f64| -> HashMap<String, f64> {
        let st = HashMap::from([("L:A380X_RMP_1_VHF_VOL_1".to_string(), vol), ("L:A32NX_FO_SYNC_EFIS_ENABLED".to_string(), sync)]);
        rpn::run(code, &st, &KEvents).writes.into_iter().collect()
    };
    assert_eq!(at(cw, 40.0, 0.0), HashMap::from([("L:A380X_RMP_1_VHF_VOL_1".to_string(), 41.0)]));
    assert_eq!(at(cw, 100.0, 0.0)["L:A380X_RMP_1_VHF_VOL_1"], 100.0, "clamped");
    assert_eq!(at(ccw, 0.0, 0.0)["L:A380X_RMP_1_VHF_VOL_1"], 0.0);
    let synced = at(ccw, 40.0, 1.0);
    assert_eq!((synced["L:A380X_RMP_2_VHF_VOL_1"], synced["L:A380X_RMP_3_VHF_VOL_1"]), (39.0, 39.0), "FO sync sets the other RMPs");
    let r = bind::resolve_all(&expand::expand(&lib).leaves);
    let b = r.binding("KNOB_RMP_1_VHF1_ROTATE_ANIM").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    assert!(matches!(&b.click, Click::Script(sc) if sc.knob && sc.up.as_deref().is_some_and(|u| u.contains("wr(\"fbw/A380X_RMP_1_VHF_VOL_1\""))), "{:?}", b.click);
    // The receive push flips its switch variable.
    let push = r.binding("KNOB_RMP_1_VHF1_PUSH_ANIM").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    assert_eq!(push.click, Click::Toggle { dref: "fbw/A380X_RMP_1_VHF_VOL_RX_SWITCH_1".into(), on: 1.0, off: 0.0 });
}

#[test]
fn start_values_come_from_the_flight_and_electrical_definitions() {
    // apron.FLT and systems.cfg excerpts from the A380X package.
    let flt = "[LocalVars.0]\nA32NX_OVHD_INTLT_ANN=1\nXMLVAR_ALT_MODE_REQUESTED=1\n\n[Switches.0]\nPanelLights=True\nPotentiometer.86=0.5\n\n[Engine Parameters.1.0]\nGeneratorSwitch=True\n";
    let cfg = "[ELECTRICAL]\nbus.1 = Name:INFINIBAT_BUS\nbattery.1 = Connections:bus.1#Capacity:99999999#Voltage:curve.1#Name:INFINIBAT ; INFINIBAT\n\
               circuit.1 = Type:CIRCUIT_GENERAL_PANEL#Connections:bus.1#Power:0.5,1,20.0#Name:General_Panel ; General panel\n\
               circuit.2 = Type:CIRCUIT_FUEL_PUMP:1#Connections:bus.1#Power:3, 5, 20.0#Name:Fuel_Pump1_Feed1 ; Fuel Pump 5W\n\
               circuit.63 = Type:CIRCUIT_LIGHT_PANEL:4#Connections:bus.1#Power:2, 5, 20.0#Name:Panel_Light_Overhead ; panel light o\n";
    let s = sim::sim_state(flt, cfg);
    assert_eq!(s.defaults["fbw/A32NX_OVHD_INTLT_ANN"], 1.0);
    assert_eq!(s.locals["L:XMLVAR_ALT_MODE_REQUESTED"], 1.0);
    assert_eq!(s.defaults["fbw/LIGHT_POTENTIOMETER_86"], 0.5);
    assert_eq!(s.defaults["fbw/GENERAL_ENG_MASTER_ALTERNATOR_1"], 1.0);
    assert_eq!(s.defaults["fbw/CIRCUIT_CONNECTION_ON_2"], 1.0);
    assert_eq!((s.defaults["fbw/LIGHT_PANEL_4"], s.defaults["fbw/LIGHT_PANEL_ON_4"]), (1.0, 1.0));
    assert_eq!(s.defaults["fbw/APU_GENERATOR_SWITCH_1"], 1.0);
    assert_eq!(s.constants["A:CIRCUIT GENERAL PANEL ON"], 1.0);
}

#[test]
fn msfs_xml_with_built_names_and_mismatched_closing_tags_parses() {
    // Inputs/Templates.xml:838-844 and Generic/Helpers.xml:51.
    let el = xml::parse(
        "<ModelBehaviors><Template Name=\"T\"><BINDING_SET_#FIRST#_PARAM_0>1</BINDING_SET_0_PARAM_#FIRST#_PARAM_0><#PARAM_NAME#>x</#PARAM_NAME#></Template><Macro Name=\"POT\">78</Macro></ModelBehaviors>",
    )
    .unwrap();
    assert_eq!(el.kids[0].kids[0].name, "BINDING_SET_#FIRST#_PARAM_0");
    assert_eq!(el.kids[0].kids[1].name, "#PARAM_NAME#");
    let lib = Library::from_text("<ModelBehaviors><Macro Name=\"POT_EFIS_CS_OIT\">78</Macro></ModelBehaviors>").unwrap();
    assert_eq!(xml::expand_macros("(A:LIGHT POTENTIOMETER:@POT_EFIS_CS_OIT, Percent over 100)", &lib.macros), "(A:LIGHT POTENTIOMETER:78, Percent over 100)");
}

/// FlyByWire's source overhead/fire.xml (FBW_Airbus_FIRE_GUARD,
/// FBW_Airbus_FIRE_BUTTON), trimmed to the click logic: the guard cover
/// toggles L:A32NX_FIRE_GUARD_<TYPE><ID> only while the fire button itself
/// is not pressed, and the button toggles L:A32NX_FIRE_BUTTON_<TYPE><ID>
/// only while its guard is open (PushButton.xml's own gating, reproduced in
/// RPN since the A380X gives these no COVER_NODE_ID/LOCK_NODE_ID).
const FIRE: &str = r###"<ModelBehaviors>
    <Template Name="FBW_Airbus_FIRE_BUTTON">
        <DefaultTemplateParameters>
            <NODE_ID>#NODE_ID#</NODE_ID>
            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
            <TOGGLE_VAR>L:A32NX_FIRE_BUTTON_#TYPE##ID#</TOGGLE_VAR>
        </DefaultTemplateParameters>
        <Component ID="#NODE_ID#" Node="#NODE_ID#">
            <UseTemplate Name="ASOBO_GT_Interaction_LeftSingle_Leave_Code">
                <LEFT_SINGLE_CODE>
                    (L:A32NX_FIRE_GUARD_#TYPE##ID#) 1 == if{
                        (#TOGGLE_VAR#) ! (&gt;#TOGGLE_VAR#)
                    }
                </LEFT_SINGLE_CODE>
                <LEFT_LEAVE_CODE/>
            </UseTemplate>
        </Component>
    </Template>

    <Template Name="FBW_Airbus_FIRE_GUARD">
        <DefaultTemplateParameters>
            <NODE_ID>#NODE_ID#</NODE_ID>
            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
            <LEFT_SINGLE_CODE/>
            <TOGGLE_VAR>L:A32NX_FIRE_GUARD_#TYPE##ID#</TOGGLE_VAR>
        </DefaultTemplateParameters>
        <Component ID="#NODE_ID#" Node="#NODE_ID#">
            <UseTemplate Name="ASOBO_GT_Interaction_LeftSingle_Leave_Code">
                <LEFT_SINGLE_CODE>
                    (L:A32NX_FIRE_BUTTON_#TYPE##ID#) 0 == if{
                        (#TOGGLE_VAR#) ! (&gt;#TOGGLE_VAR#)
                    } #LEFT_SINGLE_CODE#
                </LEFT_SINGLE_CODE>
                <LEFT_LEAVE_CODE/>
            </UseTemplate>
        </Component>
    </Template>
</ModelBehaviors>"###;

#[test]
fn engine_fire_pushbutton_only_toggles_while_its_guard_is_open() {
    // A380_Cockpit_Behavior.xml's own Overhead_Fire/ENG1 component: TYPE,
    // ID and BUTTON_ID set once and inherited by both templates.
    let lib = library(
        &[FIRE],
        r###"<Component ID="ENG1">
            <DefaultTemplateParameters>
                <TYPE>ENG</TYPE>
                <ID>1</ID>
                <BUTTON_ID>ENG1</BUTTON_ID>
            </DefaultTemplateParameters>
            <UseTemplate Name="FBW_Airbus_FIRE_GUARD">
                <NODE_ID>A380X_OVHD_ENG1_FIRE_GUARD</NODE_ID>
                <PART_ID>A380X_OVHD_ENG1_FIRE_GUARD</PART_ID>
            </UseTemplate>
            <UseTemplate Name="FBW_Airbus_FIRE_BUTTON">
                <NODE_ID>PUSH_OVHD_FIRE_ENG1</NODE_ID>
                <PART_ID>FIRE_ENG1</PART_ID>
            </UseTemplate>
        </Component>"###,
    );
    let r = bind::resolve_all(&expand::expand(&lib).leaves);

    let button = r.binding("PUSH_OVHD_FIRE_ENG1").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    let Click::Script(bs) = &button.click else { panic!("{:?}", button.click) };
    let press = bs.press.as_deref().unwrap();
    assert!(press.contains("rd(\"fbw/A32NX_FIRE_GUARD_ENG1\")"), "the button's press must read the guard first: {press}");
    assert!(press.contains("wr(\"fbw/A32NX_FIRE_BUTTON_ENG1\""), "{press}");
    assert!(button.targets.contains("fbw/A32NX_FIRE_BUTTON_ENG1"), "{:?}", button.targets);

    let guard = r.binding("A380X_OVHD_ENG1_FIRE_GUARD").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    let Click::Script(gs) = &guard.click else { panic!("{:?}", guard.click) };
    let gpress = gs.press.as_deref().unwrap();
    assert!(gpress.contains("rd(\"fbw/A32NX_FIRE_BUTTON_ENG1\")"), "the guard's press must read the button first: {gpress}");
    assert!(gpress.contains("wr(\"fbw/A32NX_FIRE_GUARD_ENG1\""), "{gpress}");
    assert!(guard.targets.contains("fbw/A32NX_FIRE_GUARD_ENG1"), "{:?}", guard.targets);

    // Guard closed, button not pressed: clicking the button changes nothing
    // (RPN semantics, not just source text) until the guard is opened.
    let run = |code: &str, guard_open: f64, button_on: f64| {
        let st = HashMap::from([("L:A32NX_FIRE_GUARD_ENG1".to_string(), guard_open), ("L:A32NX_FIRE_BUTTON_ENG1".to_string(), button_on)]);
        rpn::run(code, &st, &KEvents).writes
    };
    // The two RPN codes fed to `to_lua` above came from FBW's own XML
    // (LEFT_SINGLE_CODE); re-run them directly through the interpreter to
    // check the gate, independent of the Lua text assertions above.
    let button_code = "(L:A32NX_FIRE_GUARD_ENG1) 1 == if{ (L:A32NX_FIRE_BUTTON_ENG1) ! (>L:A32NX_FIRE_BUTTON_ENG1) }";
    assert!(run(button_code, 0.0, 0.0).is_empty(), "guard closed: no write");
    assert_eq!(run(button_code, 1.0, 0.0), vec![("L:A32NX_FIRE_BUTTON_ENG1".to_string(), 1.0)], "guard open: toggles on");
}
