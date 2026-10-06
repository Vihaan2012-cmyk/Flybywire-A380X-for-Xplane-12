//! The resolver on FlyByWire's own XML: excerpts copied verbatim from the
//! A380X package (behaviour/legacy/generated/A32NX_Interior_Generics.xml,
//! A32NX_Interior_Elec.xml, A32NX_Interior_Misc.xml, legacy/Airbus.xml,
//! pedestal/pedestal.xml, A380_COCKPIT.xml).

use std::collections::HashMap;

use super::bind::{self, Click, KEvents};
use super::expand;
use super::rpn;
use super::xml::Library;

const FBW_PUSH_TOGGLE: &str = r##"
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

            <Condition Check="MOMENTARY">
                <True>
                    <WWISE_EVENT_1>mpb1on</WWISE_EVENT_1>
                    <WWISE_EVENT_2>mpb1off</WWISE_EVENT_2>
                </True>
                <False>
                    <WWISE_EVENT_1>pb1on</WWISE_EVENT_1>
                    <WWISE_EVENT_2>pb1off</WWISE_EVENT_2>
                </False>
            </Condition>

            <NORMALIZED_TIME_1>0.1</NORMALIZED_TIME_1>
            <NORMALIZED_TIME_2>0.5</NORMALIZED_TIME_2>

            <Condition Check="DOWN_CODE">
                <True>
                    <STATUS>True</STATUS>
                </True>
                <False>
                    <STATUS>False</STATUS>
                </False>
            </Condition>

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
                            <Condition Check="DOWN_CODE">
                                <True>
                                    <DOWN_STATE_CODE>#DOWN_CODE#</DOWN_STATE_CODE>
                                </True>
                                <False>
                                    <DOWN_STATE_CODE>(L:XMLVAR_Momentary_#NODE_ID#_Pressed)</DOWN_STATE_CODE>
                                </False>
                            </Condition>
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
        </UseTemplate>
    </Template>

    <Template Name="FBW_Airbus_Battery_Master_Switch">
        <DefaultTemplateParameters>
            <ID>1</ID>
        </DefaultTemplateParameters>

        <UseTemplate Name="FBW_Push_Toggle">
            <LEFT_SINGLE_CODE>
                (L:A32NX_OVHD_ELEC_BAT_#ID#_PB_IS_AUTO, Bool) if{
                    0 (&gt;L:A32NX_OVHD_ELEC_BAT_#ID#_PB_IS_AUTO)
                } els{
                    1 (&gt;L:A32NX_OVHD_ELEC_BAT_#ID#_PB_IS_AUTO)
                }
            </LEFT_SINGLE_CODE>

            <SEQ1_CODE>(L:A32NX_OVHD_ELEC_BAT_#ID#_PB_HAS_FAULT, Bool)</SEQ1_CODE>
            <SEQ2_CODE>(L:A32NX_OVHD_ELEC_BAT_#ID#_PB_IS_AUTO, Bool) !</SEQ2_CODE>

            <DOWN_CODE>(L:A32NX_OVHD_ELEC_BAT_#ID#_PB_IS_AUTO, Bool)</DOWN_CODE>
        </UseTemplate>
    </Template>
"##;

// A32NX_Exterior.xml FBW_Airbus_Wiper (verbatim, lines 99-162): the wiper
// blade's own animation, off CIRCUIT POWER SETTING and a self-contained O:
// accumulator.
const FBW_AIRBUS_WIPER: &str = r##"
    <Template Name="FBW_Airbus_Wiper">
        <DefaultTemplateParameters>
            <ANIM_NAME>HANDLING_Wipers</ANIM_NAME>
            <CIRCUIT_ID_WIPERS>1</CIRCUIT_ID_WIPERS>
            <MIN_SPEED_PERCENT>0.1</MIN_SPEED_PERCENT>
            <MAX_SPEED>313</MAX_SPEED>
            <WWISE_EVENT_1>wipers_forward</WWISE_EVENT_1>
            <NORMALIZED_TIME_1>0.2</NORMALIZED_TIME_1>
            <WWISE_EVENT_2>wipers_backward</WWISE_EVENT_2>
            <NORMALIZED_TIME_2>0.8</NORMALIZED_TIME_2>
        </DefaultTemplateParameters>
        <OverrideTemplateParameters>
            <ONE_MINUS_MIN_SPEED Process="Float">1 #MIN_SPEED_PERCENT# -</ONE_MINUS_MIN_SPEED>
            <FAILURE_CODE>
                (O:AnimCode) 0 &gt; if{
                (A:CIRCUIT POWER SETTING:#CIRCUIT_ID_WIPERS#, percent over 100) 1 #MIN_SPEED_PERCENT# - * #MIN_SPEED_PERCENT# + #MAX_SPEED# * (&gt;O:_AnimSpeed)
                (O:_GoForward) if{ 1 (&gt;O:_AnimDirection) } els{ -1 (&gt;O:_AnimDirection) }
                (O:_AnimDirection) (O:_AnimSpeed) * (A:ANIMATION DELTA TIME, seconds) * (O:AnimCode) + (&gt;O:NewAnimCode)
                (O:NewAnimCode) 100 &gt; if{
                100 (O:NewAnimCode) 100 % - (&gt;O:NewAnimCode)
                1 (&gt;O:_ChangedDirection)
                }
                (O:NewAnimCode) 0 &lt; if{
                0 (&gt;O:NewAnimCode)
                }
                (O:_ChangedDirection) if{
                (O:_GoForward) ! (&gt;O:_GoForward)
                0 (&gt;O:_ChangedDirection)
                }
                (O:NewAnimCode) (&gt;O:AnimCode)
                }
                (O:AnimCode)
            </FAILURE_CODE>
        </OverrideTemplateParameters>
        <Update Once="True">
            0 (&gt;O:AnimCode)
        </Update>
        <UseTemplate Name="ASOBO_GT_Anim_Code">
            <ANIM_CODE>
                (A:CIRCUIT POWER SETTING:#CIRCUIT_ID_WIPERS#, percent over 100) #ONE_MINUS_MIN_SPEED# * #MIN_SPEED_PERCENT# + #MAX_SPEED# * (&gt;O:_AnimSpeed)
                (O:_GoForward) if{ 1 (&gt;O:_AnimDirection) } els{ -1 (&gt;O:_AnimDirection) }
                (O:_AnimDirection) (O:_AnimSpeed) * (A:ANIMATION DELTA TIME, seconds) * (O:AnimCode) + (&gt;O:NewAnimCode)
                (O:NewAnimCode) 105 &gt; if{
                105 (O:NewAnimCode) 105 % - (&gt;O:NewAnimCode)
                1 (&gt;O:_ChangedDirection)
                }
                (O:NewAnimCode) 40 &lt; (O:_GoForward) ! and if{
                (O:NewAnimCode) abs 80 % (&gt;O:NewAnimCode)
                1 (&gt;O:_ChangedDirection)
                }
                (O:NewAnimCode) 0 &lt; if{
                (O:NewAnimCode) abs 100 % (&gt;O:NewAnimCode)
                1 (&gt;O:_ChangedDirection)
                }
                (O:_ChangedDirection) if{
                (O:_GoForward) ! (&gt;O:_GoForward)
                0 (&gt;O:_ChangedDirection)
                }
                (O:NewAnimCode)
            </ANIM_CODE>
            <FAILURE>(A:CIRCUIT ON:#CIRCUIT_ID_WIPERS#, Bool)</FAILURE>
        </UseTemplate>
        <UseTemplate Name="ASOBO_GT_AnimTriggers_2SoundEvents"/>
    </Template>
"##;

// A32NX_Interior_Misc.xml FBW_Airbus_Wiper_Knob (verbatim, lines 394-433): the
// knob that actually sets CIRCUIT POWER SETTING -- the same variable the
// blade's own ANIM_CODE reads.
const FBW_AIRBUS_WIPER_KNOB: &str = r##"
    <Template Name="FBW_Airbus_Wiper_Knob">
        <DefaultTemplateParameters>
            <NODE_ID>HANDLING_Switch_Wipers</NODE_ID>
            <ANIM_NAME>HANDLING_Switch_Wipers</ANIM_NAME>
            <CIRCUIT_ID_WIPERS>1</CIRCUIT_ID_WIPERS>
            <WWISE_EVENT>turnknob</WWISE_EVENT>
            <POWER_NORM>100</POWER_NORM>
            <POWER_SLOW>75</POWER_SLOW>
            <ANIMTIP_0>TT:COCKPIT.TOOLTIPS.WIPERS_SET_OFF</ANIMTIP_0>
            <ANIMTIP_1>TT:COCKPIT.TOOLTIPS.WIPERS_SET_SLOW</ANIMTIP_1>
            <ANIMTIP_2>TT:COCKPIT.TOOLTIPS.WIPERS_SET_FAST</ANIMTIP_2>
        </DefaultTemplateParameters>
        <OverrideTemplateParameters>
            <SET_POWER>#CIRCUIT_ID_WIPERS# (&gt;K:2:ELECTRICAL_CIRCUIT_POWER_SETTING_SET)</SET_POWER>
            <GET_POWER>(A:CIRCUIT POWER SETTING:#CIRCUIT_ID_WIPERS#, Percent)</GET_POWER>
            <INIT_WIPERS_STATE>
                (A:CIRCUIT SWITCH ON:#CIRCUIT_ID_WIPERS#, Bool) if{ #CIRCUIT_ID_WIPERS# (&gt;K:ELECTRICAL_CIRCUIT_TOGGLE) }
            </INIT_WIPERS_STATE>
            <WIPERS_ON>(A:CIRCUIT SWITCH ON:#CIRCUIT_ID_WIPERS#, Bool) ! if{ #CIRCUIT_ID_WIPERS# (&gt;K:ELECTRICAL_CIRCUIT_TOGGLE) }</WIPERS_ON>
            <WIPERS_OFF>(A:CIRCUIT SWITCH ON:#CIRCUIT_ID_WIPERS#, Bool) if{ #CIRCUIT_ID_WIPERS# (&gt;K:ELECTRICAL_CIRCUIT_TOGGLE) }</WIPERS_OFF>
            <IS_WIPERS_ON>(A:CIRCUIT SWITCH ON:#CIRCUIT_ID_WIPERS#, Bool)</IS_WIPERS_ON>
        </OverrideTemplateParameters>

        <Component ID="#NODE_ID#" Node="#NODE_ID#">
            <UseTemplate Name="ASOBO_GT_Update">
                <UPDATE_ONCE>True</UPDATE_ONCE>
                <UPDATE_CODE>#INIT_WIPERS_STATE#</UPDATE_CODE>
            </UseTemplate>

            <UseTemplate Name="ASOBO_GT_Switch_3States">
                <CODE_POS_0>#WIPERS_OFF#</CODE_POS_0>
                <CODE_POS_1>#WIPERS_ON# #POWER_SLOW# #SET_POWER#</CODE_POS_1>
                <CODE_POS_2>#WIPERS_ON# #POWER_NORM# #SET_POWER#</CODE_POS_2>
                <STATE0_TEST>#IS_WIPERS_ON# !</STATE0_TEST>
                <STATE1_TEST>#IS_WIPERS_ON# #GET_POWER# #POWER_SLOW# == and</STATE1_TEST>
                <STATE2_TEST>#IS_WIPERS_ON# #GET_POWER# #POWER_NORM# == and</STATE2_TEST>
                <SWITCH_DIRECTION>Horizontal</SWITCH_DIRECTION>
                <ARROW_TYPE>Curved</ARROW_TYPE>
            </UseTemplate>
        </Component>
    </Template>
"##;

const FBW_ANIM_INTERACTIONS: &str = r##"
    <Template Name="FBW_Anim_Interactions">
        <Parameters Type="Default">
            <Switch Param="ANIM_TYPE">
                <Case Value="KNOB">
                    <ANIM_LAG>1000</ANIM_LAG>
                </Case>
                <Case Value="SWITCH">
                    <ANIM_LAG>1000</ANIM_LAG>
                </Case>
            </Switch>
        </Parameters>
        <UseTemplate Name="#ANIM_TEMPLATE#"/>
    </Template>
"##;

const ENGINE_MASTER: &str = r##"
    <Template Name="FBW_ENGINE_Switch_Master_Template">
        <DefaultTemplateParameters>
            <NODE_ID>ENGINE_Switch_Master_#ID#</NODE_ID>
            <ANIM_NAME>ENGINE_Switch_Master_#ID#</ANIM_NAME>
            <WWISE_EVENT_1>engmaster</WWISE_EVENT_1>
            <WWISE_EVENT_2>engmaster</WWISE_EVENT_2>
            <PART_ID>ENGINE_Switch_Master</PART_ID>
            <ANIMTIP_0>TT:COCKPIT.TOOLTIPS.ENGINE#ID#_MASTER_TURN_ON</ANIMTIP_0>
            <ANIMTIP_1>TT:COCKPIT.TOOLTIPS.ENGINE#ID#_MASTER_TURN_OFF</ANIMTIP_1>
        </DefaultTemplateParameters>
        <Component ID="#NODE_ID#" Node="#NODE_ID#">
            <Update Frequency="5">
                (* This Update Ensures that the state of some simvars that are linked to the same button is consistent. *)
                (A:FUELSYSTEM VALVE SWITCH:#VALVE_ID#, Bool) sp1
                (A:GENERAL ENG STARTER:#ID#, Bool) l1 != if{
                    (&gt;K:TOGGLE_STARTER#ID#)
                }
            </Update>
            <UseTemplate Name="ASOBO_GT_Switch_Code">
                <ANIM_CODE>
                    (A:FUELSYSTEM VALVE SWITCH:#VALVE_ID#, Bool) 100 *
                </ANIM_CODE>
                <LEFT_SINGLE_CODE>
                    (A:FUELSYSTEM VALVE SWITCH:#VALVE_ID#, Bool) if{
                        #VALVE_ID# (&gt;K:FUELSYSTEM_VALVE_CLOSE)
                        (A:GENERAL ENG STARTER:#ID#, Bool) if{
                            (&gt;K:TOGGLE_STARTER#ID#)
                        }
                    } els{
                        #VALVE_ID# (&gt;K:FUELSYSTEM_VALVE_OPEN)
                        (A:GENERAL ENG STARTER:#ID#, Bool) ! if{
                            (&gt;K:TOGGLE_STARTER#ID#)
                        }
                    }
                </LEFT_SINGLE_CODE>
            </UseTemplate>
        </Component>
    </Template>
"##;

const ENGINE_MODE: &str = r##"
    <Template Name="A32NX_ENGINE_MODE_SELECTOR_TEMPLATE">
        <DefaultTemplateParameters>
            <ENGINE_COUNT>2</ENGINE_COUNT>
        </DefaultTemplateParameters>
        <UseTemplate Name="A32NX_ENGINE_MODE_SELECTOR_SUBTEMPLATE">
            <ENGINE_CURRENT>#ENGINE_COUNT#</ENGINE_CURRENT>
        </UseTemplate>
    </Template>

    <Template Name="A32NX_ENGINE_MODE_SELECTOR_SUBTEMPLATE">
        <DefaultTemplateParameters>
            <NODE_ID>ENGINE_Switch_Engine_Mode</NODE_ID>
            <ANIM_NAME>ENGINE_Switch_Engine_Mode</ANIM_NAME>
            <PART_ID>ENGINE_Switch_Engine_Mode</PART_ID>
            <SWITCH_DIRECTION>Horizontal</SWITCH_DIRECTION>
            <ARROW_TYPE>Curved</ARROW_TYPE>
            <CODE_POS_1></CODE_POS_1>
            <CODE_POS_2></CODE_POS_2>
            <SWITCH_POSITION_TYPE>L</SWITCH_POSITION_TYPE>
            <SWITCH_POSITION_VAR>XMLVAR_ENG_MODE_SEL</SWITCH_POSITION_VAR>
            <STATE0_TEST>1</STATE0_TEST>
            <STATE1_TEST>1</STATE1_TEST>
            <STATE2_TEST>1</STATE2_TEST>
            <WWISE_EVENT>turnknob</WWISE_EVENT>
        </DefaultTemplateParameters>
        <Condition>
            <Test>
                <Greater>
                    <Value>ENGINE_CURRENT</Value>
                    <Number>0</Number>
                </Greater>
            </Test>
            <True>
                <UseTemplate Name="A32NX_ENGINE_MODE_SELECTOR_SUBTEMPLATE">
                    <CODE_POS_1>
                        1 (&gt;K:TURBINE_IGNITION_SWITCH_SET#ENGINE_CURRENT#)
                        #CODE_POS_1#
                    </CODE_POS_1>
                    <CODE_POS_2>
                        2 (&gt;K:TURBINE_IGNITION_SWITCH_SET#ENGINE_CURRENT#)
                        #CODE_POS_2#
                    </CODE_POS_2>

                    <STATE0_TEST> (A:TURB ENG IGNITION SWITCH EX1:#ENGINE_CURRENT#, Enum) 0 ==
                        #STATE0_TEST# and</STATE0_TEST>
                    <STATE1_TEST> (A:TURB ENG IGNITION SWITCH EX1:#ENGINE_CURRENT#, Enum) 1 ==
                        #STATE1_TEST# and</STATE1_TEST>
                    <STATE2_TEST> (A:TURB ENG IGNITION SWITCH EX1:#ENGINE_CURRENT#, Enum) 2 ==
                        #STATE2_TEST# and</STATE2_TEST>
                    <ENGINE_CURRENT Process="Int">#ENGINE_CURRENT# 1 -</ENGINE_CURRENT>
                </UseTemplate>
            </True>
            <False>
                <Component ID="#NODE_ID#" Node="#NODE_ID#">
                    <UseTemplate Name="ASOBO_GT_Switch_3States">
                        <CODE_POS_0>
                            0 (&gt;K:TURBINE_IGNITION_SWITCH_SET)
                        </CODE_POS_0>
                        <CODE_POS_2>
                            #CODE_POS_2#
                            (&gt;H:A320_Neo_EICAS_2_Ignition_Start)
                        </CODE_POS_2>
                    </UseTemplate>
                </Component>
            </False>
        </Condition>
    </Template>
"##;

fn resolve(templates: &[&str], components: &str) -> bind::Resolution {
    let text = format!("<ModelBehaviors>{}{components}</ModelBehaviors>", templates.concat());
    let lib = Library::from_text(&text).unwrap();
    bind::resolve_all(&expand::expand(&lib).leaves)
}

#[test]
fn battery_pushbutton_toggles_its_auto_variable() {
    // A380_COCKPIT.xml, Overhead_Electricals.
    let r = resolve(
        &[FBW_PUSH_TOGGLE],
        r#"<Component ID="Overhead_Electricals">
                <DefaultTemplateParameters><TYPE>AIRBUS</TYPE></DefaultTemplateParameters>
                <UseTemplate Name="FBW_Airbus_Battery_Master_Switch">
                    <NODE_ID>PUSH_OVHD_ELEC_BAT1</NODE_ID>
                    <PART_ID>BATTERY_MASTER_SWITCH_1</PART_ID>
                    <ID>1</ID>
                    <SEQ_POWERED>(L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED, Bool)</SEQ_POWERED>
                </UseTemplate>
            </Component>"#,
    );
    let b = r.binding("PUSH_OVHD_ELEC_BAT1").expect("bound");
    assert_eq!(b.click, Click::Toggle { dref: "fbw/A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO".into(), on: 1.0, off: 0.0 });
    assert_eq!(b.node, "PUSH_OVHD_ELEC_BAT1");
    // Pressed in while AUTO: the animation follows the variable.
    assert!(b.look.as_deref().is_some_and(|l| l.contains("fbw/A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO")), "{:?}", b.look);
}

#[test]
fn wiper_blade_animation_becomes_a_mirror_of_the_knobs_power_setting() {
    // A380_Cockpit_Behavior.xml:135-144's WipersLeft Component, and the
    // real HANDLING_Switch_Wiper_left knob, both on circuit 141.
    let r = resolve(
        &[FBW_AIRBUS_WIPER, FBW_AIRBUS_WIPER_KNOB],
        r#"<Component ID="WipersLeft">
                <UseTemplate Name="FBW_Airbus_Wiper">
                    <ANIM_NAME>HANDLING_Wiper_left</ANIM_NAME>
                    <CIRCUIT_ID_WIPERS>141</CIRCUIT_ID_WIPERS>
                </UseTemplate>
            </Component>
            <Component ID="HANDLING_Switch_Wiper_left" Node="HANDLING_Switch_Wiper_left">
                <UseTemplate Name="FBW_Airbus_Wiper_Knob">
                    <NODE_ID>HANDLING_Switch_Wiper_left</NODE_ID>
                    <ANIM_NAME>HANDLING_Switch_Wiper_left</ANIM_NAME>
                    <CIRCUIT_ID_WIPERS>141</CIRCUIT_ID_WIPERS>
                </UseTemplate>
            </Component>"#,
    );
    // The knob is a real, bound control that writes CIRCUIT POWER SETTING...
    let knob = r.binding("HANDLING_Switch_Wiper_left").unwrap_or_else(|| panic!("knob unresolved: {:?}", r.unresolved));
    assert!(knob.targets.contains("fbw/CIRCUIT_POWER_SETTING_141"), "{:?}", knob.targets);
    // ...which is exactly what lets the blade's own ANIM_CODE (self-contained
    // O: locals, `A:ANIMATION DELTA TIME`, no click of its own) qualify as a
    // mirror instead of being dropped.
    let (_, lua) = r.mirrors.iter().find(|(a, _)| a == "handling_wiper_left").unwrap_or_else(|| panic!("no mirror for the wiper blade: {:?}", r.mirrors));
    assert!(lua.contains("\"fbw/CIRCUIT_POWER_SETTING_141\""), "{lua}");
}

/// Run a bound control's MSFS code on a state, as the Lua does.
fn run(code: &str, state: &[(&str, f64)]) -> HashMap<String, f64> {
    let st: HashMap<String, f64> = state.iter().map(|(k, v)| (k.to_string(), *v)).collect();
    let r = rpn::run(code, &st, &KEvents);
    let mut out = st;
    out.extend(r.writes);
    out
}

#[test]
fn engine_master_opens_the_fuel_valve_and_sets_the_starter() {
    // pedestal.xml, ENG MASTER 2, and the template's own default names.
    let r = resolve(
        &[FBW_ANIM_INTERACTIONS, ENGINE_MASTER],
        r#"<Component ID="Engines">
                <UseTemplate Name="FBW_Anim_Interactions">
                    <ANIM_TYPE>SWITCH</ANIM_TYPE>
                    <ANIM_TEMPLATE>FBW_ENGINE_Switch_Master_Template</ANIM_TEMPLATE>
                    <AIRBUS_TYPE />
                    <ANIM_NAME>SWITCH_ENGINES_ENG2</ANIM_NAME>
                    <ANIM_LAG>250</ANIM_LAG>
                    <NODE_ID>SWITCH_ENGINES_ENG2</NODE_ID>
                    <ID>2</ID>
                    <VALVE_ID>2</VALVE_ID>
                    <POTENTIOMETER>9999</POTENTIOMETER>
                </UseTemplate>
                <UseTemplate Name="FBW_ENGINE_Switch_Master_Template">
                    <ID>3</ID>
                    <VALVE_ID>3</VALVE_ID>
                </UseTemplate>
            </Component>"#,
    );
    for (anim, n) in [("SWITCH_ENGINES_ENG2", 2), ("ENGINE_Switch_Master_3", 3)] {
        let b = r.binding(anim).unwrap_or_else(|| panic!("{anim} bound: {:?}", r.unresolved));
        let starter = format!("fbw/GENERAL_ENG_STARTER_{n}");
        let valve = format!("fbw/FUELSYSTEM_VALVE_SWITCH_{n}");
        assert!(b.targets.contains(&starter) && b.targets.contains(&valve), "{:?}", b.targets);
        // Two variables: SASL runs the code.
        let Click::Script(sc) = &b.click else { panic!("{anim}: {:?}", b.click) };
        let lua = sc.press.as_deref().unwrap();
        assert!(lua.contains(&format!("wr(\"{starter}\"")) && lua.contains(&format!("\"fbw/FUELSYSTEM_VALVE_SWITCH_\" .. string.format")), "{lua}");
        // The animation shows the valve (on = 1).
        assert!(b.look.as_deref().is_some_and(|l| l.contains(&valve)));
    }
    // What the code does: master ON opens the valve and starts; OFF closes and stops.
    let code = "(A:FUELSYSTEM VALVE SWITCH:2, Bool) if{ 2 (>K:FUELSYSTEM_VALVE_CLOSE) (A:GENERAL ENG STARTER:2, Bool) if{ (>K:TOGGLE_STARTER2) } } els{ 2 (>K:FUELSYSTEM_VALVE_OPEN) (A:GENERAL ENG STARTER:2, Bool) ! if{ (>K:TOGGLE_STARTER2) } }";
    let on = run(code, &[]);
    assert_eq!((on["A:FUELSYSTEM VALVE SWITCH:2"], on["A:GENERAL ENG STARTER:2"]), (1.0, 1.0));
    let off = run(code, &[("A:FUELSYSTEM VALVE SWITCH:2", 1.0), ("A:GENERAL ENG STARTER:2", 1.0)]);
    assert_eq!((off["A:FUELSYSTEM VALVE SWITCH:2"], off["A:GENERAL ENG STARTER:2"]), (0.0, 0.0));
}

#[test]
fn engine_mode_knob_sets_ignition_on_all_four_engines() {
    // pedestal.xml, the engine start selector.
    let r = resolve(
        &[FBW_ANIM_INTERACTIONS, ENGINE_MODE],
        r#"<Component ID="Engines">
                <UseTemplate Name="FBW_Anim_Interactions">
                    <ANIM_TYPE>KNOB</ANIM_TYPE>
                    <ANIM_TEMPLATE>A32NX_ENGINE_MODE_SELECTOR_TEMPLATE</ANIM_TEMPLATE>
                    <ANIM_NAME>KNOB_ENGINES_MODE</ANIM_NAME>
                    <NODE_ID>KNOB_ENGINES_MODE</NODE_ID>
                    <ENGINE_COUNT>4</ENGINE_COUNT>
                    <ID>0</ID>
                    <ANIMREF_ID>0</ANIMREF_ID>
                    <ANIMTIP_0_ON_PERCENT>0</ANIMTIP_0_ON_PERCENT>
                    <ANIMTIP_1_ON_PERCENT>.5</ANIMTIP_1_ON_PERCENT>
                    <ANIMTIP_2_ON_PERCENT>1</ANIMTIP_2_ON_PERCENT>
                </UseTemplate>
            </Component>"#,
    );
    let b = r.binding("KNOB_ENGINES_MODE").unwrap_or_else(|| panic!("bound: {:?}", r.unresolved));
    let mut want: Vec<String> = (1..=4).map(|e| format!("fbw/TURB_ENG_IGNITION_SWITCH_EX1_{e}")).collect();
    // The knob's own position, published too (home()'s XMLVAR_ENG_MODE_SEL
    // allowlist): FwsSystemDisplayLogic.ts's checkEnginePage reads it by
    // this exact name to auto-select the SD ENG page on start/crank.
    want.push("fbw/XMLVAR_ENG_MODE_SEL".to_string());
    assert_eq!(b.targets.iter().cloned().collect::<Vec<_>>(), want);
    let Click::Script(sc) = &b.click else { panic!("{:?}", b.click) };
    assert!(sc.up.is_some() && sc.down.is_some() && sc.horizontal);
    // Each detent's code, as the template builds it: 0/1/2 on every engine.
    let lib = Library::from_text(&format!(
        "<ModelBehaviors>{FBW_ANIM_INTERACTIONS}{ENGINE_MODE}<Component ID=\"e\"><UseTemplate Name=\"A32NX_ENGINE_MODE_SELECTOR_TEMPLATE\"><ANIM_NAME>KNOB_ENGINES_MODE</ANIM_NAME><NODE_ID>KNOB_ENGINES_MODE</NODE_ID><ENGINE_COUNT>4</ENGINE_COUNT></UseTemplate></Component></ModelBehaviors>"
    ))
    .unwrap();
    let leaf = expand::expand(&lib).leaves.into_iter().find(|l| l.template == "ASOBO_GT_Switch_3States").unwrap();
    for (pos, v) in [(0, 0.0), (1, 1.0), (2, 2.0)] {
        let out = run(leaf.get(&format!("CODE_POS_{pos}")).unwrap(), &[]);
        for e in 1..=4 {
            assert_eq!(out[&format!("A:TURB ENG IGNITION SWITCH EX1:{e}")], v, "position {pos}, engine {e}");
        }
    }
    // The knob shows NORM (the middle, 0.5) when every engine is at 1.
    let lua = b.look.as_deref().unwrap();
    assert!(lua.contains("TURB_ENG_IGNITION_SWITCH_EX1_4") && lua.contains("/ 200"), "{lua}");
    let state = leaf.get("STATE1_TEST").unwrap();
    let st: Vec<(String, f64)> = (1..=4).map(|e| (format!("A:TURB ENG IGNITION SWITCH EX1:{e}"), 1.0)).collect();
    let st: HashMap<String, f64> = st.into_iter().collect();
    assert_eq!(rpn::run(state, &st, &KEvents).stack.last(), Some(&rpn::Val::Num(1.0)));
}

#[test]
fn rpn_translates_to_lua() {
    let lua = bind::to_lua("(L:A32NX_X, Bool) ! (>L:A32NX_X) 92 50 (>K:2:LIGHT_POTENTIOMETER_SET)", "c").unwrap();
    // An L: variable reads as it holds its value, whatever unit is named.
    assert!(lua.contains("P(s, rd(\"fbw/A32NX_X\"))"), "{lua}");
    assert!(lua.contains("wr(\"fbw/A32NX_X\", Q(s))"), "{lua}");
    assert!(lua.contains("\"fbw/LIGHT_POTENTIOMETER_\" .. string.format(\"%d\", a0)"), "{lua}");
    // The stock Asobo altitude knob's absolute-value write: routed to the
    // port's own pending dataref (prim.rs TrimPulses-style one-shot input;
    // no plugin command can carry a float argument), not left unresolved.
    let alt = bind::to_lua("(>K:AP_ALT_VAR_SET_ENGLISH)", "c").unwrap();
    assert!(alt.contains("wr(\"fbw/XP_FCU_ALT_SET_PENDING\"") && alt.contains("a0"), "{alt}");
    assert!(bind::to_lua("(>K:AP_MASTER)", "c").unwrap().contains("CMD(\"fbw/event/AP_MASTER\")"));
    assert!(bind::to_lua("(#TOGGLE_SIMVAR#_LOCK) ! (>#TOGGLE_SIMVAR#_LOCK)", "c").is_err());
}

/// W196: FBW_Airbus_Sidestick_Priority's press code is compound (an
/// AP-disconnect K: event plus the priority variable) and its release code
/// only clears the variable; home()'s generic "L" case would sanitize the
/// colon in `A32NX_PRIORITY_TAKEOVER:1`/`:2` into a dataref
/// (`fbw/A32NX_PRIORITY_TAKEOVER_1`/`_2`) nothing reads. The plugin's own
/// republish of the colon form every tick (lib.rs:1523-1533) overwrites it
/// regardless of name; its real input is the begin/end phase of its own
/// command (afs_events::PriorityTakeoverCommands). Both IDs must fire their
/// own command, and the existing AP-disconnect command must survive.
#[test]
fn sidestick_priority_takeover_fires_the_plugins_held_command() {
    let press = "(>K:AUTOPILOT_OFF) 1 (>L:A32NX_PRIORITY_TAKEOVER:1)";
    let press_lua = bind::to_lua(press, "c").unwrap();
    assert!(press_lua.contains("CMD(\"fbw/event/AUTOPILOT_OFF\")"), "{press_lua}");
    assert!(press_lua.contains("CMD_HELD(\"fbw/event/A32NX_PRIORITY_TAKEOVER_CAPT\", Q(s))"), "{press_lua}");
    let release_lua = bind::to_lua("0 (>L:A32NX_PRIORITY_TAKEOVER:1)", "c").unwrap();
    assert!(release_lua.contains("CMD_HELD(\"fbw/event/A32NX_PRIORITY_TAKEOVER_CAPT\", Q(s))"), "{release_lua}");
    // The first officer's side (ID 2) fires the other command.
    let fo = bind::to_lua("1 (>L:A32NX_PRIORITY_TAKEOVER:2)", "c").unwrap();
    assert!(fo.contains("CMD_HELD(\"fbw/event/A32NX_PRIORITY_TAKEOVER_FO\", Q(s))"), "{fo}");
}

/// CTRL-007: the parking brake lever's real click code
/// (FBW_LANDING_GEAR_Switch_ParkingBrake_SubTemplate,
/// A32NX_Interior_Handling.xml) wraps `(>K:PARKING_BRAKES)` in MSFS's
/// `(M:Event) 'WheelUp' scmi 0 == if{...} els{...}` mouse-wheel dispatch,
/// which used to fail to translate at all ("reads M:Event"); M:/R:/S: reads
/// now resolve to an empty string (as rpn.rs's own interpreter already
/// treats them), so the wheel branches never match and the plain click
/// toggles the real dataref FBW's handling.rs reads.
#[test]
fn parking_brake_lever_falls_through_msfs_mousewheel_dispatch_to_the_real_toggle() {
    let rpn = "(M:Event) 'WheelUp' scmi 0 == if{ (>K:PARKING_BRAKES) } els{ \
               (M:Event) 'WheelDown' scmi 0 == if{ (>K:PARKING_BRAKES) } els{ (>K:PARKING_BRAKES) } }";
    let lua = bind::to_lua(rpn, "c").unwrap();
    assert!(lua.contains("wr(\"fbw/A32NX_PARK_BRAKE_LEVER_POS\""), "{lua}");
}

/// CTRL-004/LGT-003: FBW_A380X_BacklightIndicator_Button_Template
/// (generic/buttons.xml) has no default for INDICATOR_POWERED or
/// INDICATOR_CODE; every caller that does not override them (the SURV
/// panel's 8 buttons, EFIS CS/FO BLANK and TAXI) used to fail to resolve at
/// all ("template parameter never given").
#[test]
fn backlight_indicator_template_defaults_fill_in_for_the_missing_params() {
    assert!(bind::to_lua("#INDICATOR_POWERED# if{ 1 (>L:A32NX_X) }", "c").unwrap().contains("P(s, 1)"));
    assert_eq!(bind::to_lua("#INDICATOR_CODE#", "c").unwrap(), "  P(s, 0)\n");
}

/// A "template parameter never given" reason now names exactly the missing
/// parameter, not 40 raw characters of whatever RPN happened to follow it.
#[test]
fn unresolved_template_parameter_reason_names_the_missing_parameter_only() {
    let err = bind::to_lua("#SOME_UNGIVEN_PARAM# 1 ==", "c").unwrap_err();
    assert_eq!(err, "template parameter never given: #SOME_UNGIVEN_PARAM#");
}

const W14_WRAPPERS: &str = r##"
    <Template Name="W14_OUTER_WRAPPER">
        <UseTemplate Name="W14_INNER_WRAPPER">
            <NODE_ID>#NODE_ID#</NODE_ID>
        </UseTemplate>
    </Template>
    <Template Name="W14_INNER_WRAPPER">
        <UseTemplate Name="ASOBO_GT_Push_Button">
            <NODE_ID>#NODE_ID#</NODE_ID>
            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
            <LEFT_SINGLE_CODE>#W14_MISSING_PARAM# (&gt;L:A32NX_X)</LEFT_SINGLE_CODE>
        </UseTemplate>
    </Template>
"##;

/// An unresolved control now carries the `UseTemplate` chain that reached
/// its leaf template (expand::Leaf::chain), not just the leaf's own name:
/// here the leaf is Asobo's own `ASOBO_GT_Push_Button`, but the parameter
/// that was never given belongs to FBW's own two wrapper templates around
/// it, which is what a fix author actually needs to know.
#[test]
fn unresolved_reason_names_the_parameter_and_keeps_the_usetemplate_chain() {
    let r = resolve(
        &[W14_WRAPPERS],
        r#"<Component ID="W14_Test">
                <UseTemplate Name="W14_OUTER_WRAPPER">
                    <NODE_ID>W14_TEST_BTN</NODE_ID>
                </UseTemplate>
            </Component>"#,
    );
    let u = r.unresolved.iter().find(|u| u.anim == "W14_TEST_BTN").unwrap_or_else(|| panic!("{:?}", r.unresolved));
    assert_eq!(u.reason, "template parameter never given: #W14_MISSING_PARAM#");
    assert_eq!(u.chain, vec!["W14_OUTER_WRAPPER".to_string(), "W14_INNER_WRAPPER".to_string()]);
}
