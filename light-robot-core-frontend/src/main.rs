mod components;

use crate::components::*;
use gloo::timers::callback::Timeout;
use light_robot_core_api::*;
use light_robot_core_flight_control::PidControlSystem;
use light_robot_core_simulation::{
    Environment, NumericalSimulation, Quat, RocketParameters, RocketState, SimulationLog, Vec3,
};
use material_yew::{MatTab, MatTabBar};
use web_sys::{HtmlInputElement, HtmlSelectElement};
use yew::prelude::*;
use yew_hooks::prelude::*;

const TEST_COLORS: [&str; 5] = ["#1565c0", "#d32f2f", "#2e7d32", "#7b1fa2", "#ef6c00"];
const TILT_GRAPH_LIMIT_DEGREES: f32 = 10.0;

#[derive(Clone, PartialEq)]
struct AppConnection {
    state: UseStateHandle<State>,
    message: UseStateHandle<Option<String>>,
    send_command: Callback<Command>,
}

fn send_command(sender: &Callback<Command>, command: Command) {
    sender.emit(command);
}

#[derive(Clone, PartialEq)]
struct StoredTest {
    result: ServoTestResult,
    visible: bool,
    color: &'static str,
}

#[derive(Clone, PartialEq)]
struct ConfigDraft {
    zero: String,
    turn: String,
    p: String,
    i: String,
    d: String,
    limit: String,
}

impl ConfigDraft {
    fn from_config(config: &ServoConfiguration) -> Self {
        Self {
            zero: config.zero_encoder_count.to_string(),
            turn: config.max_turn_degrees.to_string(),
            p: config.position_p.to_string(),
            i: config.position_i.to_string(),
            d: config.position_d.to_string(),
            limit: config.duty_cycle_limit.to_string(),
        }
    }
}

fn axis_name(axis: ServoAxis) -> &'static str {
    match axis {
        ServoAxis::X => "X axis",
        ServoAxis::Y => "Y axis",
    }
}

fn device_name(device: &ServoDeviceId) -> String {
    let id1 = device
        .chip_id1
        .iter()
        .map(|byte| format!("{:02X}", byte))
        .collect::<Vec<_>>()
        .join("");
    let id2 = device
        .chip_id2
        .iter()
        .map(|byte| format!("{:02X}", byte))
        .collect::<Vec<_>>()
        .join("");
    format!("{} / {}", id1, id2)
}

const PRELAUNCH_STEPS: [(PrelaunchChecklistStep, &str); 8] = [
    (PrelaunchChecklistStep::ConfigureYServo, "Configure Y servo"),
    (PrelaunchChecklistStep::ConfigureXServo, "Configure X servo"),
    (PrelaunchChecklistStep::CheckTvcDirections, "Check TVC axis directions"),
    (PrelaunchChecklistStep::CheckParachuteConnection, "Check parachute on PYR1 only"),
    (PrelaunchChecklistStep::ConfigureAndTestParachute, "Configure and test parachute activation"),
    (PrelaunchChecklistStep::ConfigureAndCheckIgniter, "Configure igniter timing and conductivity"),
    (PrelaunchChecklistStep::MeasureMomentsOfInertia, "Measure moments of inertia"),
    (PrelaunchChecklistStep::RunSimulationAndSavePid, "Run simulation/HIL and save PID"),
];

#[function_component]
fn PrelaunchChecklist() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let checklist = connection.state.prelaunch_checklist.clone();
    let socket = connection.send_command.clone();
    let start = { let socket = socket.clone(); Callback::from(move |_| send_command(&socket, Command::StartPrelaunchChecklist)) };
    let abort = { let socket = socket.clone(); Callback::from(move |_| send_command(&socket, Command::AbortPrelaunchChecklist)) };
    html! { <Card title="pre-launch checklist" icon="fact_check">
        if !checklist.active {
            <p>{"Saved sign-offs lock completed settings."}</p>
            <button onclick={start}>{"Start"}</button>
        } else {
            <p>{format!("{} of {} signed off", checklist.completed_steps, PrelaunchChecklistStep::COUNT)}</p>
            <ol class="checklist">
            {for PRELAUNCH_STEPS.iter().map(|(step, label)| {
                let index = step.index();
                let signed = index < checklist.completed_steps;
                let current = index == checklist.completed_steps;
                let socket = socket.clone();
                let return_step = *step;
                let return_to = Callback::from(move |_| send_command(&socket, Command::ReturnToPrelaunchStep { step: return_step }));
                html! { <li class={if signed {"signed-off"} else if current {"current-step"} else {"pending-step"}}>
                    <span>{*label}</span>
                    if signed { <><strong>{"Done"}</strong><button class="danger" onclick={return_to}>{"Redo from here"}</button></> }
                    else if current { <small>{"Current"}</small> }
                    else { <small>{"Waiting"}</small> }
                </li> }
            })}
            </ol>
            <p class="safety-note">{"Redo clears this step and later steps."}</p>
            <button class="danger" onclick={abort}>{"Abort"}</button>
        }
    </Card> }
}

fn guided_step_copy(step: u8) -> (&'static str, &'static str, PrelaunchChecklistStep) {
    match step {
        0 => ("Configure Y servo", "Set up the Y axis.", PrelaunchChecklistStep::ConfigureYServo),
        1 => ("Configure X servo", "Set up the X axis.", PrelaunchChecklistStep::ConfigureXServo),
        2 => ("Check TVC directions", "Verify both axes move correctly.", PrelaunchChecklistStep::CheckTvcDirections),
        3 => ("Check parachute connection", "PYR1: continuity. PYR2: open.", PrelaunchChecklistStep::CheckParachuteConnection),
        4 => ("Test parachute", "Set duration; test PYR1.", PrelaunchChecklistStep::ConfigureAndTestParachute),
        5 => ("Configure igniter", "Set duration; test PYR2 with wire only.", PrelaunchChecklistStep::ConfigureAndCheckIgniter),
        6 => ("Measure inertia", "Capture and save inertia.", PrelaunchChecklistStep::MeasureMomentsOfInertia),
        _ => ("Simulation and PID", "Run simulation or HIL; save PID.", PrelaunchChecklistStep::RunSimulationAndSavePid),
    }
}

#[function_component]
fn GuidedOnboarding() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let completed = connection.state.prelaunch_checklist.completed_steps;
    if completed >= PrelaunchChecklistStep::COUNT {
        return html! { <Card title="checklist complete" icon="task_alt"><p>{"8 / 8 signed off and saved."}</p></Card> };
    }
    let (title, instruction, step) = guided_step_copy(completed);
    let socket = connection.send_command.clone();
    let sign = Callback::from(move |_| send_command(&socket, Command::SignOffPrelaunchStep { step }));
    html! { <>
        <Card title={format!("Step {} of {} — {}", completed + 1, PrelaunchChecklistStep::COUNT, title)} icon="directions_run">
            <p>{instruction}</p>
        </Card>
        if completed == 0 || completed == 1 { <ServoCalibration/> }
        else if completed == 2 { <ServoOrientationCheck/> }
        else if completed == 3 || completed == 4 || completed == 5 { <PyroConfigurationPage/> }
        else if completed == 6 { <MomentOfInertia/> }
        else { <RocketSimulation/> }
        <Card title="sign-off" icon="task_alt"><button class="save-button" onclick={sign}>{"Done — next step"}</button></Card>
    </> }
}

#[function_component]
fn PyroConfigurationPage() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let socket = connection.send_command.clone();
    let configuration = use_state_eq(|| state.pyro_configuration.clone());
    let parachute_locked = state.prelaunch_checklist.completed_steps > 4;
    let igniter_locked = state.prelaunch_checklist.completed_steps > 5;
    let pyro2_test_allowed = state.prelaunch_checklist.active && state.prelaunch_checklist.completed_steps == 5;
    let update = |channel: u8| { let configuration = configuration.clone(); Callback::from(move |value: f32| {
        let mut next = (*configuration).clone();
        if channel == 1 { next.parachute_duration_ms = value.round().clamp(1.0, 10_000.0) as u16; } else { next.igniter_duration_ms = value.round().clamp(1.0, 10_000.0) as u16; }
        configuration.set(next);
    })};
    let save = { let socket = socket.clone(); let configuration = configuration.clone(); Callback::from(move |_| send_command(&socket, Command::SetPyroConfiguration { configuration: (*configuration).clone() })) };
    let test1 = { let socket = socket.clone(); Callback::from(move |_| send_command(&socket, Command::TestPyro { channel: 1 })) };
    let test2 = { let socket = socket.clone(); Callback::from(move |_| send_command(&socket, Command::TestPyro { channel: 2 })) };
    html! { <div class="calibration-page"><Card title="pyro checkout" icon="electrical_services">
        <p class="safety-note">{"PYR1: parachute test load only. PYR2: wire short only; never an igniter."}</p>
        <div class="status">{format!("PYR1: {}{} · PYR2: {}{}", if state.pyro.channel1.continuity {"CONTINUITY"} else {"OPEN"}, if state.pyro.channel1.fire {" · FIRING"} else {""}, if state.pyro.channel2.continuity {"CONTINUITY"} else {"OPEN"}, if state.pyro.channel2.fire {" · FIRING"} else {""})}</div>
        <div class="config-grid"><label>{"PYR1 parachute duration (ms)"}<NumericInput value={configuration.parachute_duration_ms.to_string()} on_commit={update(1)} disabled={parachute_locked}/></label><label>{"PYR2 igniter duration (ms)"}<NumericInput value={configuration.igniter_duration_ms.to_string()} on_commit={update(2)} disabled={igniter_locked}/></label></div>
        <div class="button-row"><button class="save-button" onclick={save} disabled={parachute_locked && igniter_locked}>{"Apply durations"}</button><button class="danger" onclick={test1} disabled={state.pyro.channel1.fire}>{if state.pyro.channel1.fire {"PYR1 firing…"} else {"Test PYR1"}}</button><button class="danger" onclick={test2} disabled={!pyro2_test_allowed || state.pyro.channel2.fire}>{if state.pyro.channel2.fire {"PYR2 firing…"} else {"Test PYR2 wire only"}}</button></div>
    </Card></div> }
}

#[function_component]
fn ServoCalibration() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let axis = use_state(|| ServoAxis::X);
    let config = use_state(ServoConfiguration::default);
    let enabled = use_state(|| false);
    let manual_position = use_state(|| 0.0_f32);
    let tests = use_state(Vec::<StoredTest>::new);
    let testing = use_state(|| false);
    let selected_device = use_state(|| 0_usize);
    let draft = use_state(|| ConfigDraft::from_config(&ServoConfiguration::default()));
    let configuration_loaded = use_state(|| false);
    let socket = connection.send_command.clone();
    {
        let state = state.clone();
        let tests = tests.clone();
        let testing = testing.clone();
        let config = config.clone();
        let draft = draft.clone();
        let enabled = enabled.clone();
        let configuration_loaded = configuration_loaded.clone();
        use_effect_with_deps(
            move |message| {
                if let Some(message) = &**message {
                    if let Ok(SocketMessage::State(new_state)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        if !*configuration_loaded {
                            let configuration = new_state.servo_calibration.x.clone();
                            draft.set(ConfigDraft::from_config(&configuration));
                            config.set(configuration);
                            enabled.set(new_state.servo_calibration.x_enabled);
                            configuration_loaded.set(true);
                        }
                        state.set(new_state);
                    }
                    if let Ok(SocketMessage::TestResult(result)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        let mut all = (*tests).clone();
                        let color = TEST_COLORS[all.len() % TEST_COLORS.len()];
                        all.push(StoredTest {
                            result,
                            visible: true,
                            color,
                        });
                        tests.set(all);
                        testing.set(false);
                    }
                }
                || ()
            },
            connection.message.clone(),
        );
    }
    let select_axis = |new_axis: ServoAxis| {
        let axis = axis.clone();
        let config = config.clone();
        let draft = draft.clone();
        let enabled = enabled.clone();
        let state = state.clone();
        Callback::from(move |_| {
            axis.set(new_axis);
            let next_config = if new_axis == ServoAxis::X {
                state.servo_calibration.x.clone()
            } else {
                state.servo_calibration.y.clone()
            };
            draft.set(ConfigDraft::from_config(&next_config));
            config.set(next_config);
            enabled.set(if new_axis == ServoAxis::X {
                state.servo_calibration.x_enabled
            } else {
                state.servo_calibration.y_enabled
            });
        })
    };
    let update = |field: &'static str| {
        let config = config.clone();
        let draft = draft.clone();
        Callback::from(move |value: f32| {
            let text = value.to_string();
            let mut next_draft = (*draft).clone();
            match field {
                "zero" => next_draft.zero = text.clone(),
                "turn" => next_draft.turn = text.clone(),
                "p" => next_draft.p = text.clone(),
                "i" => next_draft.i = text.clone(),
                "d" => next_draft.d = text.clone(),
                "limit" => next_draft.limit = text.clone(),
                _ => {}
            }
            draft.set(next_draft);
            let mut c = (*config).clone();
            match field {
                "turn" => c.max_turn_degrees = value,
                "zero" => c.zero_encoder_count = value.round().clamp(0.0, 16383.0) as u16,
                "p" => c.position_p = value,
                "i" => c.position_i = value,
                "d" => c.position_d = value,
                "limit" => c.duty_cycle_limit = value.clamp(0.0, 1.0),
                _ => {}
            };
            config.set(c);
        })
    };
    let toggle_reverse = {
        let config = config.clone();
        Callback::from(move |event: Event| {
            let mut next = (*config).clone();
            next.reverse_motor = event.target_unchecked_into::<HtmlInputElement>().checked();
            config.set(next);
        })
    };
    let toggle_control_reverse = {
        let config = config.clone();
        Callback::from(move |event: Event| {
            let mut next = (*config).clone();
            next.reverse_control = event.target_unchecked_into::<HtmlInputElement>().checked();
            config.set(next);
        })
    };
    let apply = {
        let axis = axis.clone();
        let config = config.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::SetServoConfiguration {
                    axis: *axis,
                    configuration: (*config).clone(),
                },
            )
        })
    };
    let choose_device = {
        let selected_device = selected_device.clone();
        Callback::from(move |event: Event| {
            selected_device.set(
                event
                    .target_unchecked_into::<HtmlInputElement>()
                    .value()
                    .parse::<usize>()
                    .unwrap_or(0),
            );
        })
    };
    let assign_device = {
        let axis = axis.clone();
        let selected_device = selected_device.clone();
        let state = state.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            if let Some(device) = state
                .servo_calibration
                .discovered_devices
                .get(*selected_device)
            {
                send_command(
                    &socket,
                    Command::AssignServoDevice {
                        axis: *axis,
                        device: device.clone(),
                    },
                );
            }
        })
    };
    let capture_zero = {
        let axis = axis.clone();
        let socket = socket.clone();
        let state = state.clone();
        let config = config.clone();
        let draft = draft.clone();
        Callback::from(move |_| {
            let mut next = (*config).clone();
            next.zero_encoder_count = if *axis == ServoAxis::X {
                state.servo1.position
            } else {
                state.servo2.position
            };
            let mut next_draft = (*draft).clone();
            next_draft.zero = next.zero_encoder_count.to_string();
            draft.set(next_draft);
            config.set(next);
            send_command(&socket, Command::CaptureServoZero { axis: *axis });
        })
    };
    let toggle = {
        let axis = axis.clone();
        let enabled = enabled.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            let on = !*enabled;
            enabled.set(on);
            send_command(
                &socket,
                Command::SetServoEnabled {
                    axis: *axis,
                    enabled: on,
                },
            );
        })
    };
    let manual = {
        let manual_position = manual_position.clone();
        Callback::from(move |event: InputEvent| {
            manual_position.set(
                event
                    .target_unchecked_into::<HtmlInputElement>()
                    .value()
                    .parse::<f32>()
                    .unwrap_or(0.0),
            )
        })
    };
    let send_manual = {
        let axis = axis.clone();
        let enabled = enabled.clone();
        let socket = socket.clone();
        Callback::from(move |event: Event| {
            let p = event
                .target_unchecked_into::<HtmlInputElement>()
                .value()
                .parse::<f32>()
                .unwrap_or(0.0)
                .clamp(-1.0, 1.0);
            if *enabled {
                send_command(
                    &socket,
                    Command::SetServoPosition {
                        axis: *axis,
                        position: p,
                    },
                );
            }
        })
    };
    let start_test = {
        let axis = axis.clone();
        let config = config.clone();
        let testing = testing.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            testing.set(true);
            send_command(
                &socket,
                Command::StartServoPerformanceTest {
                    axis: *axis,
                    configuration: (*config).clone(),
                },
            );
        })
    };
    let save = {
        let axis = axis.clone();
        let config = config.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::SetServoConfiguration {
                    axis: *axis,
                    configuration: (*config).clone(),
                },
            );
            send_command(&socket, Command::SaveServoConfigurations);
        })
    };
    let detected = if *axis == ServoAxis::X {
        state.servo_calibration.detected_x
    } else {
        state.servo_calibration.detected_y
    };
    let position = if *axis == ServoAxis::X {
        state.servo1.position
    } else {
        state.servo2.position
    };
    let assigned_device = if *axis == ServoAxis::X {
        state.servo_calibration.x_device.as_ref()
    } else {
        state.servo_calibration.y_device.as_ref()
    };
    let device_picker = if state.servo_calibration.discovered_devices.is_empty() {
        html! { <p>{"No servos. Check CAN power, wiring, termination; reboot."}</p> }
    } else {
        html! {
            <div class="device-picker">
                <label>{"Detected device (Chip ID1 / Chip ID2)"}
                    <select onchange={choose_device}>
                        {for state.servo_calibration.discovered_devices.iter().enumerate().map(|(index, device)| html! {
                            <option value={index.to_string()} selected={index == *selected_device}>{device_name(device)}</option>
                        })}
                    </select>
                </label>
                <button onclick={assign_device}>{format!("Assign selected device to {}", axis_name(*axis))}</button>
            </div>
        }
    };
    html! { <div class="calibration-page">
      <Card title="servo calibration" icon="tune"><p class="safety-note">{"Bench only. Restrain rocket; keep clear of TVC."}</p><div class="axis-row"><span>{"Servo:"}</span><button class={if *axis == ServoAxis::X {"selected"} else {""}} onclick={select_axis(ServoAxis::X)}>{"X axis"}</button><button class={if *axis == ServoAxis::Y {"selected"} else {""}} onclick={select_axis(ServoAxis::Y)}>{"Y axis"}</button></div>{device_picker}<div class="status">{format!("{}: {}; encoder {}", axis_name(*axis), if detected {"detected"} else {"not detected"}, position)}<br/>{match assigned_device { Some(device) => format!("Assigned: {}", device_name(device)), None => "No device assigned to this axis".into() }}</div></Card>
      <Card title="configuration" icon="settings"><div class="config-grid">
        <label>{"Encoder zero (0–16383)"}<NumericInput value={draft.zero.clone()} on_commit={update("zero")}/></label><label>{"Max turn (° at ±1.0)"}<NumericInput value={draft.turn.clone()} on_commit={update("turn")}/></label><label>{"Position P"}<NumericInput value={draft.p.clone()} on_commit={update("p")}/></label><label>{"Position I"}<NumericInput value={draft.i.clone()} on_commit={update("i")}/></label><label>{"Position D"}<NumericInput value={draft.d.clone()} on_commit={update("d")}/></label><label>{"Duty limit (0–1)"}<NumericInput value={draft.limit.clone()} on_commit={update("limit")}/></label><label class="checkbox-label"><input type="checkbox" checked={config.reverse_motor} onchange={toggle_reverse}/>{"Reverse motor direction (servo firmware)"}</label><label class="checkbox-label"><input type="checkbox" checked={config.reverse_control} onchange={toggle_control_reverse}/>{"Reverse TVC command direction (flight controller: −1 ↔ +1)"}</label>
      </div><div class="button-row"><button onclick={apply}>{"Apply"}</button><button onclick={capture_zero}>{"Set current as zero"}</button></div></Card>
      <Card title="manual test" icon="gamepad"><button class={if *enabled {"danger"} else {""}} onclick={toggle}>{if *enabled {"Motor ON — holding position"} else {"Motor OFF — freewheeling"}}</button><label class="slider-label">{format!("Command: {:.2}", *manual_position)}<input type="range" min="-1" max="1" step="0.01" value={manual_position.to_string()} disabled={!*enabled} oninput={manual} onchange={send_manual}/></label></Card>
      <Card title="step response" icon="show_chart"><p>{"0 → +0.75 · 2 s settle · 250 ms capture"}</p><button disabled={*testing} onclick={start_test}>{if *testing {"Capturing…"} else {"Run test"}}</button><TestPlot tests={(*tests).clone()}/><TestTable tests={tests.clone()} config={config.clone()}/></Card><button class="save-button" onclick={save}>{"Save to flash"}</button>
    </div> }
}

#[derive(Properties, PartialEq)]
struct TestPlotProps {
    tests: Vec<StoredTest>,
}
#[function_component]
fn TestPlot(props: &TestPlotProps) -> Html {
    let lines = props.tests.iter().filter(|t| t.visible).map(|t| {
        let points = t
            .result
            .samples
            .iter()
            .map(|s| {
                format!(
                    "{:.1},{:.1}",
                    s.time_us as f32 / 250_000.0 * 600.0,
                    // Focus the chart on the useful 0…+1 command range,
                    // with a small -0.1…+1.1 visual margin.
                    270.0 - (s.position + 0.1) / 1.2 * 250.0
                )
            })
            .collect::<Vec<_>>()
            .join(" ");
        html! { <polyline points={points} fill="none" stroke={t.color} stroke-width="2"/> }
    });
    html! { <svg class="response-plot" viewBox="0 0 600 300">
        <line x1="0" y1="249" x2="600" y2="249" class="axis"/>
        <line x1="0" y1="197" x2="600" y2="197" class="grid"/>
        <line x1="0" y1="145" x2="600" y2="145" class="grid"/>
        <line x1="0" y1="93" x2="600" y2="93" class="grid"/>
        <line x1="0" y1="41" x2="600" y2="41" class="grid"/>
        <line x1="120" y1="20" x2="120" y2="260" class="grid"/>
        <line x1="240" y1="20" x2="240" y2="260" class="grid"/>
        <line x1="360" y1="20" x2="360" y2="260" class="grid"/>
        <line x1="480" y1="20" x2="480" y2="260" class="grid"/>
        <line x1="600" y1="20" x2="600" y2="260" class="grid"/>
        {for lines}
        <text x="4" y="249">{"0.00"}</text><text x="4" y="197">{"0.25"}</text><text x="4" y="145">{"0.50"}</text><text x="4" y="93">{"0.75"}</text><text x="4" y="41">{"1.00"}</text><text x="0" y="278">{"0"}</text><text x="112" y="278">{"50"}</text><text x="228" y="278">{"100"}</text><text x="348" y="278">{"150"}</text><text x="468" y="278">{"200"}</text><text x="545" y="278">{"250 ms"}</text>
    </svg> }
}

#[derive(Properties, PartialEq)]
struct TestTableProps {
    tests: UseStateHandle<Vec<StoredTest>>,
    config: UseStateHandle<ServoConfiguration>,
}
#[function_component]
fn TestTable(props: &TestTableProps) -> Html {
    html! { <table class="test-table"><thead><tr><th>{"Test"}</th><th>{"Axis"}</th><th>{"P / I / D"}</th><th>{"Samples"}</th><th>{"Actions"}</th></tr></thead><tbody>{for props.tests.iter().enumerate().map(|(index, test)| { let toggle = props.tests.clone(); let delete = props.tests.clone(); let recolor = props.tests.clone(); let config = props.config.clone(); let saved = test.clone(); html! { <tr><td><span class="color-dot" title="Change line color" style={format!("background:{}", test.color)} onclick={Callback::from(move |_| { let mut a = (*recolor).clone(); let current = TEST_COLORS.iter().position(|color| *color == a[index].color).unwrap_or(0); a[index].color = TEST_COLORS[(current + 1) % TEST_COLORS.len()]; recolor.set(a); })}></span>{index + 1}</td><td>{axis_name(test.result.axis)}</td><td>{format!("{:.3} / {:.3} / {:.3}", test.result.configuration.position_p, test.result.configuration.position_i, test.result.configuration.position_d)}</td><td>{test.result.samples.len()}</td><td><button onclick={Callback::from(move |_| { let mut a = (*toggle).clone(); a[index].visible = !a[index].visible; toggle.set(a); })}>{if test.visible {"Hide"} else {"Show"}}</button><button onclick={Callback::from(move |_| { let mut a = (*delete).clone(); a.remove(index); delete.set(a); })}>{"Delete"}</button><button onclick={Callback::from(move |_| config.set(saved.result.configuration.clone()))}>{"Load config"}</button></td></tr> } })}</tbody></table> }
}

fn inertia_axis_name(axis: Option<InertiaAxis>) -> &'static str {
    match axis {
        Some(InertiaAxis::X) => "X axis",
        Some(InertiaAxis::Y) => "Y axis",
        None => "—",
    }
}

/// Axial inertia of a uniform 50 mm-diameter cylinder: I = ½mr².
fn cylinder_z_inertia(configuration: &InertiaConfiguration) -> f32 {
    0.5 * (configuration.mass_g / 1000.0) * 0.025_f32.powi(2)
}

#[derive(Clone, PartialEq)]
struct PendulumFit {
    samples: Vec<InertiaGyroSample>,
    axis: InertiaAxis,
    mass_kg: f64,
    offset_m: f64,
    // Optimized coordinates: ln(I), ln(initial angle), phase, gyro bias,
    // ln(damping). Exponential coordinates keep physical values valid.
    coordinates: [f64; 5],
    cost: f64,
    learning_rate: f64,
    iterations: u32,
    finished: bool,
}

impl PendulumFit {
    fn from_capture(result: &InertiaCaptureResult, mass_g: f64, offset_mm: f64) -> Option<Self> {
        let axis = result.dominant_axis?;
        if result.samples.len() < 8 || mass_g <= 0.0 || offset_mm <= 0.0 {
            return None;
        }
        let values = result.samples.iter().map(|sample| if axis == InertiaAxis::X { sample.gyro_x_radps } else { sample.gyro_y_radps } as f64).collect::<Vec<_>>();
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let bias = (min + max) * 0.5;
        let omega_amplitude = ((max - min) * 0.5).max(0.01);
        let crossings = result
            .samples
            .windows(2)
            .filter_map(|pair| {
                let a = (if axis == InertiaAxis::X {
                    pair[0].gyro_x_radps
                } else {
                    pair[0].gyro_y_radps
                }) as f64
                    - bias;
                let b = (if axis == InertiaAxis::X {
                    pair[1].gyro_x_radps
                } else {
                    pair[1].gyro_y_radps
                }) as f64
                    - bias;
                if a.signum() != b.signum() && (a - b).abs() > f64::EPSILON {
                    let fraction = a.abs() / (a.abs() + b.abs());
                    Some(
                        (pair[0].time_us as f64
                            + fraction * (pair[1].time_us - pair[0].time_us) as f64)
                            / 1_000_000.0,
                    )
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        let half_period = crossings
            .windows(2)
            .map(|pair| pair[1] - pair[0])
            .filter(|period| *period > 0.001)
            .sum::<f64>()
            / (crossings.len().saturating_sub(1).max(1) as f64);
        let frequency_hz = if crossings.len() > 1 {
            1.0 / (2.0 * half_period)
        } else {
            1.0
        };
        let natural_omega = (2.0 * std::f64::consts::PI * frequency_hz).max(0.1);
        let mass_kg = mass_g / 1000.0;
        let offset_m = offset_mm / 1000.0;
        let inertia = (mass_kg * 9.80665 * offset_m / (natural_omega * natural_omega)).max(1e-5);
        // A zero crossing identifies phase only modulo π: either the positive
        // or negative angular-velocity half-cycle could pass through it. Try
        // both branches against the full capture before optimization.
        let phase = crossings
            .first()
            .map(|time| -natural_omega * *time)
            .unwrap_or(0.0);
        let envelope_peaks = values
            .windows(3)
            .enumerate()
            .filter_map(|(index, window)| {
                let center = (window[1] - bias).abs();
                if center >= (window[0] - bias).abs() && center >= (window[2] - bias).abs() {
                    Some((index + 1, center))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        let initial_damping = match (envelope_peaks.first(), envelope_peaks.last()) {
            (Some((first_index, first_peak)), Some((last_index, last_peak)))
                if last_index > first_index && *last_peak > 0.001 =>
            {
                let elapsed = (result.samples[*last_index].time_us
                    - result.samples[*first_index].time_us) as f64
                    / 1_000_000.0;
                (2.0 * (first_peak / last_peak).ln() / elapsed.max(0.001)).clamp(0.001, 5.0)
            }
            _ => 0.02,
        };
        let mut fit = Self {
            samples: result.samples.clone(),
            axis,
            mass_kg,
            offset_m,
            coordinates: [
                inertia.ln(),
                (omega_amplitude / natural_omega).max(0.005).ln(),
                phase,
                bias,
                initial_damping.ln(),
            ],
            cost: f64::INFINITY,
            learning_rate: 0.025,
            iterations: 0,
            finished: false,
        };
        let mut opposite_phase = fit.coordinates;
        opposite_phase[2] += std::f64::consts::PI;
        let direct_cost = fit.cost_for(fit.coordinates);
        let opposite_cost = fit.cost_for(opposite_phase);
        if opposite_cost < direct_cost {
            fit.coordinates = opposite_phase;
            fit.cost = opposite_cost;
        } else {
            fit.cost = direct_cost;
        }
        Some(fit)
    }

    fn parameters(&self, coordinates: [f64; 5]) -> (f64, f64, f64, f64, f64) {
        (
            coordinates[0].exp().clamp(1e-6, 100.0),
            coordinates[1].exp().clamp(0.001, 1.5),
            coordinates[2],
            coordinates[3].clamp(-20.0, 20.0),
            coordinates[4].exp().clamp(0.0001, 10.0),
        )
    }

    fn simulated(&self, coordinates: [f64; 5]) -> Vec<f64> {
        let (inertia, angle_amplitude, phase, bias, damping) = self.parameters(coordinates);
        let natural_omega = (self.mass_kg * 9.80665 * self.offset_m / inertia).sqrt();
        let mut theta = angle_amplitude * phase.cos();
        let mut omega = -angle_amplitude * natural_omega * phase.sin();
        let mut previous_time = 0.0;
        self.samples
            .iter()
            .map(|sample| {
                let time = sample.time_us as f64 / 1_000_000.0;
                let dt = (time - previous_time).max(0.0);
                previous_time = time;
                let acceleration = |angle: f64, velocity: f64| {
                    -(self.mass_kg * 9.80665 * self.offset_m / inertia) * angle.sin()
                        - damping * velocity
                };
                let k1_theta = omega;
                let k1_omega = acceleration(theta, omega);
                let k2_theta = omega + k1_omega * dt * 0.5;
                let k2_omega =
                    acceleration(theta + k1_theta * dt * 0.5, omega + k1_omega * dt * 0.5);
                let k3_theta = omega + k2_omega * dt * 0.5;
                let k3_omega =
                    acceleration(theta + k2_theta * dt * 0.5, omega + k2_omega * dt * 0.5);
                let k4_theta = omega + k3_omega * dt;
                let k4_omega = acceleration(theta + k3_theta * dt, omega + k3_omega * dt);
                theta += dt * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta) / 6.0;
                omega += dt * (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega) / 6.0;
                omega + bias
            })
            .collect()
    }

    fn cost_for(&self, coordinates: [f64; 5]) -> f64 {
        let simulated = self.simulated(coordinates);
        let cost = simulated
            .iter()
            .zip(&self.samples)
            .map(|(prediction, sample)| {
                let measured = if self.axis == InertiaAxis::X {
                    sample.gyro_x_radps
                } else {
                    sample.gyro_y_radps
                } as f64;
                let residual = prediction - measured;
                if residual.is_finite() {
                    residual * residual
                } else {
                    1e12
                }
            })
            .sum::<f64>()
            / simulated.len().max(1) as f64;
        if cost.is_finite() {
            cost.min(1e12)
        } else {
            1e12
        }
    }

    fn iterate(&mut self) {
        if self.finished {
            return;
        }
        let mut gradient = [0.0; 5];
        for index in 0..5 {
            let step = if index == 3 { 0.001 } else { 0.002 };
            let mut plus = self.coordinates;
            plus[index] += step;
            let mut minus = self.coordinates;
            minus[index] -= step;
            gradient[index] = (self.cost_for(plus) - self.cost_for(minus)) / (2.0 * step);
        }
        let norm = gradient
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        if self.iterations >= 900 {
            self.finished = true;
            return;
        }
        if !norm.is_finite() {
            // Keep the fit visible and retry with a smaller step rather than
            // silently reporting completion after a single unstable trial.
            self.learning_rate *= 0.5;
            self.iterations += 1;
            if self.learning_rate < 1e-6 && self.iterations >= 120 {
                self.finished = true;
            }
            return;
        }
        if norm < 1e-10 {
            self.iterations += 1;
            if self.iterations >= 120 {
                self.finished = true;
            }
            return;
        }
        let mut candidate = self.coordinates;
        for index in 0..5 {
            candidate[index] -= self.learning_rate * gradient[index] / norm;
        }
        let candidate_cost = self.cost_for(candidate);
        if candidate_cost < self.cost {
            self.coordinates = candidate;
            self.cost = candidate_cost;
            self.learning_rate = (self.learning_rate * 1.04).min(0.1);
        } else {
            self.learning_rate *= 0.5;
        }
        self.iterations += 1;
        if (self.learning_rate < 1e-5 && self.iterations >= 120) || self.iterations >= 900 {
            self.finished = true;
        }
    }
}

#[function_component]
fn MomentOfInertia() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let result = use_state_eq(InertiaCaptureResult::default);
    let configuration = use_state_eq(InertiaConfiguration::default);
    let configuration_loaded = use_state(|| false);
    let fit = use_state_eq(|| None::<PendulumFit>);
    let published_fit_iteration = use_state(|| None::<u32>);
    let socket = connection.send_command.clone();
    {
        let state = state.clone();
        let result = result.clone();
        let configuration = configuration.clone();
        let configuration_loaded = configuration_loaded.clone();
        let fit = fit.clone();
        let published_fit_iteration = published_fit_iteration.clone();
        use_effect_with_deps(
            move |message| {
                if let Some(message) = &**message {
                    if let Ok(SocketMessage::State(next_state)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        if !*configuration_loaded {
                            let mut loaded = next_state.inertia_configuration.clone();
                            loaded.moment_of_inertia_kgm2[2] = cylinder_z_inertia(&loaded);
                            configuration.set(loaded);
                            configuration_loaded.set(true);
                        }
                        state.set(next_state);
                    }
                    if let Ok(SocketMessage::InertiaCaptureResult(next_result)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        let next_fit = PendulumFit::from_capture(
                            &next_result,
                            configuration.mass_g as f64,
                            configuration.center_of_mass_offset_mm as f64,
                        );
                        result.set(next_result);
                        fit.set(next_fit);
                        published_fit_iteration.set(None);
                    }
                }
                || ()
            },
            connection.message.clone(),
        );
    }
    let fitting = fit.as_ref().map(|fit| !fit.finished).unwrap_or(false);
    let fit_iteration = fit.as_ref().map(|fit| fit.iterations).unwrap_or(0);
    {
        let fit = fit.clone();
        use_effect_with_deps(
            move |fitting| {
                let interval = if fitting.0 {
                    Some(Timeout::new(20, move || {
                        let mut next = (*fit).clone();
                        if let Some(model) = next.as_mut() {
                            // Render after every numerical iteration so the
                            // user can observe convergence of the fitted line.
                            model.iterate();
                        }
                        fit.set(next);
                    }))
                } else {
                    None
                };
                move || drop(interval)
            },
            (fitting, fit_iteration),
        );
    }
    let start_capture = {
        let socket = socket.clone();
        Callback::from(move |_| send_command(&socket, Command::StartInertiaCapture))
    };
    {
        let fit = fit.clone();
        let configuration = configuration.clone();
        let socket = socket.clone();
        let published = published_fit_iteration.clone();
        let completed_iteration = fit
            .as_ref()
            .filter(|model| model.finished)
            .map(|model| model.iterations);
        use_effect_with_deps(
            move |completed_iteration| {
                if let Some(iteration) = *completed_iteration {
                    if *published != Some(iteration) {
                        if let Some(model) = fit.as_ref() {
                            let mut next = (*configuration).clone();
                            let index = if model.axis == InertiaAxis::X { 0 } else { 1 };
                            next.moment_of_inertia_kgm2[index] =
                                model.parameters(model.coordinates).0 as f32;
                            next.moment_of_inertia_kgm2[2] = cylinder_z_inertia(&next);
                            configuration.set(next.clone());
                            send_command(
                                &socket,
                                Command::SetInertiaConfiguration {
                                    configuration: next,
                                },
                            );
                            published.set(Some(iteration));
                        }
                    }
                }
                || ()
            },
            completed_iteration,
        );
    }
    let update_mass = {
        let configuration = configuration.clone();
        Callback::from(move |value: f32| {
            let mut next = (*configuration).clone();
            next.mass_g = value.max(0.0);
            next.moment_of_inertia_kgm2[2] = cylinder_z_inertia(&next);
            configuration.set(next);
        })
    };
    let update_offset = {
        let configuration = configuration.clone();
        Callback::from(move |value: f32| {
            let mut next = (*configuration).clone();
            next.center_of_mass_offset_mm = value.max(0.0);
            configuration.set(next);
        })
    };
    let update_inertia = |index: usize| {
        let configuration = configuration.clone();
        Callback::from(move |value: f32| {
            let mut next = (*configuration).clone();
            next.moment_of_inertia_kgm2[index] = value.max(0.0);
            configuration.set(next);
        })
    };
    let save_configuration = {
        let configuration = configuration.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::SaveInertiaConfiguration {
                    configuration: (*configuration).clone(),
                },
            )
        })
    };
    let peak = result.samples.iter().fold(0.1_f32, |peak, sample| {
        peak.max(sample.gyro_x_radps.abs())
            .max(sample.gyro_y_radps.abs())
    });
    let duration_us = result
        .samples
        .last()
        .map(|sample| sample.time_us.max(1))
        .unwrap_or(5_000_000);
    let points = |x: bool| {
        result
            .samples
            .iter()
            .map(|sample| {
                let horizontal = sample.time_us as f32 / duration_us as f32 * 600.0;
                let value = if x {
                    sample.gyro_x_radps
                } else {
                    sample.gyro_y_radps
                };
                format!("{:.1},{:.1}", horizontal, 130.0 - value / peak * 105.0)
            })
            .collect::<Vec<_>>()
            .join(" ")
    };
    let fitted = fit
        .as_ref()
        .map(|model| model.simulated(model.coordinates))
        .unwrap_or_default();
    let fitted_points = result
        .samples
        .iter()
        .zip(fitted.iter())
        .map(|(sample, value)| {
            let horizontal = sample.time_us as f32 / duration_us as f32 * 600.0;
            format!(
                "{:.1},{:.1}",
                horizontal,
                130.0 - *value as f32 / peak * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let fit_parameters = fit
        .as_ref()
        .map(|model| model.parameters(model.coordinates));
    html! {
        <div class="inertia-page">
            <Card title="moment of inertia measurement" icon="rotate_right">
                <p class="safety-note">{"Bench only. Secure pendulum; pyro and TVC off."}</p>
                <p>{"Release, then capture 5 s."}</p>
                <div class="config-grid"><label>{"Rocket mass (g)"}<NumericInput value={configuration.mass_g.to_string()} on_commit={update_mass}/></label><label>{"Center of mass position (mm)"}<NumericInput value={configuration.center_of_mass_offset_mm.to_string()} on_commit={update_offset}/></label><label>{"X moment (kg·m²)"}<NumericInput value={format!("{:.5e}", configuration.moment_of_inertia_kgm2[0])} on_commit={update_inertia(0)}/></label><label>{"Y moment (kg·m²)"}<NumericInput value={format!("{:.5e}", configuration.moment_of_inertia_kgm2[1])} on_commit={update_inertia(1)}/></label><label>{"Z moment — 50 mm cylinder estimate (kg·m²)"}<input type="text" readonly=true value={format!("{:.5e}", configuration.moment_of_inertia_kgm2[2])}/></label></div>
                <button onclick={start_capture} disabled={state.inertia_capture.running}>{if state.inertia_capture.running {"Capturing gyro data…"} else {"Capture 5 seconds"}}</button>
                <button class="save-button" onclick={save_configuration}>{"Save parameters to flash"}</button>
            </Card>
            <Card title="capture result" icon="show_chart">
                <div class="inertia-summary"><span>{"Dominant swing axis"}</span><strong>{inertia_axis_name(result.dominant_axis)}</strong><span>{"Measured acquisition rate"}</span><strong>{format!("{:.1} Hz", result.measured_rate_hz)}</strong><span>{"Samples"}</span><strong>{result.samples.len()}</strong></div>
                <svg class="inertia-plot" viewBox="0 0 600 260" aria-label="Gyroscope capture plot">
                    <line x1="0" y1="130" x2="600" y2="130" class="axis"/>
                    <line x1="120" y1="10" x2="120" y2="245" class="grid"/><line x1="240" y1="10" x2="240" y2="245" class="grid"/><line x1="360" y1="10" x2="360" y2="245" class="grid"/><line x1="480" y1="10" x2="480" y2="245" class="grid"/>
                    <polyline points={points(true)} fill="none" stroke="#1565c0" stroke-width="2"/>
                    <polyline points={points(false)} fill="none" stroke="#d32f2f" stroke-width="2"/>
                    <polyline points={fitted_points} fill="none" stroke="#2e7d32" stroke-width="2" stroke-dasharray="6 3"/>
                    <text x="8" y="20">{"X gyro"}</text><text x="8" y="40">{"Y gyro"}</text><text x="500" y="252">{"5 s"}</text>
                </svg>
                <p class="plot-legend"><span class="legend-x">{"X gyro"}</span><span class="legend-y">{"Y gyro"}</span><span class="legend-fit">{"Fitted dominant axis"}</span>{" · rad/s"}</p>
            </Card>
            if let Some((inertia, amplitude, phase, bias, damping)) = fit_parameters {
                <Card title="physical pendulum fit" icon="calculate">
                    <div class="inertia-summary"><span>{"Fit status"}</span><strong>{if fitting {"Optimizing"} else {"Complete"}}</strong><span>{"Iterations"}</span><strong>{fit.as_ref().unwrap().iterations}</strong><span>{"Mean squared error"}</span><strong>{format!("{:.6}", fit.as_ref().unwrap().cost)}</strong><span>{"Moment of inertia"}</span><strong>{format!("{:.5} kg·m²", inertia)}</strong><span>{"Initial angle amplitude"}</span><strong>{format!("{:.3} rad", amplitude)}</strong><span>{"Phase"}</span><strong>{format!("{:.3} rad", phase)}</strong><span>{"Gyro baseline"}</span><strong>{format!("{:.4} rad/s", bias)}</strong><span>{"Decay rate"}</span><strong>{format!("{:.4} s⁻¹", damping)}</strong></div>
                </Card>
            }
        </div>
    }
}

#[function_component]
fn RocketOnboarding() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let checklist_active = connection.state.prelaunch_checklist.active;
    let page = use_state(|| 0_usize);
    let show_calibration = {
        let page = page.clone();
        Callback::from(move |_| page.set(0))
    };
    let show_inertia = {
        let page = page.clone();
        Callback::from(move |_| page.set(1))
    };
    let show_orientation = {
        let page = page.clone();
        Callback::from(move |_| page.set(2))
    };
    let show_simulation = {
        let page = page.clone();
        Callback::from(move |_| page.set(3))
    };
    let show_pyro = {
        let page = page.clone();
        Callback::from(move |_| page.set(4))
    };
    html! {
        <>
            <PrelaunchChecklist/>
            if checklist_active { <GuidedOnboarding/> } else { <><Card title="rocket onboarding" icon="school">
                <p>{"Choose a setup page."}</p>
                <div class="axis-row"><button class={if *page == 0 {"selected"} else {""}} onclick={show_calibration}>{"Servo calibration"}</button><button class={if *page == 1 {"selected"} else {""}} onclick={show_inertia}>{"Moment of inertia"}</button><button class={if *page == 2 {"selected"} else {""}} onclick={show_orientation}>{"TVC orientation"}</button><button class={if *page == 3 {"selected"} else {""}} onclick={show_simulation}>{"Flight simulation"}</button><button class={if *page == 4 {"selected"} else {""}} onclick={show_pyro}>{"Pyro checkout"}</button></div>
            </Card>
            if *page == 0 { <ServoCalibration/> } else if *page == 1 { <MomentOfInertia/> } else if *page == 2 { <ServoOrientationCheck/> } else if *page == 3 { <RocketSimulation/> } else { <PyroConfigurationPage/> }</> }
        </>
    }
}

fn run_rocket_preview(
    inertia: &InertiaConfiguration,
    configuration: &SimulationConfiguration,
) -> Vec<SimulationLog> {
    let environment = Environment {
        wind: Vec3::new(
            configuration.wind_mps[0],
            configuration.wind_mps[1],
            configuration.wind_mps[2],
        ),
        max_time: 8.0,
        dt: 0.002,
        substeps: 5,
        ..Environment::default()
    };
    let rocket = RocketParameters::from_configuration(inertia, configuration, environment.dt);
    let radians = std::f32::consts::PI / 180.0;
    let mut initial = RocketState::default();
    initial.rotation = Quat::axis_angle(
        Vec3::new(1.0, 0.0, 0.0),
        configuration.initial_tilt_degrees[0] * radians,
    ) * Quat::axis_angle(
        Vec3::new(0.0, 1.0, 0.0),
        configuration.initial_tilt_degrees[1] * radians,
    );
    let mut controller = PidControlSystem::new(
        configuration.pid_gains[0],
        configuration.pid_gains[1],
        configuration.pid_gains[2],
    );
    controller.set_estimated_rotation(initial.rotation);
    NumericalSimulation.run(environment, rocket, initial, &mut controller, true)
}

fn plot_series(
    trajectory: &[SimulationLog],
    magnitude: f32,
    value: impl Fn(&SimulationLog) -> f32,
) -> String {
    let duration = trajectory
        .last()
        .map(|point| point.time)
        .unwrap_or(1.0)
        .max(0.001);
    trajectory
        .iter()
        .map(|point| {
            format!(
                "{:.1},{:.1}",
                40.0 + point.time / duration * 540.0,
                130.0 - value(point) / magnitude * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// Renderer-neutral data consumed by the shared simulation/HIL plot view.
#[derive(Clone, PartialEq)]
struct PlotSample {
    time_s: f32,
    position: [f32; 3],
    tilt_degrees: [f32; 2],
    command: [f32; 2],
    actual: [f32; 2],
}

fn simulation_plot_samples(log: &[SimulationLog]) -> Vec<PlotSample> {
    log.iter()
        .map(|point| {
            let up = point.rotation.rotate(Vec3::UP);
            PlotSample {
                time_s: point.time,
                position: [point.position.x, point.position.y, point.position.z],
                tilt_degrees: [
                    (-up.y).atan2(up.z).to_degrees(),
                    up.x.atan2(up.z).to_degrees(),
                ],
                command: point.tvc,
                actual: point.tvc,
            }
        })
        .collect()
}

fn hil_plot_samples(log: &[HilSimulationSample]) -> Vec<PlotSample> {
    log.iter()
        .map(|sample| PlotSample {
            time_s: sample.scheduled_time_us as f32 / 1_000_000.0,
            position: sample.position_m,
            tilt_degrees: sample.orientation_xy_degrees,
            command: sample.tvc_command,
            actual: sample.tvc_actual,
        })
        .collect()
}

#[derive(Properties, PartialEq)]
struct SimulationResultsProps {
    samples: Vec<PlotSample>,
    title: String,
    cutoff_s: Option<f32>,
    #[prop_or_default]
    average_hz: Option<f32>,
}

#[function_component]
fn SimulationResultsView(props: &SimulationResultsProps) -> Html {
    if props.samples.is_empty() {
        return html! { <Card title={props.title.clone()} icon="timeline"><p>{"No result is available yet."}</p></Card> };
    }
    let duration = props.samples.last().unwrap().time_s.max(0.001);
    let spatial = props.samples.iter().fold(1.0_f32, |v, s| {
        v.max(s.position[0].abs()).max(s.position[2].abs())
    });
    let tilt = TILT_GRAPH_LIMIT_DEGREES;
    let trajectory = props
        .samples
        .iter()
        .map(|s| {
            format!(
                "{:.1},{:.1}",
                300.0 + s.position[0] / spatial * 210.0,
                235.0 - s.position[2] / spatial * 210.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let series = |value: fn(&PlotSample) -> f32, scale: f32| {
        props
            .samples
            .iter()
            .map(|s| {
                format!(
                    "{:.1},{:.1}",
                    40.0 + s.time_s / duration * 540.0,
                    130.0 - value(s) / scale * 105.0
                )
            })
            .collect::<Vec<_>>()
            .join(" ")
    };
    let tx = series(|s| s.tilt_degrees[0], tilt);
    let ty = series(|s| s.tilt_degrees[1], tilt);
    let cx = series(|s| s.command[0], 1.0);
    let ax = series(|s| s.actual[0], 1.0);
    let cy = series(|s| s.command[1], 1.0);
    let ay = series(|s| s.actual[1], 1.0);
    let cut = props
        .cutoff_s
        .filter(|t| *t <= duration)
        .map(|t| 40.0 + t / duration * 540.0);
    html! { <><Card title={format!("{} trajectory",props.title)} icon="timeline"><p class="plot-legend">{"X and height use the same metres-per-pixel scale."}</p><svg class="simulation-plot" viewBox="0 0 600 260"><line x1="30" y1="235" x2="580" y2="235" class="axis"/><line x1="300" y1="15" x2="300" y2="235" class="grid"/><polyline points={trajectory} fill="none" stroke="#1565c0" stroke-width="3"/><text x="485" y="252">{format!("±{:.1} m",spatial)}</text></svg></Card><Card title={format!("{} tilt",props.title)} icon="show_chart"><svg class="simulation-plot" viewBox="0 0 600 260"><line x1="40" y1="130" x2="580" y2="130" class="axis"/><line x1="40" y1={format!("{:.1}",130.0-5.0/tilt*105.0)} x2="580" y2={format!("{:.1}",130.0-5.0/tilt*105.0)} class="reference"/><line x1="40" y1={format!("{:.1}",130.0+5.0/tilt*105.0)} x2="580" y2={format!("{:.1}",130.0+5.0/tilt*105.0)} class="reference"/>{if let Some(x)=cut {html!{<line x1={x.to_string()} y1="15" x2={x.to_string()} y2="235" class="cutoff"/>}}else{Html::default()}}<polyline points={tx} fill="none" stroke="#1565c0" stroke-width="2"/><polyline points={ty} fill="none" stroke="#d32f2f" stroke-width="2"/><text x="550" y="22">{"+10°"}</text><text x="550" y="232">{"−10°"}</text></svg><p class="plot-legend"><span class="legend-x">{"X tilt"}</span><span class="legend-y">{"Y tilt"}</span>{" · fixed ±10° scale · gray: ±5°"}</p></Card><Card title={format!("{} TVC response",props.title)} icon="settings_input_component"><svg class="simulation-plot" viewBox="0 0 600 260"><line x1="40" y1="25" x2="580" y2="25" class="reference"/><line x1="40" y1="130" x2="580" y2="130" class="axis"/><line x1="40" y1="235" x2="580" y2="235" class="reference"/><polyline points={cx} fill="none" stroke="#1565c0" stroke-width="2"/><polyline points={ax} fill="none" stroke="#ef6c00" stroke-width="2"/><polyline points={cy} fill="none" stroke="#d32f2f" stroke-width="2" stroke-dasharray="6 3"/><polyline points={ay} fill="none" stroke="#2e7d32" stroke-width="2" stroke-dasharray="6 3"/><text x="550" y="22">{"+1"}</text><text x="550" y="127">{"0"}</text><text x="550" y="232">{"−1"}</text></svg><p class="plot-legend"><span class="legend-x">{"X command"}</span><span class="legend-fit">{"X actual"}</span><span class="legend-y">{"Y command"}</span><span class="legend-fit">{"Y actual"}</span></p>{if let Some(hz)=props.average_hz {html!{<div class="status">{format!("Average physics loop: {:.1} Hz",hz)}</div>}}else{Html::default()}}</Card></> }
}

#[function_component]
fn RocketSimulation() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let configuration = use_state_eq(SimulationConfiguration::default);
    let trajectory = use_state(Vec::<SimulationLog>::new);
    let hil_result = use_state_eq(HilSimulationResult::default);
    let show_hil = use_state(|| false);
    let loaded = use_state(|| false);
    let socket = connection.send_command.clone();
    {
        let state = state.clone();
        let configuration = configuration.clone();
        let loaded = loaded.clone();
        let hil_result = hil_result.clone();
        use_effect_with_deps(
            move |message| {
                if let Some(message) = &**message {
                    if let Ok(SocketMessage::State(next)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        if !*loaded {
                            configuration.set(next.simulation_configuration.clone());
                            loaded.set(true);
                        }
                        state.set(next);
                    }
                    if let Ok(SocketMessage::HilSimulationChunk(chunk)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        let mut result = if chunk.index == 0 {
                            HilSimulationResult {
                                samples: Vec::new(),
                                missed_deadlines: chunk.missed_deadlines,
                            }
                        } else {
                            (*hil_result).clone()
                        };
                        result.samples.extend(chunk.samples);
                        result.missed_deadlines = chunk.missed_deadlines;
                        hil_result.set(result);
                    }
                }
                || ()
            },
            connection.message.clone(),
        );
    }
    let update = |field: &'static str| {
        let configuration = configuration.clone();
        Callback::from(move |value: f32| {
            let mut next = (*configuration).clone();
            match field {
                "thrust" => next.thrust_newtons = value.max(0.0),
                "burn" => next.burn_time_s = value.max(0.0),
                "angle" => next.max_tvc_angle_degrees = value.max(0.0),
                "rate" => next.max_tvc_rate_degrees_per_s = value.max(0.0),
                "delay" => next.tvc_delay_ms = value.max(0.0).min(u16::MAX as f32).round() as u16,
                "misalign_x" => next.tvc_misalignment_degrees[0] = value,
                "misalign_y" => next.tvc_misalignment_degrees[1] = value,
                "axial" => next.drag_coefficient_axial = value.max(0.0),
                "sideways" => next.drag_coefficient_sideways = value.max(0.0),
                "area" => next.reference_area_m2 = value.max(0.0),
                "cp" => next.center_of_pressure_offset_mm = value,
                "wind_x" => next.wind_mps[0] = value,
                "wind_z" => next.wind_mps[2] = value,
                "tilt_x" => next.initial_tilt_degrees[0] = value,
                "tilt_y" => next.initial_tilt_degrees[1] = value,
                "p" => next.pid_gains[0] = value,
                "i" => next.pid_gains[1] = value,
                "d" => next.pid_gains[2] = value,
                _ => {}
            }
            configuration.set(next);
        })
    };
    let select_thrust_profile = {
        let configuration = configuration.clone();
        Callback::from(move |event: Event| {
            let mut next = (*configuration).clone();
            next.motor_thrust_profile = match event
                .target_unchecked_into::<HtmlSelectElement>()
                .value()
                .as_str()
            {
                "klima-d3" => MotorThrustProfile::KlimaD3,
                _ => MotorThrustProfile::Constant,
            };
            configuration.set(next);
        })
    };
    let save = {
        let configuration = configuration.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::SaveSimulationConfiguration {
                    configuration: (*configuration).clone(),
                },
            )
        })
    };
    let start = {
        let trajectory = trajectory.clone();
        let configuration = configuration.clone();
        let inertia = state.inertia_configuration.clone();
        Callback::from(move |_| trajectory.set(run_rocket_preview(&inertia, &configuration)))
    };
    let start_hil = {
        let configuration = configuration.clone();
        let socket = socket.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::StartHilSimulation {
                    configuration: (*configuration).clone(),
                },
            )
        })
    };
    let show_simulation = {
        let show_hil = show_hil.clone();
        Callback::from(move |_| show_hil.set(false))
    };
    let show_hil_result = {
        let show_hil = show_hil.clone();
        Callback::from(move |_| show_hil.set(true))
    };
    let trajectory = &*trajectory;
    let trajectory_scale = trajectory.iter().fold(1.0_f32, |scale, point| {
        scale
            .max(point.position.x.abs())
            .max(point.position.z.abs())
    });
    let trajectory_path = trajectory
        .iter()
        .map(|point| {
            format!(
                "{:.1},{:.1}",
                300.0 + point.position.x / trajectory_scale * 210.0,
                235.0 - point.position.z / trajectory_scale * 210.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let burn_duration =
        RocketParameters::from_configuration(&state.inertia_configuration, &configuration, 0.002)
            .burn_duration_s();
    let cutoff = trajectory
        .iter()
        .find(|point| point.time >= burn_duration)
        .copied();
    let cutoff_trajectory = cutoff.map(|point| {
        format!(
            "{:.1},{:.1}",
            300.0 + point.position.x / trajectory_scale * 210.0,
            235.0 - point.position.z / trajectory_scale * 210.0
        )
    });
    let duration = trajectory
        .last()
        .map(|point| point.time)
        .unwrap_or(1.0)
        .max(0.001);
    let cutoff_time_x = cutoff.map(|point| 40.0 + point.time / duration * 540.0);
    let tilt = |point: &SimulationLog| {
        let up = point.rotation.rotate(Vec3::UP);
        [
            (-up.y).atan2(up.z).to_degrees(),
            up.x.atan2(up.z).to_degrees(),
        ]
    };
    let tilt_scale = trajectory.iter().fold(10.0_f32, |scale, point| {
        let value = tilt(point);
        scale.max(value[0].abs()).max(value[1].abs())
    });
    let tvc_scale = trajectory.iter().fold(
        configuration.max_tvc_angle_degrees.max(1.0),
        |scale, point| {
            scale
                .max((point.tvc[0] * configuration.max_tvc_angle_degrees).abs())
                .max((point.tvc[1] * configuration.max_tvc_angle_degrees).abs())
        },
    );
    let tilt_reference_positive = 130.0 - 5.0 / tilt_scale * 105.0;
    let tilt_reference_negative = 130.0 + 5.0 / tilt_scale * 105.0;
    let tvc_reference_positive = 130.0 - configuration.max_tvc_angle_degrees / tvc_scale * 105.0;
    let tvc_reference_negative = 130.0 + configuration.max_tvc_angle_degrees / tvc_scale * 105.0;
    let tilt_x_path = plot_series(trajectory, tilt_scale, |point| tilt(point)[0]);
    let tilt_y_path = plot_series(trajectory, tilt_scale, |point| tilt(point)[1]);
    let tvc_x_path = plot_series(trajectory, tvc_scale, |point| {
        point.tvc[0] * configuration.max_tvc_angle_degrees
    });
    let tvc_y_path = plot_series(trajectory, tvc_scale, |point| {
        point.tvc[1] * configuration.max_tvc_angle_degrees
    });
    let final_point = trajectory.last().copied();
    let hil = &*hil_result;
    let hil_average_loop_hz =
        hil.samples
            .first()
            .zip(hil.samples.last())
            .and_then(|(first, last)| {
                let elapsed_us = last.time_us.saturating_sub(first.time_us);
                (elapsed_us > 0).then(|| {
                    (hil.samples.len().saturating_sub(1) as f32 * 10.0)
                        / (elapsed_us as f32 / 1_000_000.0)
                })
            });
    let hil_duration = hil
        .samples
        .last()
        .map(|sample| sample.scheduled_time_us.max(1) as f32)
        .unwrap_or(1.0);
    let hil_scale = hil.samples.iter().fold(1.0_f32, |scale, sample| {
        scale
            .max(sample.position_m[0].abs())
            .max(sample.position_m[2].abs())
    });
    let hil_trajectory = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                300.0 + sample.position_m[0] / hil_scale * 210.0,
                235.0 - sample.position_m[2] / hil_scale * 210.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let hil_tilt_scale = hil.samples.iter().fold(5.0_f32, |scale, sample| {
        scale
            .max(sample.orientation_xy_degrees[0].abs())
            .max(sample.orientation_xy_degrees[1].abs())
    });
    let hil_tilt_x = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                40.0 + sample.scheduled_time_us as f32 / hil_duration * 540.0,
                130.0 - sample.orientation_xy_degrees[0] / hil_tilt_scale * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let hil_tilt_y = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                40.0 + sample.scheduled_time_us as f32 / hil_duration * 540.0,
                130.0 - sample.orientation_xy_degrees[1] / hil_tilt_scale * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let hil_tvc = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                40.0 + sample.scheduled_time_us as f32 / hil_duration * 540.0,
                130.0 - sample.tvc_command[0] * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let hil_actual = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                40.0 + sample.scheduled_time_us as f32 / hil_duration * 540.0,
                130.0 - sample.tvc_actual[0] * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let hil_tvc_y = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                40.0 + sample.scheduled_time_us as f32 / hil_duration * 540.0,
                130.0 - sample.tvc_command[1] * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    let hil_actual_y = hil
        .samples
        .iter()
        .map(|sample| {
            format!(
                "{:.1},{:.1}",
                40.0 + sample.scheduled_time_us as f32 / hil_duration * 540.0,
                130.0 - sample.tvc_actual[1] * 105.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    html! { <div class="simulation-page">
        <Card title="flight simulation" icon="rocket_launch">
            <p>{"Offline model. Uses saved inertia data."}</p>
            <div class="simulation-readonly"><span>{format!("Mass: {:.0} g", state.inertia_configuration.mass_g)}</span><span>{format!("COM / gimbal: {:.1} mm", state.inertia_configuration.center_of_mass_offset_mm)}</span><span>{format!("Iₓ/Iy: {:.3e} / {:.3e} kg·m²", state.inertia_configuration.moment_of_inertia_kgm2[0], state.inertia_configuration.moment_of_inertia_kgm2[1])}</span></div>
            <div class="config-grid">
                <label>{"Motor thrust profile"}<select onchange={select_thrust_profile} value={match configuration.motor_thrust_profile { MotorThrustProfile::Constant => "constant", MotorThrustProfile::KlimaD3 => "klima-d3" }}><option value="constant">{"Constant thrust"}</option><option value="klima-d3">{"Klima D3 (manufacturer curve)"}</option></select></label>
                { if configuration.motor_thrust_profile == MotorThrustProfile::Constant { html! { <><label>{"Thrust (N)"}<NumericInput value={configuration.thrust_newtons.to_string()} on_commit={update("thrust")}/></label><label>{"Burn time (s)"}<NumericInput value={configuration.burn_time_s.to_string()} on_commit={update("burn")}/></label></> } } else { html! { <div class="simulation-readonly"><span>{"Klima D3: 9.01 N peak · 6.26 s curve duration"}</span><span>{"Manufacturer RASP data via ThrustCurve.org"}</span></div> } } }
                <label>{"Max TVC angle (°)"}<NumericInput value={configuration.max_tvc_angle_degrees.to_string()} on_commit={update("angle")}/></label>
                <label>{"Max TVC rate (°/s)"}<NumericInput value={configuration.max_tvc_rate_degrees_per_s.to_string()} on_commit={update("rate")}/></label>
                <label>{"TVC delay (ms)"}<NumericInput value={configuration.tvc_delay_ms.to_string()} on_commit={update("delay")}/></label>
                <label>{"TVC misalignment X / Y (°)"}<span class="inline-inputs"><NumericInput value={configuration.tvc_misalignment_degrees[0].to_string()} on_commit={update("misalign_x")}/><NumericInput value={configuration.tvc_misalignment_degrees[1].to_string()} on_commit={update("misalign_y")}/></span></label>
                <label>{"Axial / side drag Cd"}<span class="inline-inputs"><NumericInput value={configuration.drag_coefficient_axial.to_string()} on_commit={update("axial")}/><NumericInput value={configuration.drag_coefficient_sideways.to_string()} on_commit={update("sideways")}/></span></label>
                <label>{"Reference area (m²)"}<NumericInput value={configuration.reference_area_m2.to_string()} on_commit={update("area")}/></label>
                <label>{"Center of pressure offset (mm)"}<NumericInput value={configuration.center_of_pressure_offset_mm.to_string()} on_commit={update("cp")}/></label>
                <label>{"Wind X / Z (m/s)"}<span class="inline-inputs"><NumericInput value={configuration.wind_mps[0].to_string()} on_commit={update("wind_x")}/><NumericInput value={configuration.wind_mps[2].to_string()} on_commit={update("wind_z")}/></span></label>
                <label>{"Initial tilt X / Y (°)"}<span class="inline-inputs"><NumericInput value={configuration.initial_tilt_degrees[0].to_string()} on_commit={update("tilt_x")}/><NumericInput value={configuration.initial_tilt_degrees[1].to_string()} on_commit={update("tilt_y")}/></span></label>
                <label>{"PID P / I / D"}<span class="triple-inputs"><NumericInput value={configuration.pid_gains[0].to_string()} on_commit={update("p")}/><NumericInput value={configuration.pid_gains[1].to_string()} on_commit={update("i")}/><NumericInput value={configuration.pid_gains[2].to_string()} on_commit={update("d")}/></span></label>
            </div>
            <div class="button-row"><button onclick={start}>{"Run"}</button><button class="danger" disabled={state.hil_simulation.running} onclick={start_hil}>{if state.hil_simulation.running {"HIL running…"} else {"Start HIL"}}</button><button class="save-button" onclick={save}>{"Save"}</button></div>
            <p class="safety-note">{"HIL moves TVC. Bench only; pyro off."}</p>
            if !hil_result.samples.is_empty() { <div class="status">{format!("Latest HIL log: {} samples · average physics loop {:.1} Hz · {} missed 1 kHz deadlines", hil_result.samples.len(), hil_average_loop_hz.unwrap_or(0.0), hil_result.missed_deadlines)}</div> }
        </Card>
        <Card title="results" icon="timeline"><div class="axis-row"><button class={if !*show_hil {"selected"} else {""}} onclick={show_simulation}>{"Simulation"}</button><button class={if *show_hil {"selected"} else {""}} disabled={hil.samples.is_empty()} onclick={show_hil_result}>{"HIL"}</button></div></Card>
        if *show_hil { <SimulationResultsView title="HIL" samples={hil_plot_samples(&hil.samples)} cutoff_s={Some(burn_duration)} average_hz={hil_average_loop_hz}/> } else { <SimulationResultsView title="Simulation" samples={simulation_plot_samples(trajectory)} cutoff_s={Some(burn_duration)}/> }
        if false && !hil.samples.is_empty() { <></> }
        if false && !trajectory.is_empty() { <><Card title="predicted trajectory" icon="timeline">
            <p class="plot-legend">{"X (lateral) and Z (height) use the same metres-per-pixel scale. Orange marks motor cut-off."}</p>
            <svg class="simulation-plot" viewBox="0 0 600 260" aria-label="Predicted rocket trajectory"><line x1="30" y1="235" x2="580" y2="235" class="axis"/><line x1="300" y1="15" x2="300" y2="235" class="grid"/><polyline points={trajectory_path} fill="none" stroke="#1565c0" stroke-width="3"/>{if let Some(point) = cutoff_trajectory { html! { <circle cx={point.split(',').next().unwrap_or("0").to_owned()} cy={point.split(',').nth(1).unwrap_or("0").to_owned()} r="5" fill="#ef6c00"/> } } else { Html::default() }}<text x="38" y="225">{"launch"}</text><text x="485" y="252">{format!("±{:.1} m", trajectory_scale)}</text></svg>
            if let Some(point) = final_point { <div class="inertia-summary"><span>{"Simulated duration"}</span><strong>{format!("{:.2} s", point.time)}</strong><span>{"Final height"}</span><strong>{format!("{:.2} m", point.position.z)}</strong><span>{"Final lateral displacement"}</span><strong>{format!("{:.2} m", point.position.x)}</strong></div> }
        </Card><Card title="tilt over time" icon="show_chart">
            <svg class="simulation-plot" viewBox="0 0 600 260" aria-label="Rocket tilt over time"><line x1="40" y1="130" x2="580" y2="130" class="axis"/><line x1="40" y1={tilt_reference_positive.to_string()} x2="580" y2={tilt_reference_positive.to_string()} class="reference"/><line x1="40" y1={tilt_reference_negative.to_string()} x2="580" y2={tilt_reference_negative.to_string()} class="reference"/>{if let Some(x) = cutoff_time_x { html! { <line x1={x.to_string()} y1="15" x2={x.to_string()} y2="235" class="cutoff"/> } } else { Html::default() }}<polyline points={tilt_x_path} fill="none" stroke="#1565c0" stroke-width="2"/><polyline points={tilt_y_path} fill="none" stroke="#d32f2f" stroke-width="2"/><text x="45" y="25">{"X tilt"}</text><text x="45" y="43">{"Y tilt"}</text><text x="490" y="252">{format!("±{:.1}°", tilt_scale)}</text></svg>
            <p class="plot-legend"><span class="legend-x">{"X tilt"}</span><span class="legend-y">{"Y tilt"}</span>{" · gray: typical ±5° · orange: motor cut-off"}</p>
        </Card><Card title="TVC angle over time" icon="settings_input_component">
            <svg class="simulation-plot" viewBox="0 0 600 260" aria-label="TVC angles over time"><line x1="40" y1="130" x2="580" y2="130" class="axis"/><line x1="40" y1={tvc_reference_positive.to_string()} x2="580" y2={tvc_reference_positive.to_string()} class="reference"/><line x1="40" y1={tvc_reference_negative.to_string()} x2="580" y2={tvc_reference_negative.to_string()} class="reference"/>{if let Some(x) = cutoff_time_x { html! { <line x1={x.to_string()} y1="15" x2={x.to_string()} y2="235" class="cutoff"/> } } else { Html::default() }}<polyline points={tvc_x_path} fill="none" stroke="#1565c0" stroke-width="2"/><polyline points={tvc_y_path} fill="none" stroke="#d32f2f" stroke-width="2"/><text x="45" y="25">{"X TVC"}</text><text x="45" y="43">{"Y TVC"}</text><text x="490" y="252">{format!("±{:.1}°", tvc_scale)}</text></svg>
            <p class="plot-legend"><span class="legend-x">{"X TVC"}</span><span class="legend-y">{"Y TVC"}</span>{" · gray: configured TVC limit · orange: motor cut-off"}</p>
        </Card></> } else { <Card title="simulation" icon="play_circle"><p>{"Set parameters, then Run."}</p></Card> }
    </div> }
}

#[function_component]
fn ServoOrientationCheck() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let socket = connection.send_command.clone();
    {
        let state = state.clone();
        use_effect_with_deps(
            move |message| {
                if let Some(message) = &**message {
                    if let Ok(SocketMessage::State(next)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        state.set(next);
                    }
                }
                || ()
            },
            connection.message.clone(),
        );
    }
    let toggle = {
        let socket = socket.clone();
        let state = state.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::SetServoOrientationCheck {
                    running: !state.servo_calibration.orientation_check_running,
                },
            );
        })
    };
    let calibration = &state.servo_calibration;
    html! {
        <div class="calibration-page">
            <Card title="TVC direction check" icon="screen_rotation">
                <p class="safety-note">{"Bench only. Secure rocket; pyro off."}</p>
                <p>{"Roll around Z. Nozzles should point down. Stop on wrong motion."}</p>
                <p>{"Fix reversal in Servo calibration, then save."}</p>
                <div class="status">{format!("IMU: {} · commands: X {:+.2}, Y {:+.2}", if state.imu.present { "online" } else { "offline" }, calibration.orientation_check_command[0], calibration.orientation_check_command[1])}</div>
                <button class={if calibration.orientation_check_running {"danger"} else {""}} disabled={!state.imu.present && !calibration.orientation_check_running} onclick={toggle}>{if calibration.orientation_check_running {"Stop check"} else {"Start check"}}</button>
            </Card>
        </div>
    }
}

#[function_component]
fn FlightDashboard() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let state = &*state;
    let imu = &state.imu;
    let battery = &state.battery;
    let barometer = &state.barometer;
    let pyro = &state.pyro;
    let format_axis = |values: &[f32; 3]| {
        format!(
            "X {:+.2}   Y {:+.2}   Z {:+.2}",
            values[0], values[1], values[2]
        )
    };
    html! {
        <>
            <Card title="TVC flight computer" icon="rocket_launch">
                <p>{"Live status"}</p>
            </Card>
            <PrelaunchChecklist/>
            <Card title="Battery" icon="battery_charging_full">
                <div class={if battery.present { "battery-status" } else { "battery-status offline" }}>{if battery.present { "LIVE — MAX17048 FUEL GAUGE" } else { "OFFLINE — CHECK I²C FUEL GAUGE" }}</div>
                <div class="battery-readings"><div><span>{"VOLTAGE"}</span><strong>{if battery.present { format!("{:.3} V", battery.voltage) } else { "—".to_owned() }}</strong></div><div><span>{"STATE OF CHARGE"}</span><strong>{if battery.present { format!("{:.0}%", battery.soc) } else { "—".to_owned() }}</strong></div></div>
            </Card>
            <Card title="Pyro continuity" icon="electrical_services">
                <div class="battery-readings">
                    <div><span>{"CHANNEL 1"}</span><strong>{if pyro.channel1.continuity { "CONTINUITY" } else { "OPEN" }}</strong><small>{format!("{:.3} V test", pyro.channel1.test_voltage)}</small></div>
                    <div><span>{"CHANNEL 2"}</span><strong>{if pyro.channel2.continuity { "CONTINUITY" } else { "OPEN" }}</strong><small>{format!("{:.3} V test", pyro.channel2.test_voltage)}</small></div>
                </div>
            </Card>
            <Card title="BMP280 barometer" icon="speed">
                <div class={if barometer.present { "imu-status" } else { "imu-status offline" }}>{if barometer.present { "ONLINE — 100 HZ ACQUISITION" } else { "OFFLINE — CHECK I²C SENSOR" }}</div>
                <div class="battery-readings">
                    <div><span>{"ALTITUDE"}</span><strong>{if barometer.present { format!("{:.1} m", barometer.altitude) } else { "—".to_owned() }}</strong></div>
                    <div><span>{"PRESSURE"}</span><strong>{if barometer.present { format!("{:.2} hPa", barometer.pressure_hpa) } else { "—".to_owned() }}</strong></div>
                    <div><span>{"TEMPERATURE"}</span><strong>{if barometer.present { format!("{:.1} °C", barometer.temperature) } else { "—".to_owned() }}</strong></div>
                </div>
                <small>{format!("{} successful samples · average {:.1} Hz", barometer.sample_count, barometer.average_rate_hz)}</small>
            </Card>
            <Card title="ICM-42688-P inertial measurement" icon="sensors">
                <div class="imu-status">{if imu.present { "ONLINE — 100 HZ ACQUISITION" } else { "OFFLINE — CHECK I²C SENSOR" }}</div>
                <div class="imu-reading"><span>{"ACCELERATION (M/S²)"}</span><code>{format_axis(&imu.acceleration_mps2)}</code></div>
                <div class="imu-reading"><span>{"ANGULAR VELOCITY (RAD/S)"}</span><code>{format_axis(&imu.angular_velocity_radps)}</code></div>
                <small>{format!("{} successful samples · average {:.1} Hz", imu.sample_count, imu.average_rate_hz)}</small>
            </Card>
        </>
    }
}

#[function_component]
fn WifiSettings() -> Html {
    let connection = use_context::<AppConnection>().expect("app connection context");
    let state = connection.state.clone();
    let ssid = use_state(String::new);
    let password = use_state(String::new);
    let saved = use_state(|| false);
    let restart_armed = use_state(|| false);
    let socket = connection.send_command.clone();
    {
        let state = state.clone();
        let ssid = ssid.clone();
        use_effect_with_deps(
            move |message| {
                if let Some(message) = &**message {
                    if let Ok(SocketMessage::State(next)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
                        if ssid.is_empty() && !next.configured_wifi_ssid.is_empty() {
                            ssid.set(next.configured_wifi_ssid.clone());
                        }
                        state.set(next);
                    }
                }
                || ()
            },
            connection.message.clone(),
        );
    }
    let update_ssid = {
        let ssid = ssid.clone();
        let saved = saved.clone();
        Callback::from(move |event: InputEvent| {
            ssid.set(event.target_unchecked_into::<HtmlInputElement>().value());
            saved.set(false);
        })
    };
    let update_password = {
        let password = password.clone();
        let saved = saved.clone();
        Callback::from(move |event: InputEvent| {
            password.set(event.target_unchecked_into::<HtmlInputElement>().value());
            saved.set(false);
        })
    };
    let save = {
        let ssid = ssid.clone();
        let password = password.clone();
        let socket = socket.clone();
        let saved = saved.clone();
        Callback::from(move |_| {
            send_command(
                &socket,
                Command::SetWifi {
                    ssid: (*ssid).clone(),
                    password: (*password).clone(),
                },
            );
            password.set(String::new());
            saved.set(true);
        })
    };
    let restart_flight_controller = {
        let socket = socket.clone();
        let restart_armed = restart_armed.clone();
        Callback::from(move |_| {
            if *restart_armed {
                send_command(&socket, Command::Reset);
                restart_armed.set(false);
            } else {
                restart_armed.set(true);
            }
        })
    };
    let active = &state.wifi_state;
    html! {
        <Card title="Wi-Fi" icon="wifi">
            <p>{format!("Active connection: {} ({})", active.credentials.ssid, if active.connection_type == WifiConnectionType::ConnectToExternal { "network" } else { "access point" })}</p>
            <p>{"Saved network is used after reboot; fallback is LRC-wifi."}</p>
            <div class="config-grid"><label>{"Network name (SSID)"}<input value={(*ssid).clone()} oninput={update_ssid}/></label><label>{"Password"}<input type="password" value={(*password).clone()} oninput={update_password}/></label></div>
            <button class="save-button" onclick={save} disabled={ssid.is_empty()}>{"Save"}</button>
            if *saved { <p>{"Saved. Reboot to connect."}</p> }
            <Card title="flight controller" icon="restart_alt">
                <p>{"Saved settings remain."}</p>
                <button class="danger" onclick={restart_flight_controller}>{if *restart_armed {"Confirm restart"} else {"Restart"}}</button>
            </Card>
        </Card>
    }
}

#[function_component]
fn App() -> Html {
    let tab = use_state(|| 0_usize);
    let state = use_state_eq(State::default);
    let message = use_state_eq(|| None::<String>);
    let socket = use_websocket("ws://lrc.local/ws".to_owned());
    {
        let state = state.clone();
        let message = message.clone();
        use_effect_with_deps(
            move |incoming| {
                if let Some(incoming) = &**incoming {
                    if let Ok(SocketMessage::State(next)) =
                        serde_json::from_str::<SocketMessage>(incoming)
                    {
                        state.set(next);
                    }
                    message.set(Some(incoming.clone()));
                }
                || ()
            },
            socket.message.clone(),
        );
    }
    let send_command = {
        let socket = socket.clone();
        Callback::from(move |command| socket.send(serde_json::to_string(&command).unwrap()))
    };
    let connection = AppConnection {
        state,
        message,
        send_command,
    };
    let activated = {
        let tab = tab.clone();
        Callback::from(move |id| tab.set(id))
    };
    html! { <ContextProvider<AppConnection> context={connection}><div class="content-frame"><div class="content-root"><MatTabBar onactivated={activated}><MatTab min_width=true icon="dashboard"/><MatTab min_width=true icon="school"/><MatTab min_width=true icon="settings"/></MatTabBar><TabPage id=0 current_id={*tab}><FlightDashboard/></TabPage><TabPage id=1 current_id={*tab}><RocketOnboarding/></TabPage><TabPage id=2 current_id={*tab}><WifiSettings/></TabPage></div></div></ContextProvider<AppConnection>> }
}
fn main() {
    yew::Renderer::<App>::new().render();
}
