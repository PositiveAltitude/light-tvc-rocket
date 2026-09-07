mod components;

use crate::components::*;
use gloo::timers::callback::Interval;
use light_robot_core_api::*;
use material_yew::{MatTab, MatTabBar};
use reqwasm::http::Request;
use serde::de::DeserializeOwned;
use wasm_bindgen_futures::spawn_local;
use web_sys::HtmlInputElement;
use yew::prelude::*;
use yew_hooks::prelude::*;

const COMMAND_URI: &str = "http://lrc.local/command";
const TEST_COLORS: [&str; 5] = ["#1565c0", "#d32f2f", "#2e7d32", "#7b1fa2", "#ef6c00"];

fn send_command(socket: &UseWebSocketHandle, command: Command) {
    socket.send(serde_json::to_string(&command).unwrap());
}

async fn fetch<T: DeserializeOwned>(url: &str) -> Option<T> {
    Request::get(url).send().await.ok()?.json::<T>().await.ok()
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

#[function_component]
fn ServoCalibration() -> Html {
    let state = use_state_eq(State::default);
    let axis = use_state(|| ServoAxis::X);
    let config = use_state(ServoConfiguration::default);
    let enabled = use_state(|| false);
    let manual_position = use_state(|| 0.0_f32);
    let tests = use_state(Vec::<StoredTest>::new);
    let testing = use_state(|| false);
    let selected_device = use_state(|| 0_usize);
    let draft = use_state(|| ConfigDraft::from_config(&ServoConfiguration::default()));
    let socket = use_websocket("ws://lrc.local/ws".to_owned());
    {
        let state = state.clone();
        let tests = tests.clone();
        let testing = testing.clone();
        use_effect_with_deps(
            move |message| {
                if let Some(message) = &**message {
                    if let Ok(SocketMessage::State(new_state)) =
                        serde_json::from_str::<SocketMessage>(message)
                    {
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
            socket.message.clone(),
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
        Callback::from(move |event: InputEvent| {
            let text = event.target_unchecked_into::<HtmlInputElement>().value();
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
            let value = text.parse::<f32>();
            let Ok(value) = value else {
                return;
            };
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
        let socket = socket.clone();
        Callback::from(move |_| send_command(&socket, Command::SaveServoConfigurations))
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
        html! { <p>{"No servos were detected at startup. Check CAN power, wiring, and termination, then reboot."}</p> }
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
      <Card title="servo calibration" icon="tune"><p class="safety-note">{"Bench use only: restrain the vehicle and keep clear of the TVC mechanism before enabling a motor."}</p><div class="axis-row"><span>{"Servo:"}</span><button class={if *axis == ServoAxis::X {"selected"} else {""}} onclick={select_axis(ServoAxis::X)}>{"X axis"}</button><button class={if *axis == ServoAxis::Y {"selected"} else {""}} onclick={select_axis(ServoAxis::Y)}>{"Y axis"}</button></div>{device_picker}<div class="status">{format!("{}: {}; encoder {}", axis_name(*axis), if detected {"detected"} else {"not detected"}, position)}<br/>{match assigned_device { Some(device) => format!("Assigned: {}", device_name(device)), None => "No device assigned to this axis".into() }}</div></Card>
      <Card title="configuration" icon="settings"><div class="config-grid">
        <label>{"Encoder zero (0–16383)"}<input type="number" min="0" max="16383" step="1" value={draft.zero.clone()} oninput={update("zero")}/></label><label>{"Max turn (° at ±1.0)"}<input type="number" min="0.01" step="0.1" value={draft.turn.clone()} oninput={update("turn")}/></label><label>{"Position P"}<input type="number" step="0.001" value={draft.p.clone()} oninput={update("p")}/></label><label>{"Position I"}<input type="number" step="0.001" value={draft.i.clone()} oninput={update("i")}/></label><label>{"Position D"}<input type="number" step="0.001" value={draft.d.clone()} oninput={update("d")}/></label><label>{"Duty limit (0–1)"}<input type="number" min="0" max="1" step="0.01" value={draft.limit.clone()} oninput={update("limit")}/></label><label class="checkbox-label"><input type="checkbox" checked={config.reverse_motor} onchange={toggle_reverse}/>{"Reverse motor direction"}</label>
      </div><div class="button-row"><button onclick={apply}>{"Apply configuration"}</button><button onclick={capture_zero}>{"Capture current position as zero"}</button></div></Card>
      <Card title="manual test" icon="gamepad"><button class={if *enabled {"danger"} else {""}} onclick={toggle}>{if *enabled {"Motor ON — holding position"} else {"Motor OFF — freewheeling"}}</button><label class="slider-label">{format!("Command: {:.2}", *manual_position)}<input type="range" min="-1" max="1" step="0.01" value={manual_position.to_string()} disabled={!*enabled} oninput={manual} onchange={send_manual}/></label></Card>
      <Card title="automatic step-response test" icon="show_chart"><p>{"Moves to zero, settles for 2 s, then steps to +0.75. The flight computer records at 1000 Hz for 250 ms."}</p><button disabled={*testing} onclick={start_test}>{if *testing {"Capturing…"} else {"Run performance test"}}</button><TestPlot tests={(*tests).clone()}/><TestTable tests={tests.clone()} config={config.clone()}/></Card><button class="save-button" onclick={save}>{"Save current configurations to flash"}</button>
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

#[function_component]
fn FlightDashboard() -> Html {
    let state = use_state_eq(State::default);
    {
        let state = state.clone();
        use_effect_with_deps(
            move |_| {
                let refresh = move || {
                    let state = state.clone();
                    spawn_local(async move {
                        if let Some(next_state) = fetch::<State>("/state").await {
                            state.set(next_state);
                        }
                    });
                };
                refresh();
                let interval = Interval::new(250, refresh);
                move || drop(interval)
            },
            (),
        );
    }
    let state = &*state;
    let imu = &state.imu;
    let format_axis = |values: &[f32; 3]| {
        format!(
            "X {:+.2}   Y {:+.2}   Z {:+.2}",
            values[0], values[1], values[2]
        )
    };
    html! {
        <>
            <Card title="TVC flight computer" icon="rocket_launch">
                <p>{"Live flight-sensor status. IMU values are refreshed in the dashboard at 4 Hz."}</p>
            </Card>
            <Card title="ICM-42688-P inertial measurement" icon="sensors">
                <div class="imu-status">{if imu.present { "ONLINE — 100 HZ ACQUISITION" } else { "OFFLINE — CHECK I²C SENSOR" }}</div>
                <div class="imu-reading"><span>{"ACCELERATION (M/S²)"}</span><code>{format_axis(&imu.acceleration_mps2)}</code></div>
                <div class="imu-reading"><span>{"ANGULAR VELOCITY (RAD/S)"}</span><code>{format_axis(&imu.angular_velocity_radps)}</code></div>
                <small>{format!("{} successful samples", imu.sample_count)}</small>
            </Card>
        </>
    }
}

#[function_component]
fn App() -> Html {
    let tab = use_state(|| 0_usize);
    let activated = {
        let tab = tab.clone();
        Callback::from(move |id| tab.set(id))
    };
    html! { <div class="content-frame"><div class="content-root"><MatTabBar onactivated={activated}><MatTab min_width=true icon="dashboard"/><MatTab min_width=true icon="tune"/><MatTab min_width=true icon="settings"/></MatTabBar><TabPage id=0 current_id={*tab}><FlightDashboard/></TabPage><TabPage id=1 current_id={*tab}><ServoCalibration/></TabPage><TabPage id=2 current_id={*tab}><p>{"System settings"}</p></TabPage></div></div> }
}
fn main() {
    yew::Renderer::<App>::new().render();
}
