mod components;

use crate::components::*;
use gloo::timers::callback::{Interval, Timeout};
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
        if result.samples.len() < 8 || mass_g <= 0.0 || offset_mm <= 0.0 { return None; }
        let values = result.samples.iter().map(|sample| if axis == InertiaAxis::X { sample.gyro_x_radps } else { sample.gyro_y_radps } as f64).collect::<Vec<_>>();
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let bias = (min + max) * 0.5;
        let omega_amplitude = ((max - min) * 0.5).max(0.01);
        let crossings = result.samples.windows(2).filter_map(|pair| {
            let a = (if axis == InertiaAxis::X { pair[0].gyro_x_radps } else { pair[0].gyro_y_radps }) as f64 - bias;
            let b = (if axis == InertiaAxis::X { pair[1].gyro_x_radps } else { pair[1].gyro_y_radps }) as f64 - bias;
            if a.signum() != b.signum() && (a - b).abs() > f64::EPSILON {
                let fraction = a.abs() / (a.abs() + b.abs());
                Some((pair[0].time_us as f64 + fraction * (pair[1].time_us - pair[0].time_us) as f64) / 1_000_000.0)
            } else { None }
        }).collect::<Vec<_>>();
        let half_period = crossings.windows(2).map(|pair| pair[1] - pair[0]).filter(|period| *period > 0.001).sum::<f64>() / (crossings.len().saturating_sub(1).max(1) as f64);
        let frequency_hz = if crossings.len() > 1 { 1.0 / (2.0 * half_period) } else { 1.0 };
        let natural_omega = (2.0 * std::f64::consts::PI * frequency_hz).max(0.1);
        let mass_kg = mass_g / 1000.0;
        let offset_m = offset_mm / 1000.0;
        let inertia = (mass_kg * 9.80665 * offset_m / (natural_omega * natural_omega)).max(1e-5);
        // A zero crossing identifies phase only modulo π: either the positive
        // or negative angular-velocity half-cycle could pass through it. Try
        // both branches against the full capture before optimization.
        let phase = crossings.first().map(|time| -natural_omega * *time).unwrap_or(0.0);
        let envelope_peaks = values.windows(3).enumerate().filter_map(|(index, window)| {
            let center = (window[1] - bias).abs();
            if center >= (window[0] - bias).abs() && center >= (window[2] - bias).abs() {
                Some((index + 1, center))
            } else { None }
        }).collect::<Vec<_>>();
        let initial_damping = match (envelope_peaks.first(), envelope_peaks.last()) {
            (Some((first_index, first_peak)), Some((last_index, last_peak))) if last_index > first_index && *last_peak > 0.001 => {
                let elapsed = (result.samples[*last_index].time_us - result.samples[*first_index].time_us) as f64 / 1_000_000.0;
                (2.0 * (first_peak / last_peak).ln() / elapsed.max(0.001)).clamp(0.001, 5.0)
            }
            _ => 0.02,
        };
        let mut fit = Self {
            samples: result.samples.clone(), axis, mass_kg, offset_m,
            coordinates: [inertia.ln(), (omega_amplitude / natural_omega).max(0.005).ln(), phase, bias, initial_damping.ln()],
            cost: f64::INFINITY, learning_rate: 0.025, iterations: 0, finished: false,
        };
        let mut opposite_phase = fit.coordinates;
        opposite_phase[2] += std::f64::consts::PI;
        let direct_cost = fit.cost_for(fit.coordinates);
        let opposite_cost = fit.cost_for(opposite_phase);
        if opposite_cost < direct_cost { fit.coordinates = opposite_phase; fit.cost = opposite_cost; }
        else { fit.cost = direct_cost; }
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
        self.samples.iter().map(|sample| {
            let time = sample.time_us as f64 / 1_000_000.0;
            let dt = (time - previous_time).max(0.0); previous_time = time;
            let acceleration = |angle: f64, velocity: f64| -(self.mass_kg * 9.80665 * self.offset_m / inertia) * angle.sin() - damping * velocity;
            let k1_theta = omega; let k1_omega = acceleration(theta, omega);
            let k2_theta = omega + k1_omega * dt * 0.5; let k2_omega = acceleration(theta + k1_theta * dt * 0.5, omega + k1_omega * dt * 0.5);
            let k3_theta = omega + k2_omega * dt * 0.5; let k3_omega = acceleration(theta + k2_theta * dt * 0.5, omega + k2_omega * dt * 0.5);
            let k4_theta = omega + k3_omega * dt; let k4_omega = acceleration(theta + k3_theta * dt, omega + k3_omega * dt);
            theta += dt * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta) / 6.0;
            omega += dt * (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega) / 6.0;
            omega + bias
        }).collect()
    }

    fn cost_for(&self, coordinates: [f64; 5]) -> f64 {
        let simulated = self.simulated(coordinates);
        let cost = simulated.iter().zip(&self.samples).map(|(prediction, sample)| {
            let measured = if self.axis == InertiaAxis::X { sample.gyro_x_radps } else { sample.gyro_y_radps } as f64;
            let residual = prediction - measured;
            if residual.is_finite() { residual * residual } else { 1e12 }
        }).sum::<f64>() / simulated.len().max(1) as f64;
        if cost.is_finite() { cost.min(1e12) } else { 1e12 }
    }

    fn iterate(&mut self) {
        if self.finished { return; }
        let mut gradient = [0.0; 5];
        for index in 0..5 {
            let step = if index == 3 { 0.001 } else { 0.002 };
            let mut plus = self.coordinates; plus[index] += step;
            let mut minus = self.coordinates; minus[index] -= step;
            gradient[index] = (self.cost_for(plus) - self.cost_for(minus)) / (2.0 * step);
        }
        let norm = gradient.iter().map(|value| value * value).sum::<f64>().sqrt();
        if self.iterations >= 900 { self.finished = true; return; }
        if !norm.is_finite() {
            // Keep the fit visible and retry with a smaller step rather than
            // silently reporting completion after a single unstable trial.
            self.learning_rate *= 0.5;
            self.iterations += 1;
            if self.learning_rate < 1e-6 && self.iterations >= 120 { self.finished = true; }
            return;
        }
        if norm < 1e-10 {
            self.iterations += 1;
            if self.iterations >= 120 { self.finished = true; }
            return;
        }
        let mut candidate = self.coordinates;
        for index in 0..5 { candidate[index] -= self.learning_rate * gradient[index] / norm; }
        let candidate_cost = self.cost_for(candidate);
        if candidate_cost < self.cost {
            self.coordinates = candidate; self.cost = candidate_cost; self.learning_rate = (self.learning_rate * 1.04).min(0.1);
        } else { self.learning_rate *= 0.5; }
        self.iterations += 1;
        if (self.learning_rate < 1e-5 && self.iterations >= 120) || self.iterations >= 900 { self.finished = true; }
    }
}

#[function_component]
fn MomentOfInertia() -> Html {
    let state = use_state_eq(State::default);
    let result = use_state_eq(InertiaCaptureResult::default);
    let configuration = use_state_eq(InertiaConfiguration::default);
    let configuration_loaded = use_state(|| false);
    let fit = use_state_eq(|| None::<PendulumFit>);
    let published_fit_iteration = use_state(|| None::<u32>);
    let socket = use_websocket("ws://lrc.local/ws".to_owned());
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
                    if let Ok(SocketMessage::State(next_state)) = serde_json::from_str::<SocketMessage>(message) {
                        if !*configuration_loaded {
                            let mut loaded = next_state.inertia_configuration.clone();
                            loaded.moment_of_inertia_kgm2[2] = cylinder_z_inertia(&loaded);
                            configuration.set(loaded);
                            configuration_loaded.set(true);
                        }
                        state.set(next_state);
                    }
                    if let Ok(SocketMessage::InertiaCaptureResult(next_result)) = serde_json::from_str::<SocketMessage>(message) {
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
            socket.message.clone(),
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
                } else { None };
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
        let fit = fit.clone(); let configuration = configuration.clone(); let socket = socket.clone(); let published = published_fit_iteration.clone();
        let completed_iteration = fit.as_ref().filter(|model| model.finished).map(|model| model.iterations);
        use_effect_with_deps(move |completed_iteration| {
            if let Some(iteration) = *completed_iteration {
                if *published != Some(iteration) {
                    if let Some(model) = fit.as_ref() {
                        let mut next = (*configuration).clone();
                        let index = if model.axis == InertiaAxis::X { 0 } else { 1 };
                        next.moment_of_inertia_kgm2[index] = model.parameters(model.coordinates).0 as f32;
                        next.moment_of_inertia_kgm2[2] = cylinder_z_inertia(&next);
                        configuration.set(next.clone());
                        send_command(&socket, Command::SetInertiaConfiguration { configuration: next });
                        published.set(Some(iteration));
                    }
                }
            }
            || ()
        }, completed_iteration);
    }
    let update_mass = { let configuration = configuration.clone(); Callback::from(move |event: InputEvent| { let mut next = (*configuration).clone(); next.mass_g = event.target_unchecked_into::<HtmlInputElement>().value().parse().unwrap_or(next.mass_g).max(0.0); next.moment_of_inertia_kgm2[2] = cylinder_z_inertia(&next); configuration.set(next); }) };
    let update_offset = { let configuration = configuration.clone(); Callback::from(move |event: InputEvent| { let mut next = (*configuration).clone(); next.center_of_mass_offset_mm = event.target_unchecked_into::<HtmlInputElement>().value().parse().unwrap_or(next.center_of_mass_offset_mm).max(0.0); configuration.set(next); }) };
    let update_inertia = |index: usize| { let configuration = configuration.clone(); Callback::from(move |event: InputEvent| { let mut next = (*configuration).clone(); next.moment_of_inertia_kgm2[index] = event.target_unchecked_into::<HtmlInputElement>().value().parse().unwrap_or(next.moment_of_inertia_kgm2[index]).max(0.0); configuration.set(next); }) };
    let save_configuration = { let configuration = configuration.clone(); let socket = socket.clone(); Callback::from(move |_| send_command(&socket, Command::SaveInertiaConfiguration { configuration: (*configuration).clone() })) };
    let peak = result.samples.iter().fold(0.1_f32, |peak, sample| {
        peak.max(sample.gyro_x_radps.abs()).max(sample.gyro_y_radps.abs())
    });
    let duration_us = result.samples.last().map(|sample| sample.time_us.max(1)).unwrap_or(5_000_000);
    let points = |x: bool| result.samples.iter().map(|sample| {
        let horizontal = sample.time_us as f32 / duration_us as f32 * 600.0;
        let value = if x { sample.gyro_x_radps } else { sample.gyro_y_radps };
        format!("{:.1},{:.1}", horizontal, 130.0 - value / peak * 105.0)
    }).collect::<Vec<_>>().join(" ");
    let fitted = fit.as_ref().map(|model| model.simulated(model.coordinates)).unwrap_or_default();
    let fitted_points = result.samples.iter().zip(fitted.iter()).map(|(sample, value)| {
        let horizontal = sample.time_us as f32 / duration_us as f32 * 600.0;
        format!("{:.1},{:.1}", horizontal, 130.0 - *value as f32 / peak * 105.0)
    }).collect::<Vec<_>>().join(" ");
    let fit_parameters = fit.as_ref().map(|model| model.parameters(model.coordinates));
    html! {
        <div class="inertia-page">
            <Card title="moment of inertia measurement" icon="rotate_right">
                <p class="safety-note">{"Bench use only: secure the rocket in the pendulum fixture, keep clear of its swing path, and do not arm pyro or TVC outputs."}</p>
                <p>{"Start a five-second gyro capture after releasing the pendulum."}</p>
                <div class="config-grid"><label>{"Rocket mass (g)"}<input type="number" min="1" step="1" value={configuration.mass_g.to_string()} oninput={update_mass}/></label><label>{"Center of mass position (mm)"}<input type="number" min="1" step="1" value={configuration.center_of_mass_offset_mm.to_string()} oninput={update_offset}/></label><label>{"X moment (kg·m²)"}<input type="number" step="any" value={format!("{:.5e}", configuration.moment_of_inertia_kgm2[0])} oninput={update_inertia(0)}/></label><label>{"Y moment (kg·m²)"}<input type="number" step="any" value={format!("{:.5e}", configuration.moment_of_inertia_kgm2[1])} oninput={update_inertia(1)}/></label><label>{"Z moment — 50 mm cylinder estimate (kg·m²)"}<input type="text" readonly=true value={format!("{:.5e}", configuration.moment_of_inertia_kgm2[2])}/></label></div>
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
    let page = use_state(|| 0_usize);
    let show_calibration = { let page = page.clone(); Callback::from(move |_| page.set(0)) };
    let show_inertia = { let page = page.clone(); Callback::from(move |_| page.set(1)) };
    html! {
        <>
            <Card title="rocket onboarding" icon="school">
                <div class="axis-row"><button class={if *page == 0 {"selected"} else {""}} onclick={show_calibration}>{"Servo calibration"}</button><button class={if *page == 1 {"selected"} else {""}} onclick={show_inertia}>{"Moment of inertia"}</button></div>
            </Card>
            if *page == 0 { <ServoCalibration/> } else { <MomentOfInertia/> }
        </>
    }
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
                <small>{format!("{} successful samples · average {:.1} Hz", imu.sample_count, imu.average_rate_hz)}</small>
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
    html! { <div class="content-frame"><div class="content-root"><MatTabBar onactivated={activated}><MatTab min_width=true icon="dashboard"/><MatTab min_width=true icon="school"/><MatTab min_width=true icon="settings"/></MatTabBar><TabPage id=0 current_id={*tab}><FlightDashboard/></TabPage><TabPage id=1 current_id={*tab}><RocketOnboarding/></TabPage><TabPage id=2 current_id={*tab}><p>{"System settings"}</p></TabPage></div></div> }
}
fn main() {
    yew::Renderer::<App>::new().render();
}
