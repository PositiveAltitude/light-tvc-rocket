mod components;

use crate::components::*;
use gloo::timers::callback::Timeout;
use light_robot_core_api::*;
use material_yew::{MatTab, MatTabBar};
use reqwasm::http::Request;
use serde::de::DeserializeOwned;
use wasm_bindgen_futures::spawn_local;
use web_sys::HtmlInputElement;
use yew::prelude::*;

const COMMAND_URI: &str = "http://lrc.local/command";

fn send_command(command: Command) {
    spawn_local(async move {
        let _ = Request::post(COMMAND_URI)
            .header("Content-Type", "application/json")
            .body(serde_json::to_string(&command).unwrap())
            .send()
            .await;
    });
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

fn axis_name(axis: ServoAxis) -> &'static str {
    match axis {
        ServoAxis::X => "X axis",
        ServoAxis::Y => "Y axis",
    }
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
    {
        let state = state.clone();
        use_effect_with_deps(
            move |_| {
                spawn_local(async move {
                    if let Some(s) = fetch::<State>("http://lrc.local/state").await {
                        state.set(s);
                    }
                });
                || ()
            },
            (),
        );
    }
    let select_axis = |new_axis: ServoAxis| {
        let axis = axis.clone();
        let config = config.clone();
        let enabled = enabled.clone();
        let state = state.clone();
        Callback::from(move |_| {
            axis.set(new_axis);
            config.set(if new_axis == ServoAxis::X {
                state.servo_calibration.x.clone()
            } else {
                state.servo_calibration.y.clone()
            });
            enabled.set(if new_axis == ServoAxis::X {
                state.servo_calibration.x_enabled
            } else {
                state.servo_calibration.y_enabled
            });
        })
    };
    let update = |field: &'static str| {
        let config = config.clone();
        Callback::from(move |event: InputEvent| {
            let value = event
                .target_unchecked_into::<HtmlInputElement>()
                .value()
                .parse::<f32>()
                .unwrap_or(0.0);
            let mut c = (*config).clone();
            match field {
                "turn" => c.max_turn_degrees = value,
                "p" => c.position_p = value,
                "i" => c.position_i = value,
                "d" => c.position_d = value,
                "vp" => c.velocity_p = value,
                "vi" => c.velocity_i = value,
                "limit" => c.duty_cycle_limit = value.clamp(0.0, 1.0),
                "vel" => c.max_velocity = value,
                _ => {}
            };
            config.set(c);
        })
    };
    let apply = {
        let axis = axis.clone();
        let config = config.clone();
        Callback::from(move |_| {
            send_command(Command::SetServoConfiguration {
                axis: *axis,
                configuration: (*config).clone(),
            })
        })
    };
    let detect = Callback::from(move |_| send_command(Command::DetectServos));
    let capture_zero = {
        let axis = axis.clone();
        Callback::from(move |_| send_command(Command::CaptureServoZero { axis: *axis }))
    };
    let toggle = {
        let axis = axis.clone();
        let enabled = enabled.clone();
        Callback::from(move |_| {
            let on = !*enabled;
            enabled.set(on);
            send_command(Command::SetServoEnabled {
                axis: *axis,
                enabled: on,
            });
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
        Callback::from(move |event: Event| {
            let p = event
                .target_unchecked_into::<HtmlInputElement>()
                .value()
                .parse::<f32>()
                .unwrap_or(0.0)
                .clamp(-1.0, 1.0);
            if *enabled {
                send_command(Command::SetServoPosition {
                    axis: *axis,
                    position: p,
                });
            }
        })
    };
    let start_test = {
        let axis = axis.clone();
        let tests = tests.clone();
        let testing = testing.clone();
        Callback::from(move |_| {
            testing.set(true);
            send_command(Command::StartServoPerformanceTest { axis: *axis });
            let tests = tests.clone();
            let testing = testing.clone();
            Timeout::new(3100, move || {
                spawn_local(async move {
                    if let Some(result) =
                        fetch::<ServoTestResult>("http://lrc.local/test_result").await
                    {
                        let colors = ["#1565c0", "#d32f2f", "#2e7d32", "#7b1fa2", "#ef6c00"];
                        let mut all = (*tests).clone();
                        let color = colors[all.len() % colors.len()];
                        all.push(StoredTest {
                            result,
                            visible: true,
                            color,
                        });
                        tests.set(all);
                    };
                    testing.set(false);
                })
            })
            .forget();
        })
    };
    let save = Callback::from(move |_| send_command(Command::SaveServoConfigurations));
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
    html! { <div class="calibration-page">
      <Card title="servo calibration" icon="tune"><p class="safety-note">{"Bench use only: restrain the vehicle and keep clear of the TVC mechanism before enabling a motor."}</p><div class="axis-row"><span>{"Servo:"}</span><button class={if *axis == ServoAxis::X {"selected"} else {""}} onclick={select_axis(ServoAxis::X)}>{"X axis"}</button><button class={if *axis == ServoAxis::Y {"selected"} else {""}} onclick={select_axis(ServoAxis::Y)}>{"Y axis"}</button><button onclick={detect}>{"Detect servos"}</button></div><div class="status">{format!("{}: {}; encoder {}", axis_name(*axis), if detected {"detected"} else {"not detected"}, position)}</div></Card>
      <Card title="configuration" icon="settings"><div class="config-grid">
        <label>{"Max turn (° at ±1.0)"}<input type="number" step="0.1" value={config.max_turn_degrees.to_string()} oninput={update("turn")}/></label><label>{"Position P"}<input type="number" step="0.001" value={config.position_p.to_string()} oninput={update("p")}/></label><label>{"Position I"}<input type="number" step="0.001" value={config.position_i.to_string()} oninput={update("i")}/></label><label>{"Position D"}<input type="number" step="0.001" value={config.position_d.to_string()} oninput={update("d")}/></label><label>{"Velocity P"}<input type="number" step="0.001" value={config.velocity_p.to_string()} oninput={update("vp")}/></label><label>{"Velocity I"}<input type="number" step="0.001" value={config.velocity_i.to_string()} oninput={update("vi")}/></label><label>{"Duty limit (0–1)"}<input type="number" min="0" max="1" step="0.01" value={config.duty_cycle_limit.to_string()} oninput={update("limit")}/></label><label>{"Max velocity"}<input type="number" step="1" value={config.max_velocity.to_string()} oninput={update("vel")}/></label>
      </div><div class="button-row"><button onclick={apply}>{"Apply configuration"}</button><button onclick={capture_zero}>{"Capture current position as zero"}</button></div></Card>
      <Card title="manual test" icon="gamepad"><button class={if *enabled {"danger"} else {""}} onclick={toggle}>{if *enabled {"Motor ON — holding position"} else {"Motor OFF — freewheeling"}}</button><label class="slider-label">{format!("Command: {:.2}", *manual_position)}<input type="range" min="-1" max="1" step="0.01" value={manual_position.to_string()} disabled={!*enabled} oninput={manual} onchange={send_manual}/></label></Card>
      <Card title="automatic step-response test" icon="show_chart"><p>{"Moves to zero, settles for 2 s, then steps to +0.75. The flight computer records at 1000 Hz for 500 ms."}</p><button disabled={*testing} onclick={start_test}>{if *testing {"Capturing…"} else {"Run performance test"}}</button><TestPlot tests={(*tests).clone()}/><TestTable tests={tests.clone()} config={config.clone()}/></Card><button class="save-button" onclick={save}>{"Save current configurations to flash"}</button>
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
                    s.time_ms as f32 / 500.0 * 600.0,
                    180.0 - (s.position + 1.0) * 80.0
                )
            })
            .collect::<Vec<_>>()
            .join(" ");
        html! { <polyline points={points} fill="none" stroke={t.color} stroke-width="2"/> }
    });
    html! { <svg class="response-plot" viewBox="0 0 600 200"><line x1="0" y1="180" x2="600" y2="180" class="axis"/><line x1="0" y1="100" x2="600" y2="100" class="grid"/><line x1="0" y1="40" x2="600" y2="40" class="grid"/>{for lines}<text x="4" y="18">{"+1.0"}</text><text x="4" y="198">{"-1.0"}</text><text x="545" y="198">{"500 ms"}</text></svg> }
}

#[derive(Properties, PartialEq)]
struct TestTableProps {
    tests: UseStateHandle<Vec<StoredTest>>,
    config: UseStateHandle<ServoConfiguration>,
}
#[function_component]
fn TestTable(props: &TestTableProps) -> Html {
    html! { <table class="test-table"><thead><tr><th>{"Test"}</th><th>{"Axis"}</th><th>{"P / I / D"}</th><th>{"Samples"}</th><th>{"Actions"}</th></tr></thead><tbody>{for props.tests.iter().enumerate().map(|(index, test)| { let toggle = props.tests.clone(); let delete = props.tests.clone(); let config = props.config.clone(); let saved = test.clone(); html! { <tr><td><span class="color-dot" style={format!("background:{}", test.color)}></span>{index + 1}</td><td>{axis_name(test.result.axis)}</td><td>{format!("{:.3} / {:.3} / {:.3}", test.result.configuration.position_p, test.result.configuration.position_i, test.result.configuration.position_d)}</td><td>{test.result.samples.len()}</td><td><button onclick={Callback::from(move |_| { let mut a = (*toggle).clone(); a[index].visible = !a[index].visible; toggle.set(a); })}>{if test.visible {"Hide"} else {"Show"}}</button><button onclick={Callback::from(move |_| { let mut a = (*delete).clone(); a.remove(index); delete.set(a); })}>{"Delete"}</button><button onclick={Callback::from(move |_| config.set(saved.result.configuration.clone()))}>{"Load config"}</button></td></tr> } })}</tbody></table> }
}

#[function_component]
fn App() -> Html {
    let tab = use_state(|| 0_usize);
    let activated = {
        let tab = tab.clone();
        Callback::from(move |id| tab.set(id))
    };
    html! { <div class="content-frame"><div class="content-root"><MatTabBar onactivated={activated}><MatTab min_width=true icon="dashboard"/><MatTab min_width=true icon="tune"/><MatTab min_width=true icon="settings"/></MatTabBar><TabPage id=0 current_id={*tab}><Card title="TVC flight computer" icon="rocket_launch"><p>{"Open the tune tab to configure and characterize the X and Y TVC servomotors."}</p></Card></TabPage><TabPage id=1 current_id={*tab}><ServoCalibration/></TabPage><TabPage id=2 current_id={*tab}><p>{"System settings"}</p></TabPage></div></div> }
}
fn main() {
    yew::Renderer::<App>::new().render();
}
