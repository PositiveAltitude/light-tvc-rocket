mod components;

use crate::components::*;
use std::any::Any;

use light_robot_core_api::*;
use std::process::Child;

use gloo::console::log;
use material_yew::text_inputs::TextFieldType;
use material_yew::*;
use reqwasm::http::Request;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use wasm_bindgen_futures::spawn_local;
use yew::prelude::*;
use yew_hooks::prelude::*;

use gloo::timers::callback::Timeout;
use wasm_bindgen::JsCast;
use web_sys::console::log;
use web_sys::{console, HtmlInputElement};

#[derive(Properties, PartialEq)]
struct RestButtonProps {
    text: String,
    command: Command,
    pointerup_command: Option<Command>,
    #[prop_or_default]
    equal_size: bool,
}

#[function_component]
fn RestButton(props: &RestButtonProps) -> Html {
    let text = props.text.clone();
    let commmand: Command = props.command.clone();

    let onpointerdown: Callback<_, ()> = Callback::from(move |_| {
        let s = "http://lrc.local/command".to_owned();
        let s = s.clone();
        let commmand = commmand.clone();
        spawn_local(async move {
            Request::post(&s)
                .body(serde_json::to_string(&commmand).unwrap())
                .send()
                .await
                .unwrap();
        });
        ()
    });

    let pointerup_command = props.pointerup_command.clone();

    let cancel_fn = move || match pointerup_command.clone() {
        None => (),
        Some(command) => {
            let s = "http://lrc.local/command".to_owned();
            let s = s.clone();
            spawn_local(async move {
                Request::post(&s)
                    .body(serde_json::to_string(&command).unwrap())
                    .send()
                    .await
                    .unwrap();
            });
            ()
        }
    };

    let onpointerup: Callback<PointerEvent, ()> = Callback::from(move |_| cancel_fn());

    html! {<span class={if props.equal_size {"equal-size"} else {""}} {onpointerdown}{onpointerup}><MatButton label={text} outlined=true/></span>}
}

static command_uri: &str = "http://lrc.local/command";

fn send_command(command: Command) {
    spawn_local(async move {
        Request::post(command_uri)
            .body(serde_json::to_string(&command).unwrap())
            .send()
            .await
            .unwrap();
    });
}

#[function_component]
fn WifiSettings() -> Html {
    let ssid = use_state(|| String::new());
    let password = use_state(|| String::new());

    let ssid1 = ssid.clone();
    let password1 = password.clone();
    let onclick = move |_| {
        let cmd = Command::SetWifi {
            ssid: (*ssid1).clone(),
            password: (*password1).clone(),
        };
        send_command(cmd);
    };

    html! { <div>
                <MatTextField label="ssid" value={(*ssid).clone()} oninput={move |s:String| {ssid.set(s)}}/>
                <MatTextField label="password" value={(*password).clone()} oninput={move |s:String| {password.set(s)}}/>
                <span {onclick}><MatButton label="Set wifi" outlined=true/></span>
        </div>
    }
}

#[function_component]
fn App() -> Html {
    let current_tab = use_state(|| 0);

    let current_tab_ = current_tab.clone();
    let onactivated = move |current_id: usize| current_tab_.set(current_id);

    html! {
        <div class={classes!("content-frame")}>
            <div class={classes!("content-root")}>
                <MatTabBar {onactivated}>
                    <MatTab min_width=true icon="dashboard"/>
                    <MatTab min_width=true icon="bolt"/>
                    <MatTab min_width=true icon="settings"/>
                </MatTabBar>
                <TabPage id=0 current_id={*current_tab}>
                    <StateComponent/>
                </TabPage>
                <TabPage id=1 current_id={*current_tab}>
                    <Card title="leds" icon="wb_twilight">
                        <HorizontalLayout>
                            <RestButton equal_size=true text="BLUE" command={Command::SetLedColor {r: 0, g: 0, b: 20}} pointerup_command={Command::SetLedColor {r: 0, g: 0, b: 0}}/>
                            <RestButton equal_size=true text="RED" command={Command::SetLedColor {r: 20, g: 0, b: 0}} pointerup_command={Command::SetLedColor {r: 0, g: 0, b: 0}}/>
                            <RestButton equal_size=true text="GREEN" command={Command::SetLedColor {r: 0, g: 20, b: 0}} pointerup_command={Command::SetLedColor {r: 0, g: 0, b: 0}}/>
                        </HorizontalLayout>
                    </Card>
                    <Card title="move" icon="open_with">
                        <HorizontalLayout>
                            <RestButton equal_size=true text="LEFT" command={Command::ServoCommand {command: ServoCommand::Update {servo1: 0.5, servo2: 0.0}}} pointerup_command={Command::ServoCommand {command: ServoCommand::Disable}}/>
                            <RestButton equal_size=true text="RIGHT" command={Command::ServoCommand {command: ServoCommand::Update {servo1: -0.5, servo2: 0.0}}} pointerup_command={Command::ServoCommand {command: ServoCommand::Disable}}/>
                            <RestButton equal_size=true text="F" command={Command::ServoCommand {command: ServoCommand::Update {servo1: 0.0, servo2: 1.0}}} pointerup_command={Command::ServoCommand {command: ServoCommand::Disable}}/>
                            <RestButton equal_size=true text="B" command={Command::ServoCommand {command: ServoCommand::Update {servo1: 0.0, servo2: -1.0}}} pointerup_command={Command::ServoCommand {command: ServoCommand::Disable}}/>
                        </HorizontalLayout>
                        <HorizontalLayout>
                            <RestButton equal_size=true text="TEST" command={Command::TestServo {servo_id: 0,start: 0.0 , end: -1.0}}/>
                        </HorizontalLayout>
                    </Card>
                </TabPage>
                <TabPage id=2 current_id={*current_tab}>
                    <WifiSettings/>
                </TabPage>
            </div>
        </div>
    }
}

#[function_component]
fn StateComponent() -> Html {
    let state = use_state_eq(|| State::default());
    let update_required = use_state_eq(|| true);

    async fn fetch_state() -> Result<State, Error> {
        fetch::<State>("http://lrc.local/state".to_string()).await
    }

    async fn fetch<T>(url: String) -> Result<T, Error>
    where
        T: DeserializeOwned,
    {
        let response = Request::get(&url).send().await;
        if let Ok(data) = response {
            (data.json::<T>().await).map_or(Err(Error::DeserializeError), |repo| Ok(repo))
        } else {
            Err(Error::RequestError)
        }
    }

    let u3 = update_required.clone();

    let state2 = state.clone();

    let async_request: UseAsyncHandle<State, Error> = use_async(async move {
        let ans = fetch_state().await;
        let ans2 = ans.clone();
        if ans.is_ok() {
            state2.set(ans.unwrap())
        };
        Timeout::new(1000, move || {
            log!("request");
            u3.set(true);
        })
        .forget();
        ans2
    });

    let u2 = update_required.clone();
    if *u2 {
        async_request.run();
        u2.set(false);
    }

    let battery_icon = match state.battery.soc {
        x if x < 0.1 => "battery_0_bar",
        x if x < 0.233 => "battery_1_bar",
        x if x < 0.366 => "battery_2_bar",
        x if x < 0.50 => "battery_3_bar",
        x if x < 0.633 => "battery_4_bar",
        x if x < 0.766 => "battery_5_bar",
        x if x <= 0.90 => "battery_6_bar",
        x if x > 0.90 => "battery_full",
        _ => "battery_unknown",
    };

    fn pyro_status(pyro: &PyroChannelState) -> &'static str {
        match pyro {
            PyroChannelState {
                fire: true,
                test_voltage: _,
            } => "active!!!",
            PyroChannelState {
                fire: false,
                test_voltage: tv,
            } if *tv > 1.0f32 => "connected",
            _ => "not connected",
        }
    }

    fn servo_state(servo: &Option<f32>) -> String {
        match servo {
            None => String::from("off"),
            Some(a) => format!("{:.4}", a),
        }
    }

    html! {
        <div class="state">
            <Card title="battery" icon={battery_icon}>
                <HorizontalLayout>
                    <span class="first-column"><VerticalLayout>
                        <div>{"Battery charge"}</div>
                        <div>{"Battery voltage"}</div>
                        <div>{"Battery charge rate"}</div>
                    </VerticalLayout></span>
                    <VerticalLayout>
                        <div>{format!("{:.0}", state.battery.soc)}</div>
                        <div>{format!("{:.2}", state.battery.voltage)}</div>
                        <div>{format!("{:.1}", state.battery.charge_rate)}</div>
                    </VerticalLayout>
                    <div class="separator"/>
                    <VerticalLayout>
                        <div>{"%"}</div>
                        <div>{"V"}</div>
                        <div>{"%/hr"}</div>
                    </VerticalLayout>
                </HorizontalLayout>
            </Card>
            <Card title="can" icon="flare">
                <div>{format!("position servo1: {:?}", state.servo1.position)}</div>
                <div>{format!("position servo2: {:?}", state.servo2.position)}</div>
            </Card>
            <Card title="asd" icon="flare">
                <CanvasComponent/>
            </Card>
        </div>
    }
}

fn main() {
    yew::Renderer::<App>::new().render();
}

#[derive(Clone, Debug, PartialEq)]
enum Error {
    RequestError,
    DeserializeError,
}
