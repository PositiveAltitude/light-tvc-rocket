use gloo_timers::callback::Timeout;
use light_robot_core_api as api;
use material_yew::*;
use reqwasm::http::Request;
use wasm_bindgen::JsCast;
use web_sys::{console, CanvasRenderingContext2d, HtmlCanvasElement};
use yew::prelude::*;

#[derive(Properties, PartialEq)]
pub struct ChildrenProps {
    pub children: Children,
}

#[function_component]
pub fn HorizontalLayout(props: &ChildrenProps) -> Html {
    html!(
        <div class="horizontal-layout">
            {props.children.clone()}
        </div>
    )
}

#[function_component]
pub fn VerticalLayout(props: &ChildrenProps) -> Html {
    html!(
        <div class="vertical-layout">
            {props.children.clone()}
        </div>
    )
}

#[derive(Properties, PartialEq)]
pub struct CardProps {
    pub children: Children,
    pub title: String,
    #[prop_or_default]
    pub icon: Option<String>,
}

#[function_component]
pub fn Card(props: &CardProps) -> Html {
    html!(
        <div class="card">
            <div class="header">
                {
                    props.icon.as_ref().map(|icon| html! {
                        <MatIcon>{icon.clone()}</MatIcon>
                    }).unwrap_or_default()
                }
                <h2>{props.title.clone()}</h2>
            </div>
            <div class="card-content">{props.children.clone()}</div>
        </div>
    )
}

#[derive(Properties, PartialEq)]
pub struct TabPageProps {
    pub children: Children,
    pub id: usize,
    pub current_id: usize,
}

#[function_component]
pub fn TabPage(props: &TabPageProps) -> Html {
    html! {
        <div class="tab-page" hidden={props.id != props.current_id}>{props.children.clone()}</div>
    }
}

// #[derive(Properties, PartialEq)]
// pub struct CanvasComponentProps {
//     pub on_touch: Callback<(f64, f64)>,
// }
#[function_component]
pub fn CanvasComponent() -> Html {
    let command_url = "http://lrc.local/command".to_owned();
    let is_throttled = use_mut_ref(|| false);
    let cancel_timeout = use_mut_ref(|| None::<Timeout>);

    let canvas_ref = use_node_ref();
    let canvas_ref2 = canvas_ref.clone();

    let on_pointer_move = {
        let canvas_ref = canvas_ref.clone();
        let is_throttled = is_throttled.clone();
        let cancel_timeout = cancel_timeout.clone();

        let command_url = command_url.clone();
        Callback::from(move |event: PointerEvent| {
            if event.buttons() == 1 {
                if *is_throttled.borrow() {
                    return;
                }

                *is_throttled.borrow_mut() = true;

                let canvas = canvas_ref
                    .cast::<HtmlCanvasElement>()
                    .expect("Failed to get canvas");
                let context = canvas
                    .get_context("2d")
                    .unwrap()
                    .unwrap()
                    .dyn_into::<CanvasRenderingContext2d>()
                    .expect("Failed to cast to CanvasRenderingContext2d");

                let rect = canvas.get_bounding_client_rect();
                let x = event.client_x() as f64 - rect.left();
                let y = event.client_y() as f64 - rect.top();

                let (mut xx, mut yy) =
                    (x / rect.width() * 2.0 - 1.0, -y / rect.height() * 2.0 + 1.0);
                xx = xx.clamp(-1.0, 1.0);
                yy = yy.clamp(-1.0, 1.0);

                context.clear_rect(0.0, 0.0, 1000.0, 1000.0);
                context.move_to(500.0, 0.0);
                context.line_to(500.0, 1000.0);
                context.move_to(0.0, 500.0);
                context.line_to(1000.0, 500.0);
                context.set_stroke_style_str("black");
                context.set_line_width(5.0);
                context.stroke();

                // Draw circle
                context.begin_path();

                context.set_stroke_style_str("blue");
                //context.set_fill_style(&"blue".into());
                context
                    .arc(
                        500.0 + xx * 500.0,
                        500.0 - yy * 500.0,
                        20.0,
                        0.0,
                        std::f64::consts::PI * 2.0,
                    )
                    .unwrap();

                console::log_1(&"Go".into());
                {
                    let s = command_url.clone();
                    let commmand = api::Command::ServoCommand {
                        command: api::ServoCommand::Update {
                            servo1: xx as f32,
                            servo2: yy as f32,
                        },
                    };
                    wasm_bindgen_futures::spawn_local(async move {
                        Request::post(&s)
                            .body(serde_json::to_string(&commmand).unwrap())
                            .send()
                            .await
                            .unwrap();
                    });
                }
                ////////////////////////////////////////////////
            }
            let is_throttled = is_throttled.clone();
            Timeout::new(20, move || {
                *is_throttled.borrow_mut() = false;
            })
            .forget();

            if let Some(timeout) = cancel_timeout.borrow_mut().take() {
                timeout.cancel();
            }

            // let cancel = {
            //     let command_url = command_url.clone();
            //     Timeout::new(150, move || {
            //         console::log_1(&format!("Stop").into());
            //         {
            //             let s = command_url.clone();
            //             let commmand = light_robot_core_api::CarCommand::Disable;
            //             wasm_bindgen_futures::spawn_local(async move {
            //                 Request::post(&s)
            //                     .body(serde_json::to_string(&commmand).unwrap())
            //                     .send()
            //                     .await
            //                     .unwrap();
            //             });
            //         }
            //     })
            // };
            //
            // *cancel_timeout.borrow_mut() = Some(cancel);
        })
    };

    let onpointerup = {
        let command_url = command_url.clone();
        Callback::from(move |_event: PointerEvent| {
            console::log_1(&"Stop".into());
            let s = command_url.clone();
            let commmand = api::Command::ServoCommand {
                command: api::ServoCommand::Disable,
            };
            wasm_bindgen_futures::spawn_local(async move {
                Request::post(&s)
                    .body(serde_json::to_string(&commmand).unwrap())
                    .send()
                    .await
                    .unwrap();
            });
        })
    };

    use_effect_with_deps(
        move |_| {
            let canvas = canvas_ref
                .cast::<HtmlCanvasElement>()
                .expect("Failed to get canvas");
            let context = canvas
                .get_context("2d")
                .expect("Failed to get context")
                .unwrap()
                .dyn_into::<CanvasRenderingContext2d>()
                .expect("Failed to cast to CanvasRenderingContext2d");

            // Set canvas size
            canvas.set_width(1000);
            canvas.set_height(1000);

            // Draw something

            context.move_to(500.0, 0.0);
            context.line_to(500.0, 1000.0);
            context.move_to(0.0, 500.0);
            context.line_to(1000.0, 500.0);
            context.set_stroke_style_str("black");
            context.set_line_width(5.0);
            context.stroke();

            || ()
        },
        (),
    );

    let o2 = on_pointer_move.clone();

    html! {
        <canvas onpointerdown= { o2} onpointermove={on_pointer_move} onpointerup={onpointerup} class={classes!("control-field")} ref={canvas_ref2} ></canvas>
        //<canvas onpointerup={onpointerup} class={classes!("control-field")} ref={canvas_ref2} ></canvas>
    }
}
