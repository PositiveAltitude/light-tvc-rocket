use material_yew::*;
use yew::prelude::*;

#[derive(Properties, PartialEq)]
pub struct CardProps {
    pub children: Children,
    pub title: String,
    #[prop_or_default]
    pub icon: Option<String>,
}

#[function_component]
pub fn Card(props: &CardProps) -> Html {
    html! { <div class="card"><div class="header">
        if let Some(icon) = &props.icon { <MatIcon>{icon.clone()}</MatIcon> }
        <h2>{props.title.clone()}</h2>
    </div><div class="card-content">{props.children.clone()}</div></div> }
}

#[derive(Properties, PartialEq)]
pub struct TabPageProps {
    pub children: Children,
    pub id: usize,
    pub current_id: usize,
}

#[function_component]
pub fn TabPage(props: &TabPageProps) -> Html {
    html! { <div class="tab-page" hidden={props.id != props.current_id}>{props.children.clone()}</div> }
}

/// A numeric editor which keeps its text draft while it has focus.
///
/// Browser number inputs otherwise reject transient values such as an empty
/// string, a minus sign, or an unfinished exponent and can make values hard to
/// replace.  All configuration pages use this component and commit on blur or
/// Enter instead.
#[derive(Properties, PartialEq)]
pub struct NumericInputProps {
    pub value: String,
    pub on_commit: Callback<f32>,
    #[prop_or_default]
    pub class: Classes,
    #[prop_or_default]
    pub disabled: bool,
}

#[function_component]
pub fn NumericInput(props: &NumericInputProps) -> Html {
    let draft = use_state(|| props.value.clone());
    {
        let draft = draft.clone();
        let value = props.value.clone();
        use_effect_with_deps(
            move |value| {
                draft.set(value.clone());
                || ()
            },
            value,
        );
    }
    let commit = {
        let draft = draft.clone();
        let on_commit = props.on_commit.clone();
        Callback::from(move |_| {
            if let Ok(value) = draft.trim().parse::<f32>() {
                on_commit.emit(value);
            }
        })
    };
    let oninput = {
        let draft = draft.clone();
        Callback::from(move |event: InputEvent| {
            draft.set(
                event
                    .target_unchecked_into::<web_sys::HtmlInputElement>()
                    .value(),
            )
        })
    };
    let onkeydown = {
        let commit = commit.clone();
        Callback::from(move |event: KeyboardEvent| {
            if event.key() == "Enter" {
                commit.emit(())
            }
        })
    };
    let onblur = Callback::from(move |_| commit.emit(()));
    html! { <input type="text" inputmode="decimal" class={props.class.clone()} value={(*draft).clone()} disabled={props.disabled} {oninput} {onblur} {onkeydown}/> }
}
