use crate::simmulation_instance::run_simulation;
use crate::visual_objects::{Rocket3DObject, spawn_all_entities};
use bevy::core_pipeline::bloom::BloomSettings;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::keyboard::KeyboardInput;
use bevy::pbr::DirectionalLightShadowMap;
use bevy::prelude::*;
use bevy::reflect::List;
use light_tvc_rocket_simulation::model::SimulationLog;

mod cone;
mod simmulation_instance;
mod visual_objects;

#[derive(Resource)]
pub struct SimulationResult {
    simulation_result: Vec<SimulationLog>,
}

#[derive(Resource)]
pub struct Trajectory {
    points: Vec<Vec3>,
}

fn main() {
    println!("Starting simulation");
    let mut simulation_result = run_simulation();
    println!("Simulation done");

    simulation_result.sort_by(|a, b| b.time.total_cmp(&a.time));

    App::new()
        .insert_resource(Msaa::Sample8)
        .insert_resource(SimulationResult { simulation_result })
        .insert_resource(Trajectory { points: vec![] })
        .insert_resource(DirectionalLightShadowMap { size: 4096 })
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .add_systems(Update, camera_orbit)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 0.1,
    });

    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1800.0,
            radius: 0.1,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(6.0, 2.0, 10.0),
        ..default()
    });

    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1100.0,
            radius: 0.1,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(-6.0, 1.0, 10.0),
        ..default()
    });

    commands.spawn((
        Camera3dBundle {
            camera: Camera {
                hdr: true, // 1. HDR is required for bloom
                ..default()
            },
            tonemapping: Tonemapping::TonyMcMapface,
            transform: Transform::from_xyz(3.0, -9.0, 5.0) //(16.0, -32.0, 15.5)
                .looking_at(Vec3::new(0.0, 0.0, 3.0), Vec3::Z),
            ..default()
        },
        BloomSettings {
            intensity: 0.15,
            // low_frequency_boost: 1.0,
            // low_frequency_boost_curvature:1.0,
            //composite_mode: Additive,
            ..default()
        },
    ));

    spawn_all_entities(&mut commands, &mut meshes, &mut materials);
}

pub fn update(
    time: Res<Time>,
    mut camera: Query<&mut Transform, (With<Camera>, Without<Rocket3DObject>)>,
    mut rocket_3d_objects: Query<
        (&mut Transform, &mut Visibility, &Rocket3DObject),
        Without<Camera>,
    >,
    mut gizmos: Gizmos,
    mut simulation_result: ResMut<SimulationResult>,
    mut trajectory: ResMut<Trajectory>,
) {
    let mut last_state = None;

    while !simulation_result.simulation_result.is_empty()
        && simulation_result.simulation_result.last().unwrap().time < time.elapsed_seconds()
    {
        last_state = simulation_result.simulation_result.pop();
    }

    last_state.iter().for_each(|a| {
        println!("time: {}", a.time);
        trajectory.points.push(a.position);
        for mut rocket_3d_object in &mut rocket_3d_objects {
            rocket_3d_object.0.translation = a.position / 20.0;
            rocket_3d_object.0.rotation = a.rotation;
        }
    });

    for point in & trajectory.points {
        gizmos.sphere(*point / 20.0, Quat::default(), 0.001, Color::WHITE);
    }
}

pub fn camera_orbit(
    keys: ResMut<Input<KeyCode>>,
    mut camera: Query<&mut Transform, (With<Camera>, Without<Rocket3DObject>)>,
) {
    if keys.pressed(KeyCode::A) {
        camera
            .single_mut()
            .rotate_around(Vec3::default(), Quat::from_rotation_z(-0.02));
    }
    if keys.pressed(KeyCode::D) {
        camera
            .single_mut()
            .rotate_around(Vec3::default(), Quat::from_rotation_z(0.02));
    }
    if keys.pressed(KeyCode::W) {
        let mut c = camera.single_mut();
        let cr = c.rotation;
        c.rotate_around(
            Vec3::default(),
            Quat::from_axis_angle(cr * Vec3::new(1.0, 0.0, 0.0), 0.01),
        );
    }
    if keys.pressed(KeyCode::S) {
        let mut c = camera.single_mut();
        let cr = c.rotation;
        c.rotate_around(
            Vec3::default(),
            Quat::from_axis_angle(cr * Vec3::new(1.0, 0.0, 0.0), -0.01),
        );
    }
    if keys.pressed(KeyCode::Z) {
        camera.single_mut().translation *= 0.99
    }

    if keys.pressed(KeyCode::X) {
        camera.single_mut().translation *= 1.01
    }
}
