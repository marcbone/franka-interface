// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
use franka_interface::rubullet::{AddDebugLineOptions, ItemId, PhysicsClient};
use franka_interface::types::{JointPositions, Torques, Vector7};
use franka_interface::RobotArguments;
use nalgebra::Vector3;
use rand_distr::{Distribution, Normal};
use std::cell::RefMut;
use std::f64::consts::PI;
use std::path::PathBuf;
use std::time::Duration;
use structopt::StructOpt;
/// Joint pose controller that can run in the simulation or on a real robot.
/// The parameters for this example are tuned for the simulation as the example does not work
/// well on the real robot. See joint_space_controller_without_mass_matrix for an example that
/// works better on the real robot.
#[derive(StructOpt, Debug)]
#[structopt(name = "joint_space_controller")]
enum CommandLineArguments {
    /// Runs program on the real robot
    Real {
        /// IP Address or hostname of the real robot
        #[structopt()]
        ip: String,
    },
    /// Runs program in the simulation
    Simulation {
        /// Path to the Franka URDF file
        #[structopt(parse(from_os_str))]
        urdf_path: PathBuf,
    },
}
struct SimParameters {
    param_p: Option<ItemId>,
    param_d: Option<ItemId>,
    param_input_noise: Option<ItemId>,
    param_output_noise: Option<ItemId>,
    restart_button: Option<ItemId>,

    p: f64,
    d: f64,
    input_noise_sigma: f64,
    output_noise_sigma: f64,
    button_value: f64,
}
impl SimParameters {
    pub fn new() -> Self {
        SimParameters {
            param_p: None,
            param_d: None,
            param_input_noise: None,
            param_output_noise: None,
            restart_button: None,
            p: 200.,
            d: 10.0,
            input_noise_sigma: 0.0,
            output_noise_sigma: 0.0,
            button_value: 0.,
        }
    }
    pub fn get_p(&self) -> f64 {
        self.p
    }
    pub fn get_d(&self) -> f64 {
        self.d
    }
    pub fn get_input_noise_sigma(&self) -> f64 {
        self.input_noise_sigma
    }
    pub fn get_output_noise_sigma(&self) -> f64 {
        self.output_noise_sigma
    }
    pub fn get_button_value(&self) -> f64 {
        self.button_value
    }
    pub fn initialize(&mut self, mut client: RefMut<PhysicsClient>) {
        self.param_p = Some(
            client
                .add_user_debug_parameter("P", 0., 5., self.p)
                .unwrap(),
        );
        self.param_d = Some(
            client
                .add_user_debug_parameter("D", 0., 5., self.d)
                .unwrap(),
        );
        self.param_input_noise = Some(
            client
                .add_user_debug_parameter("input noise sigma", 0., 0.05, self.input_noise_sigma)
                .unwrap(),
        );
        self.param_output_noise = Some(
            client
                .add_user_debug_parameter("output noise sigma", 0., 0.05, self.output_noise_sigma)
                .unwrap(),
        );
        self.restart_button = Some(
            client
                .add_user_debug_parameter("Restart", 1., 0., self.button_value)
                .unwrap(),
        );
    }
    pub fn update_values(&mut self, mut client: RefMut<PhysicsClient>) {
        self.p = client
            .read_user_debug_parameter(self.param_p.unwrap())
            .unwrap();
        self.d = client
            .read_user_debug_parameter(self.param_d.unwrap())
            .unwrap();
        self.input_noise_sigma = client
            .read_user_debug_parameter(self.param_input_noise.unwrap())
            .unwrap();
        self.output_noise_sigma = client
            .read_user_debug_parameter(self.param_output_noise.unwrap())
            .unwrap();
        self.button_value = client
            .read_user_debug_parameter(self.restart_button.unwrap())
            .unwrap();
    }
}
struct MySimSetup {
    path: PathBuf,
}

impl SimulationSetup for MySimSetup {
    fn set_simulation_time_step(&self) -> Duration {
        Duration::from_millis(1)
    }

    fn set_franka_urdf_path(&self) -> PathBuf {
        self.path.clone()
    }
}
struct MyRealSetup {}
impl RealSetup for MyRealSetup {}
fn main() {
    let args: CommandLineArguments = CommandLineArguments::from_args();
    let mode;
    let mut hostname = String::new();
    let mut path = PathBuf::new();
    match args {
        CommandLineArguments::Real { ip } => {
            mode = Mode::Real;
            hostname = ip
        }
        CommandLineArguments::Simulation { urdf_path } => {
            mode = Mode::Simulation;
            path = urdf_path
        }
    };
    let mut parameters = SimParameters::new();

    let mut experiment = Experiment::new(mode, MySimSetup { path }, MyRealSetup {});
    let mut robot = experiment.new_robot(RobotArguments {
        hostname,
        base_pose: None,
        initial_config: None,
    });
    experiment.use_physics_client(|client| {
        parameters.initialize(client);
    });
    experiment.use_physics_client(|client| {
        parameters.update_values(client);
    });
    robot.close_gripper();
    robot.joint_motion(
        0.1,
        Vector7::from_column_slice(&[0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.]),
    );
    let mut time = Duration::from_secs(0);
    let mut cartesian_positions = Vec::<Vector3<f64>>::new();
    robot.control_joint_positions(|state, duration| {
        time += duration.clone();
        let mut out = JointPositions::from(get_joint_angles(time));
        if time.as_secs_f64() >= 10. {
            out.is_finished = true;
        }
        cartesian_positions.push(state.end_effector_pose.translation.vector);
        out
    });

    experiment.use_physics_client(|mut client| {
        for i in 0..cartesian_positions.len() / 100 {
            client
                .add_user_debug_line(
                    cartesian_positions[100 * i],
                    cartesian_positions[100 * i + 99],
                    AddDebugLineOptions {
                        line_width: 3.,
                        ..Default::default()
                    },
                )
                .unwrap();
        }
    });

    robot.joint_motion(
        0.1,
        [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.].into(),
    );

    let mut last_button_value = 0.;
    let mut cartesian_positions = Vec::<Vector3<f64>>::new();
    let mut first_time = true;
    let mut item_ids = Vec::<ItemId>::new();

    loop {
        let mut index = 0;
        experiment.use_physics_client(|client| parameters.update_values(client));
        if parameters.get_button_value() > last_button_value || !experiment.is_simulation() {
            last_button_value = parameters.button_value;
            let mut time = Duration::from_secs(0);
            let p = parameters.get_p();
            let d = parameters.get_d();
            let input_noise_dist = Normal::new(0., parameters.get_input_noise_sigma()).unwrap();
            let output_noise_dist = Normal::new(0., parameters.get_output_noise_sigma()).unwrap();
            robot.control_torques(|state, duration| {
                time += duration.clone();
                let noise_position: Vector7 =
                    [input_noise_dist.sample(&mut rand::thread_rng()); 7].into();
                let noise_velocity: Vector7 =
                    [input_noise_dist.sample(&mut rand::thread_rng()); 7].into();
                let noise_torques: Vector7 =
                    [output_noise_dist.sample(&mut rand::thread_rng()); 7].into();
                let (position, velocity, acceleration) = get_desired_joint_state(time);
                let mut mass_normalization = Vector7::zeros();
                for i in 0..7 {
                    mass_normalization[i] = 1. / state.mass_matrix.row(i).norm();
                }
                let tau: Vector7 = state.mass_matrix
                    * (acceleration
                        + p * (position - state.joint_positions + noise_position)
                        + d * (velocity - state.joint_velocities + noise_velocity))
                    + state.coriolis;

                if first_time {
                    cartesian_positions.push(state.end_effector_pose.translation.vector);
                } else {
                    cartesian_positions[index] = state.end_effector_pose.translation.vector;
                    index += 1;
                }

                let mut out = Torques::from(tau + noise_torques);
                if time.as_secs_f64() >= 10. {
                    out.is_finished = true;
                }
                out
            });
            experiment.use_physics_client(|mut client| {
                for i in 0..cartesian_positions.len() / 100 {
                    if first_time {
                        item_ids.push(
                            client
                                .add_user_debug_line(
                                    cartesian_positions[100 * i],
                                    cartesian_positions[100 * i + 99],
                                    AddDebugLineOptions {
                                        line_color_rgb: [1., 0., 0.],
                                        ..Default::default()
                                    },
                                )
                                .unwrap(),
                        );
                    } else {
                        client
                            .add_user_debug_line(
                                cartesian_positions[100 * i],
                                cartesian_positions[100 * i + 99],
                                AddDebugLineOptions {
                                    line_color_rgb: [1., 0., 0.],
                                    replace_item_id: Some(item_ids[i]),
                                    ..Default::default()
                                },
                            )
                            .unwrap();
                    }
                }
            });
            first_time = false;
            robot.joint_motion(
                0.1,
                [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.].into(),
            );
            if !experiment.is_simulation() {
                break;
            }
        } else {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
}

fn get_joint_angles(time: Duration) -> Vector7 {
    let total_duration = Duration::from_secs(5);
    let start: Vector7 = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.].into();
    let offset: Vector7 = Vector7::from_column_slice(&[1.3, 0.1, 0., 0., 0.3, 0., 0.2])
        * (1. - f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64()))
        / 2.0;
    start + offset
}

fn get_desired_joint_state(time: Duration) -> (Vector7, Vector7, Vector7) {
    let total_duration = Duration::from_secs(5);
    let start: Vector7 = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.].into();
    let offset = Vector7::from_column_slice(&[1.3, 0.1, 0., 0., 0.3, 0., 0.2]);
    let position: Vector7 = start
        + offset * (1. - f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64())) / 2.0;
    let velocity = offset * (PI * f64::sin(PI * time.as_secs_f64() / total_duration.as_secs_f64()))
        / (2. * total_duration.as_secs_f64());
    let acceleration =
        offset * PI * PI * f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64())
            / (2. * f64::powf(total_duration.as_secs_f64(), 2.));
    (position, velocity, acceleration)
}
