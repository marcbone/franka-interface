// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use franka::Matrix7;
use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
use franka_interface::rubullet::nalgebra::Vector6;
use franka_interface::rubullet::{AddDebugLineOptions, ItemId, PhysicsClient};
use franka_interface::types::{CartesianPose, Torques, Vector7};
use franka_interface::RobotArguments;
use nalgebra::{Isometry3, Vector3};
use rand_distr::{Distribution, Normal};
use std::cell::RefMut;
use std::f64::consts::PI;
use std::path::PathBuf;
use std::time::Duration;
use structopt::StructOpt;
/// Cartesian pose controller that can run in the simulation or on a real robot.
/// The parameters for this example are tuned for the simulation as the example does not work
/// well on the real robot. See cartesian_space_controller_without_mass_matrix for an example that
/// works better on the real robot.
#[derive(StructOpt, Debug)]
#[structopt(name = "cartesian_space_controller")]
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
    param_p_nullspace: Option<ItemId>,
    param_input_noise: Option<ItemId>,
    param_output_noise: Option<ItemId>,
    restart_button: Option<ItemId>,

    p: f64,
    d: f64,
    p_nullspace: f64,
    input_noise_sigma: f64,
    output_noise_sigma: f64,
    button_value: f64,
}
impl SimParameters {
    pub fn new() -> Self {
        SimParameters {
            param_p: None,
            param_d: None,
            param_p_nullspace: None,
            param_input_noise: None,
            param_output_noise: None,
            restart_button: None,
            p: 0.3,
            d: 10.,
            p_nullspace: 5.,
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
    pub fn get_p_nullspace(&self) -> f64 {
        self.p_nullspace
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
                .add_user_debug_parameter("P", 0., 10., self.p)
                .unwrap(),
        );
        self.param_d = Some(
            client
                .add_user_debug_parameter("D", 0., 10., self.d)
                .unwrap(),
        );
        self.param_p_nullspace = Some(
            client
                .add_user_debug_parameter("P_NULLSPACE", 0., 10., self.p_nullspace)
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
        self.p_nullspace = client
            .read_user_debug_parameter(self.param_p_nullspace.unwrap())
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
        [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.].into(),
    );
    let mut time = Duration::from_secs(0);
    let mut cartesian_positions = Vec::<Vector3<f64>>::new();
    let mut start_pose = robot.get_state().end_effector_pose;
    robot.control_cartesian_pose(|state, duration| {
        time += duration.clone();
        if time.as_secs_f64() == 0. {
            start_pose = state.end_effector_pose_c;
        }
        let mut out = CartesianPose::from(get_position(start_pose, time));
        if time.as_secs_f64() >= 4. {
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
            let p_nullspace = parameters.get_p_nullspace();
            let input_noise_dist = Normal::new(0., parameters.get_input_noise_sigma()).unwrap();
            let output_noise_dist = Normal::new(0., parameters.get_output_noise_sigma()).unwrap();
            robot.control_torques(|state, duration| {
                time += duration.clone();
                let noise_position: Vector7 =
                    [input_noise_dist.sample(&mut rand::thread_rng()); 7].into();
                let noise_velocity: Vector7 =
                    [input_noise_dist.sample(&mut rand::thread_rng()); 7].into();
                let noise_c_pose: Vector6<f64> =
                    [input_noise_dist.sample(&mut rand::thread_rng()); 6].into();
                let noise_c_vel: Vector6<f64> =
                    [input_noise_dist.sample(&mut rand::thread_rng()); 6].into();
                let noise_torques: Vector7 =
                    [output_noise_dist.sample(&mut rand::thread_rng()); 7].into();

                let null_space_goal_velocity: Vector7 = [
                    (0.5 - (state.joint_positions[0] + noise_position[0]))
                        - (state.joint_velocities[0] + noise_velocity[0]),
                    0.,
                    0.,
                    0.,
                    0.,
                    0.,
                    0.,
                ]
                .into();
                let end_effector_velocity = state.jacobian * state.joint_velocities;
                let (position, velocity, acceleration) =
                    get_desired_cartesian_state(start_pose, time);

                let mass_inv = state.mass_matrix.try_inverse().unwrap();
                let lambda = (state.jacobian * mass_inv * state.jacobian.transpose())
                    .try_inverse()
                    .unwrap();

                let pose_error = differentiate_one_sample(position, state.end_effector_pose, 0.001)
                    + noise_c_pose;
                let vel_error = velocity - end_effector_velocity + noise_c_vel;

                let mut tau: Vector7 = state.jacobian.transpose()
                    * lambda
                    * (acceleration + p * pose_error + d * vel_error)
                    + state.coriolis;
                let h_m = lambda * state.jacobian * mass_inv;
                let tau_ns_desired = p_nullspace * null_space_goal_velocity;
                let proj = Matrix7::identity() - state.jacobian.transpose() * h_m;
                let tau_ns = proj * tau_ns_desired;
                tau += tau_ns;

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

fn get_position(mut start: Isometry3<f64>, time: Duration) -> Isometry3<f64> {
    let total_duration = Duration::from_secs(2);
    let mut offset: Isometry3<f64> = Isometry3::translation(0., 0., -0.2);
    offset.translation.vector *=
        (1. - f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64())) / 2.0;
    start.translation.vector += offset.translation.vector;
    start
}

fn get_desired_cartesian_state(
    start: Isometry3<f64>,
    time: Duration,
) -> (Isometry3<f64>, Vector6<f64>, Vector6<f64>) {
    let total_duration = Duration::from_secs(5);

    let offset = Vector6::new(0., 0., -0.2, 0., 0., 0.);
    let offset_position: Vector6<f64> =
        offset * (1. - f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64())) / 2.0;
    let position: Isometry3<f64> =
        Isometry3::translation(offset_position[0], offset_position[1], offset_position[2]) * start;
    let velocity = offset * (PI * f64::sin(PI * time.as_secs_f64() / total_duration.as_secs_f64()))
        / (2. * total_duration.as_secs_f64());
    let acceleration =
        offset * PI * PI * f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64())
            / (2. * f64::powf(total_duration.as_secs_f64(), 2.));
    (position, velocity, acceleration)
}
fn differentiate_one_sample(
    pose: Isometry3<f64>,
    last_pose: Isometry3<f64>,
    delta_t: f64,
) -> Vector6<f64> {
    let head = (pose.translation.vector - last_pose.translation.vector) / delta_t;
    let delta_rotation = pose.rotation * last_pose.rotation.inverse();
    let scaled_axis = delta_rotation.scaled_axis() / delta_t;
    let mut res = [0.; 6];
    for i in 0..3 {
        res[i] = head[i];
    }
    for i in 0..3 {
        res[i + 3] = scaled_axis[i];
    }
    res.into()
}
