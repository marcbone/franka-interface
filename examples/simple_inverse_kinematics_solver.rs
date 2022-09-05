// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use franka;
use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
use franka_interface::types::Vector7;
use franka_interface::{Robot, RobotArguments};
use std::f64::consts::PI;
use std::path::PathBuf;
use std::time::Duration;

use franka::{Matrix6x7, Matrix7};
use nalgebra;
use nalgebra::{Isometry3, Vector6};
use structopt::StructOpt;
/// A simple inverse kinematics solver that can convert end-effector poses to joint angles.
/// Further, it uses the nullspace to try to move the joints toward the home configuration.
#[derive(StructOpt, Debug)]
#[structopt(name = "simple_inverse_kinematics_solver")]
enum CommandLineArguments {
    /// Runs program on the real robot
    Real {
        /// IP Address or hostname of the real robot
        #[structopt()]
        ip: String,
        /// Path to the libfranka dynamics model file.
        /// It can be downloaded with libfranka-rs.
        /// See the download_model example.
        #[structopt(short, long, parse(from_os_str))]
        model: PathBuf,
    },
    /// Runs program in the simulation
    Simulation {
        /// Path to the Franka URDF file
        #[structopt(parse(from_os_str))]
        urdf_path: PathBuf,
        /// Path to the libfranka dynamics model file.
        /// It can be downloaded with libfranka-rs.
        /// See the download_model example.
        #[structopt(short, long, parse(from_os_str))]
        model: PathBuf,
    },
}
struct MySimSetup {
    path: PathBuf,
}
impl SimulationSetup for MySimSetup {
    fn set_franka_urdf_path(&self) -> PathBuf {
        self.path.clone()
    }
}
struct MyRealSetup {}
impl RealSetup for MyRealSetup {}
fn main() {
    let args: CommandLineArguments = CommandLineArguments::from_args();
    let mode;
    let dynamics_model;
    let mut hostname = String::new();
    let mut path = PathBuf::new();
    match args {
        CommandLineArguments::Real { ip, model } => {
            mode = Mode::Real;
            hostname = ip;
            dynamics_model = model;
        }
        CommandLineArguments::Simulation { urdf_path, model } => {
            mode = Mode::Simulation;
            path = urdf_path;
            dynamics_model = model;
        }
    };

    let mut experiment = Experiment::new(mode, MySimSetup { path }, MyRealSetup {});
    let mut robot = experiment.new_robot(RobotArguments {
        hostname,
        base_pose: None,
        initial_config: None,
    });
    let mut goal_pose = robot.get_state().end_effector_pose;
    goal_pose.translation.z -= 0.2;
    goal_pose.translation.y -= 0.4;
    goal_pose.translation.x -= 0.2;

    let q_new = inverse_kinematics(&mut robot, goal_pose, dynamics_model);

    robot.joint_motion(0.1, q_new);
    std::thread::sleep(Duration::from_secs(10));
}

fn inverse_kinematics(
    robot: &mut Robot,
    goal_pose: Isometry3<f64>,
    model_file: PathBuf,
) -> Vector7 {
    let home_pose: Vector7 = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.].into();
    let model = Model::new(model_file);
    let mut q = robot.get_state().joint_positions;
    let start = std::time::Instant::now();
    for _ in 0..300000 {
        let jacobian = model.jacobian(q);
        let jacobian_inv = jacobian.pseudo_inverse(0.001).unwrap();
        let pose = model.forward(q);
        let null_space_projection = Matrix7::identity() - jacobian_inv * jacobian;

        let null_space_q_dot = null_space_projection * (home_pose - q);
        let pose_error = differentiate_one_sample(goal_pose, pose, 1.);
        if pose_error.norm() < 1e-4 {
            break;
        }
        q += 0.5 * jacobian_inv * pose_error + 0.05 * null_space_q_dot;
    }
    let duration = std::time::Instant::now() - start;
    println!("it took {:?} to solve the inverse kinematics", duration);
    println!("Joint configuration: {}", q);
    println!("End-effector pose{}", model.forward(q));
    q
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
struct Model {
    model: franka::Model,
}
impl Model {
    pub fn new(file: PathBuf) -> Self {
        Model {
            model: franka::Model::new(file, None).unwrap(),
        }
    }
    pub fn jacobian(&self, q: Vector7) -> Matrix6x7 {
        let f_t_ee = [
            0.7071, -0.7071, 0., 0., 0.7071, 0.7071, 0., 0., 0., 0., 1., 0., 0., 0., 0.1034, 1.,
        ];
        let j =
            self.model
                .zero_jacobian(&franka::Frame::EndEffector, q.as_ref(), &f_t_ee, &[0.; 16]);
        Matrix6x7::from_column_slice(&j)
    }
    pub fn forward(&self, q: Vector7) -> Isometry3<f64> {
        let f_t_ee = [
            0.7071, -0.7071, 0., 0., 0.7071, 0.7071, 0., 0., 0., 0., 1., 0., 0., 0., 0.1034, 1.,
        ];
        franka::utils::array_to_isometry(&self.model.pose(
            &franka::Frame::EndEffector,
            &q.as_ref(),
            &f_t_ee,
            &[0.; 16],
        ))
    }
}
