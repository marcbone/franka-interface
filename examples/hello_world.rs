// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
use franka_interface::RobotArguments;
use std::f64::consts::PI;
use std::path::PathBuf;
use structopt::StructOpt;

/// Simple example that can run either inside the simulation or on a real robot.
/// For the simulation it is necessary to specify the URDF path of panda robot.
/// For the real robot it is necessary to specify the IP address or hostname of the robot.
/// Use the --real option to run the program on the real robot.
#[derive(StructOpt, Debug)]
#[structopt(name = "hello_world")]
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
struct MySimSetup {
    pub path: PathBuf,
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

    let mut experiment = Experiment::new(mode, MySimSetup { path }, MyRealSetup {});
    let mut robot = experiment.new_robot(RobotArguments {
        hostname,
        base_pose: None,
        initial_config: None,
    });
    robot.joint_motion(
        0.1,
        [1., PI / 4., 0., -2. * PI / 4., 0., PI / 2., -PI / 4.].into(),
    );
}
