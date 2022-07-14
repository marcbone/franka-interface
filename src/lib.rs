// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use std::cell::RefCell;
use std::convert::TryInto;
use std::f64::consts::PI;
use std::rc::Rc;
use std::time::{Duration, Instant};
pub mod experiment;
use franka::{Frame, Matrix6x7, Matrix7, MotionGenerator};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rubullet::{
    BodyId, ChangeConstraintOptions, ChangeDynamicsOptions, ControlCommand, JointType,
    PhysicsClient, UrdfOptions,
};
use rubullet::{ControlCommandArray, InverseKinematicsParametersBuilder, LoadModelFlags};

use crate::types::{CartesianPose, GripperState, JointPositions, RobotState, Torques, Vector7};
use std::path::PathBuf;

pub(crate) mod environment;
pub mod sim_camera;
pub mod types;
pub use rubullet;
use rubullet::nalgebra::U7;

/// An abstraction over a robot. Can either be a real robot or a robot in the simulation.
#[allow(clippy::large_enum_variant)]
pub enum Robot {
    Sim(FrankaSim),
    Real(FrankaReal),
}
impl Robot {
    /// sets the angular damping for a simulated robot
    pub fn set_angular_damping(&mut self, damping: f64) {
        match self {
            Robot::Sim(robot) => {
                robot.physics_client.borrow_mut().change_dynamics(
                    robot.id,
                    None,
                    ChangeDynamicsOptions {
                        angular_damping: Some(damping),
                        ..Default::default()
                    },
                );
            }
            Robot::Real(_) => {}
        }
    }
    /// sets the static friction ("stiction") for a simulated robot for torque control.
    pub fn set_joint_friction_force(&mut self, force: f64) {
        assert!(force.is_sign_positive(), "friction force must be positive");
        match self {
            Robot::Sim(robot) => {
                robot.joint_friction_force = force;
            }
            Robot::Real(_) => {}
        }
    }
    /// point to point motion in joint space.
    /// # Arguments
    /// * `speed_factor` - General speed factor in range [0, 1].
    /// * `q_goal` - Target joint positions.
    pub fn joint_motion(&mut self, speed_factor: f64, q_goal: Vector7) {
        match self {
            Robot::Sim(robot) => {
                let mut goal = [0.; 7];
                for i in 0..7 {
                    goal[i] = q_goal[i];
                }
                let mut motion_generator = MotionGenerator::new(speed_factor, &goal);
                let control_callback = |state: &RobotState, step: &Duration| -> JointPositions {
                    let mut franka_state = franka::RobotState::default();
                    for i in 0..7 {
                        franka_state.q_d[i] = state.joint_positions_d[i];
                    }
                    let joint_pos = motion_generator.generate_motion(&franka_state, step);
                    joint_pos.into()
                };
                robot.control_joint_positions(control_callback);
            }
            Robot::Real(robot) => {
                robot
                    .robot
                    .joint_motion(speed_factor, &q_goal.into())
                    .unwrap();
            }
        }
    }
    /// Starts a control loop for a joint position motion generator
    /// # Example
    /// ```no_run
    ///# use franka::Vector7;
    ///# use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
    ///# use franka_interface::types::JointPositions;
    ///# use franka_interface::RobotArguments;
    ///# use std::f64::consts::PI;
    ///# use std::path::PathBuf;
    ///#
    ///# struct MySimSetup {}
    ///# impl SimulationSetup for MySimSetup {
    ///#     fn set_franka_urdf_path(&self) -> PathBuf {
    ///#         "path/to/panda.urdf".into()
    ///#     }
    ///# }
    ///# struct MyRealSetup {}
    ///# impl RealSetup for MyRealSetup {}
    ///# fn main() {
    ///#     let mut experiment = Experiment::new(Mode::Simulation, MySimSetup {}, MyRealSetup {});
    ///#     let mut robot = experiment.new_robot(RobotArguments {
    ///#         hostname: "franka".into(),
    ///#         base_pose: None,
    ///#         initial_config: None,
    ///#     });
    ///     let mut initial_joint_positions = Vector7::zeros();
    ///     let mut time = 0.;
    ///     robot.control_joint_positions(|state, time_step| {
    ///         time += time_step.as_secs_f64();
    ///         if time == 0. {
    ///             initial_joint_positions = state.joint_positions;
    ///         }
    ///         let mut out = JointPositions::from(initial_joint_positions);
    ///         let delta_angle = PI / 8. * (1. - f64::cos(PI / 2.5 * time));
    ///         out.joint_positions[3] += delta_angle;
    ///         out.joint_positions[4] += delta_angle;
    ///         out.joint_positions[6] += delta_angle;
    ///         if time >= 5.0 {
    ///             println!("Finished motion, shutting down example");
    ///             out.is_finished = true;
    ///         }
    ///         out
    ///     })
    ///# }
    /// ```
    pub fn control_joint_positions<F: FnMut(&RobotState, &Duration) -> JointPositions>(
        &mut self,
        control_callback: F,
    ) {
        match self {
            Robot::Sim(robot) => {
                robot.control_joint_positions(control_callback);
            }
            Robot::Real(robot) => {
                robot.control_joint_positions(control_callback);
            }
        }
    }

    /// Starts a control loop for sending joint-level torque commands
    /// # Example
    /// ```no_run
    ///# use franka::Vector7;
    ///# use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
    ///# use franka_interface::types::Torques;
    ///# use franka_interface::RobotArguments;
    ///# use std::f64::consts::PI;
    ///# use std::path::PathBuf;
    ///# use std::time::Duration;
    ///#
    ///# struct MySimSetup {}
    ///# impl SimulationSetup for MySimSetup {
    ///#     fn set_franka_urdf_path(&self) -> PathBuf {
    ///#         "path/to/panda.urdf".into()
    ///#     }
    ///#     fn set_simulation_time_step(&self) -> Duration {
    ///#         Duration::from_millis(1)
    ///#     }
    ///# }
    ///# struct MyRealSetup {}
    ///# impl RealSetup for MyRealSetup {}
    ///# fn main() {
    ///#     let mut experiment = Experiment::new(Mode::Simulation, MySimSetup {}, MyRealSetup {});
    ///#     let mut robot = experiment.new_robot(RobotArguments {
    ///#         hostname: "franka".into(),
    ///#         base_pose: None,
    ///#         initial_config: None,
    ///#     });
    ///#
    ///     let mut time = 0.;
    ///     let mut rotation_force: f64 = 0.01;
    ///     robot.control_torques(|state, time_step| {
    ///         time += time_step.as_secs_f64();
    ///         if state.joint_positions[6] > PI / 4. + 0.1 && rotation_force.is_sign_positive() {
    ///             rotation_force *= -1.;
    ///         }
    ///         if state.joint_positions[6] < PI / 4. - 0.1 && rotation_force.is_sign_negative() {
    ///             rotation_force *= -1.;
    ///         }
    ///         let mut torques = Torques::from(Vector7::from_column_slice(&[
    ///             0.,
    ///             0.,
    ///             0.,
    ///             0.,
    ///             0.,
    ///             0.,
    ///             rotation_force,
    ///         ]));
    ///         if time > 10. {
    ///             torques.is_finished = true;
    ///         }
    ///         torques
    ///     });
    ///# }
    /// ```
    pub fn control_torques<F: FnMut(&RobotState, &Duration) -> Torques>(
        &mut self,
        control_callback: F,
    ) {
        match self {
            Robot::Sim(robot) => {
                robot.control_torques(control_callback);
            }
            Robot::Real(robot) => {
                robot.control_torques(control_callback);
            }
        }
    }
    /// Starts a control loop for a Cartesian pose motion generator
    /// # Example
    /// ```no_run
    ///# use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
    ///# use franka_interface::types::CartesianPose;
    ///# use franka_interface::RobotArguments;
    ///# use nalgebra::Isometry3;
    ///# use std::f64::consts::PI;
    ///# use std::path::PathBuf;
    ///# use std::time::Duration;
    ///#
    ///# struct MySimSetup {}
    ///# impl SimulationSetup for MySimSetup {
    ///#     fn set_franka_urdf_path(&self) -> PathBuf {
    ///#          "path/to/panda.urdf".into()
    ///#     }
    ///#     fn set_simulation_time_step(&self) -> Duration {
    ///#         Duration::from_millis(1)
    ///#     }
    ///# }
    ///# struct MyRealSetup {}
    ///# impl RealSetup for MyRealSetup {}
    ///# fn main() {
    ///#     let mut experiment = Experiment::new(Mode::Simulation, MySimSetup {}, MyRealSetup {});
    ///#     let mut robot = experiment.new_robot(RobotArguments {
    ///#         hostname: "franka".into(),
    ///#         base_pose: None,
    ///#         initial_config: None,
    ///#     });
    ///#
    ///     let mut time = 0.;
    ///     let mut initial_pose = CartesianPose::from(Isometry3::identity());
    ///     robot.control_cartesian_pose(|state, time_step| {
    ///         time += time_step.as_secs_f64();
    ///         if time == 0. {
    ///             initial_pose.pose = state.end_effector_pose;
    ///         }
    ///         let radius = 0.3;
    ///         let angle = PI / 4. * (1. - f64::cos(PI / 5. * time));
    ///         let delta_x = radius * f64::sin(angle);
    ///         let delta_z = radius * (f64::cos(angle) - 1.);
    ///
    ///         let mut out = CartesianPose::from(initial_pose.pose);
    ///         out.pose.translation.x += delta_x;
    ///         out.pose.translation.z += delta_z;
    ///         if time > 10. {
    ///             out.is_finished = true;
    ///         }
    ///         out
    ///     });
    ///# }
    /// ```
    pub fn control_cartesian_pose<F: FnMut(&RobotState, &Duration) -> CartesianPose>(
        &mut self,
        control_callback: F,
    ) {
        match self {
            Robot::Sim(robot) => {
                robot.control_cartesian_pose(control_callback);
            }
            Robot::Real(robot) => {
                robot.control_cartesian_pose(control_callback);
            }
        }
    }
    /// opens the gripper
    /// # Arguments
    /// * `width` - opening width in meter
    pub fn open_gripper(&mut self, width: f64) {
        match self {
            Robot::Sim(robot) => {
                robot.open_gripper(width);
            }
            Robot::Real(robot) => {
                robot.open_gripper(width);
            }
        }
    }
    /// closes the gripper
    pub fn close_gripper(&mut self) {
        match self {
            Robot::Sim(robot) => {
                robot.close_gripper();
            }
            Robot::Real(robot) => {
                robot.close_gripper();
            }
        }
    }
    /// queries the robot state
    pub fn get_state(&mut self) -> RobotState {
        match self {
            Robot::Sim(robot) => robot.get_state(),
            Robot::Real(robot) => robot.get_state(),
        }
    }
    /// queries the gripper state
    pub fn get_gripper_state(&mut self) -> GripperState {
        match self {
            Robot::Sim(robot) => robot.get_gripper_state(),
            Robot::Real(robot) => robot.get_gripper_state(),
        }
    }
    /// queries the current space jacobian
    pub fn get_jacobian(&mut self) -> Matrix6x7 {
        match self {
            Robot::Sim(robot) => robot.get_jacobian(),
            Robot::Real(robot) => robot.get_jacobian(),
        }
    }
}
/// a struct that specifies the argument that are needed to connect to a real robot or spawn a
/// robot inside the simulation.
pub struct RobotArguments {
    /// IP address or hostname of the robot. Only needed when connected to a real robot.
    pub hostname: String,
    /// Base pose of the robot compared to the world frame. Only needed for the simulation.
    pub base_pose: Option<Isometry3<f64>>,
    /// Initial joint configuration of the robot The last two are the gripper positions.
    /// Only needed for the simulation. Default it the home pose.
    pub initial_config: Option<[f64; FrankaSim::PANDA_NUM_TOTAL_DOFS]>,
}

pub(crate) trait FrankaInterface {
    fn control_joint_positions<F: FnMut(&RobotState, &Duration) -> JointPositions>(
        &mut self,
        control_callback: F,
    );
    fn control_torques<F: FnMut(&RobotState, &Duration) -> Torques>(&mut self, control_callback: F);
    fn control_cartesian_pose<F: FnMut(&RobotState, &Duration) -> CartesianPose>(
        &mut self,
        control_callback: F,
    );
    fn open_gripper(&mut self, width: f64);
    fn close_gripper(&mut self);
    fn get_state(&mut self) -> RobotState;
    fn get_gripper_state(&mut self) -> GripperState;
    fn get_jacobian(&mut self) -> Matrix6x7;
    fn get_jacobian_at(&mut self, q: Vector7) -> Matrix6x7;
}
struct Model(franka::Model);
impl Model {
    fn update_state_properties(&self, state: &franka::RobotState) -> RobotState {
        let jacobian = self.0.zero_jacobian_from_state(&Frame::kEndEffector, state);
        let mass_matrix = self.0.mass_from_state(state);
        let coriolis_force = self.0.coriolis_from_state(state);
        let gravity = self.0.gravity_from_state(state, None);
        let mut state: RobotState = state.clone().into();
        state.jacobian = Matrix6x7::from_column_slice(&jacobian);
        state.mass_matrix = Matrix7::from_column_slice(&mass_matrix);
        state.coriolis = coriolis_force.into();
        state.gravity = gravity.into();
        state
    }
}
/// Is responsible for the connection to the real robot.
pub struct FrankaReal {
    robot: franka::Robot,
    model: Model,
    gripper: franka::Gripper,
}

impl FrankaReal {
    pub(crate) fn new(hostname: &str) -> Self {
        let mut robot =
            franka::Robot::new(hostname, None, None).expect("could not connect to robot");
        robot
            .set_collision_behavior(
                [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6],
                [100.; 6],
            )
            .unwrap();
        robot
            .set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])
            .unwrap();
        robot
            .set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])
            .unwrap();
        let gripper = franka::Gripper::new(hostname).expect("could not connect to gripper");
        let model = Model(robot.load_model(false).unwrap());
        FrankaReal {
            robot,
            model,
            gripper,
        }
    }
}

impl FrankaInterface for FrankaReal {
    fn control_joint_positions<F: FnMut(&RobotState, &Duration) -> JointPositions>(
        &mut self,
        mut control_callback: F,
    ) {
        let model = &self.model;
        let new_callback =
            |state: &franka::RobotState, duration: &Duration| -> franka::JointPositions {
                control_callback(&model.update_state_properties(state), duration).into()
            };
        self.robot
            .control_joint_positions(new_callback, None, None, None)
            .unwrap();
    }

    fn control_torques<F: FnMut(&RobotState, &Duration) -> Torques>(
        &mut self,
        mut control_callback: F,
    ) {
        let model = &self.model;
        let new_callback = |state: &franka::RobotState, duration: &Duration| -> franka::Torques {
            control_callback(&model.update_state_properties(state), duration).into()
        };
        let robot = &mut self.robot;
        robot
            .set_collision_behavior(
                [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6],
                [100.; 6],
            )
            .unwrap();
        robot
            .set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])
            .unwrap();
        robot
            .set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])
            .unwrap();
        robot.control_torques(new_callback, None, None).unwrap();
    }

    fn control_cartesian_pose<F: FnMut(&RobotState, &Duration) -> CartesianPose>(
        &mut self,
        mut control_callback: F,
    ) {
        let model = &self.model;
        let new_callback =
            |state: &franka::RobotState, duration: &Duration| -> franka::CartesianPose {
                control_callback(&model.update_state_properties(state), duration).into()
            };
        self.robot
            .control_cartesian_pose(new_callback, None, None, None)
            .unwrap();
    }

    fn open_gripper(&mut self, width: f64) {
        self.gripper.move_gripper(width, 0.1).unwrap();
    }

    fn close_gripper(&mut self) {
        self.gripper.grasp(0.0, 0.1, 10., None, None).unwrap();
    }

    fn get_state(&mut self) -> RobotState {
        let state = self.robot.read_once().unwrap();
        self.model.update_state_properties(&state)
    }

    fn get_gripper_state(&mut self) -> GripperState {
        self.gripper.read_once().unwrap().into()
    }

    fn get_jacobian(&mut self) -> Matrix6x7 {
        let q = self.get_state().joint_positions;
        self.get_jacobian_at(q)
    }
    fn get_jacobian_at(&mut self, q: Vector7) -> Matrix6x7 {
        let f_t_ee = [
            1., 0., 0., 1., 0., 1., 0., 1., 0., 0., 1., 1., 0., 0., 0.07, 1.,
        ];
        let jacobian =
            self.model
                .0
                .zero_jacobian(&Frame::kEndEffector, q.as_ref(), &f_t_ee, &[0.; 16]);
        Matrix6x7::from_column_slice(jacobian.as_ref())
    }
}
/// A struct representing a simulated Panda robot.
pub struct FrankaSim {
    pub(crate) physics_client: Rc<RefCell<PhysicsClient>>,
    /// BodyId of the robot in RuBullet
    pub id: BodyId,
    /// the duration of a control loop cycle
    pub time_step: Duration,
    /// pose of the robot base relative to the world frame
    pub base_pose: Isometry3<f64>,
    pub joint_friction_force: f64,
    pub robot_joint_indices: [usize; FrankaSim::PANDA_NUM_DOFS],
    pub gripper_joint_indices: [usize; FrankaSim::GRIPPER_JOINT_NAMES.len()],
    pub end_effector_link_index: usize,
}

impl FrankaSim {
    const INITIAL_JOINT_POSITIONS: [f64; FrankaSim::PANDA_NUM_TOTAL_DOFS] = [
        0.,
        -PI / 4.,
        0.,
        -3. * PI / 4.,
        0.,
        PI / 2.,
        PI / 4.,
        0.02,
        0.02,
    ];
    const PANDA_NUM_DOFS: usize = 7;
    const GRIPPER_OPEN_WIDTH: f64 = 0.08;
    const ROBOT_JOINT_NAMES: [&'static str; FrankaSim::PANDA_NUM_DOFS] = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ];
    const GRIPPER_JOINT_NAMES: [&'static str; 2] = ["panda_finger_joint1", "panda_finger_joint2"];
    const END_EFFECTOR_LINK_NAME: &'static str = "panda_grasptarget";
    const PANDA_NUM_TOTAL_DOFS: usize =
        FrankaSim::PANDA_NUM_DOFS + FrankaSim::GRIPPER_JOINT_NAMES.len();
}

pub(crate) struct FrankaSimWithoutClient {
    pub urdf_path: PathBuf,
    pub base_pose: Option<Isometry3<f64>>,
    pub initial_config: Option<[f64; FrankaSim::PANDA_NUM_TOTAL_DOFS]>,
}

impl FrankaSim {
    fn wait_for_next_cycle(&self, start_time: Instant) {
        let time_passed = Instant::now() - start_time;
        if time_passed < self.time_step {
            std::thread::sleep(self.time_step - time_passed);
        }
    }
    pub(crate) fn new(
        physics_client: Rc<RefCell<PhysicsClient>>,
        config: FrankaSimWithoutClient,
        time_step: &Duration,
    ) -> Self {
        let position = config.base_pose.unwrap_or_else(|| {
            Isometry3::rotation(
                UnitQuaternion::from_euler_angles(-PI / 2. * 0., 0., 0.).scaled_axis(),
            )
        });
        let initial_config = config
            .initial_config
            .unwrap_or(FrankaSim::INITIAL_JOINT_POSITIONS);
        if let Some(directory) = config.urdf_path.parent() {
            physics_client
                .borrow_mut()
                .set_additional_search_path(directory)
                .unwrap();
        }
        let urdf_options = UrdfOptions {
            use_fixed_base: true,
            base_transform: position,
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };

        let panda_id = physics_client
            .borrow_mut()
            .load_urdf(config.urdf_path.file_name().unwrap(), urdf_options)
            .unwrap();
        physics_client.borrow_mut().change_dynamics(
            panda_id,
            None,
            ChangeDynamicsOptions {
                linear_damping: Some(0.),
                angular_damping: Some(0.),
                ..Default::default()
            },
        );
        let mut robot_joint_indices: [Option<usize>; Self::PANDA_NUM_DOFS] =
            [None; Self::PANDA_NUM_DOFS];
        let mut gripper_joint_indices = [None; Self::GRIPPER_JOINT_NAMES.len()];
        let mut end_effector_link_index = None;
        let num_joints = physics_client.borrow_mut().get_num_joints(panda_id);
        for i in 0..num_joints {
            let joint_info = physics_client.borrow_mut().get_joint_info(panda_id, i);
            let joint_name = joint_info.joint_name;
            if let Some(joint_index) = FrankaSim::ROBOT_JOINT_NAMES
                .iter()
                .position(|&x| x == joint_name)
            {
                robot_joint_indices[joint_index] = Some(i);
            }
            if let Some(joint_index) = FrankaSim::GRIPPER_JOINT_NAMES
                .iter()
                .position(|&x| x == joint_name)
            {
                gripper_joint_indices[joint_index] = Some(i);
            }
            if joint_info.link_name == FrankaSim::END_EFFECTOR_LINK_NAME {
                end_effector_link_index = Some(i);
            }
        }
        let end_effector_link_index = end_effector_link_index.unwrap_or_else(|| {
            panic!(
                "Could not find end-effector link: \"{}\" in URDF",
                FrankaSim::END_EFFECTOR_LINK_NAME
            )
        });
        fn turn_options_into_values<F: Fn(usize) -> String, const N: usize>(
            array: [Option<usize>; N],
            error_handler: F,
        ) -> [usize; N] {
            array
                .iter()
                .enumerate()
                .map(|(i, x)| x.unwrap_or_else(|| panic!("{}", error_handler(i))))
                .collect::<Vec<_>>()
                .try_into()
                .unwrap()
        }

        let robot_joint_indices = turn_options_into_values(robot_joint_indices, |i| {
            format!(
                "Could not find joint: \"{}\" in URDF",
                FrankaSim::ROBOT_JOINT_NAMES[i]
            )
        });
        let gripper_joint_indices = turn_options_into_values(gripper_joint_indices, |i| {
            format!(
                "Could not find joint: \"{}\" in URDF",
                FrankaSim::GRIPPER_JOINT_NAMES[i]
            )
        });

        for i in robot_joint_indices {
            physics_client.borrow_mut().change_dynamics(
                panda_id,
                i,
                ChangeDynamicsOptions {
                    joint_damping: Some(0.0),
                    ..Default::default()
                },
            );
        }

        let mut index = 0;
        for i in robot_joint_indices {
            let info = physics_client.borrow_mut().get_joint_info(panda_id, i);
            if info.joint_type == JointType::Revolute || info.joint_type == JointType::Prismatic {
                physics_client
                    .borrow_mut()
                    .reset_joint_state(panda_id, i, initial_config[index], None)
                    .unwrap();
                index += 1;
            }
        }

        let gripper_constraint = physics_client
            .borrow_mut()
            .create_constraint(
                panda_id,
                gripper_joint_indices[0],
                panda_id,
                gripper_joint_indices[1],
                JointType::Gear,
                [1., 0., 0.],
                Isometry3::identity(),
                Isometry3::identity(),
            )
            .unwrap();
        physics_client.borrow_mut().change_constraint(
            gripper_constraint,
            ChangeConstraintOptions {
                gear_ratio: Some(-1.),
                erp: Some(0.1),
                max_force: Some(50.),
                ..Default::default()
            },
        );
        physics_client.borrow_mut().step_simulation().unwrap();
        FrankaSim {
            physics_client,
            id: panda_id,
            time_step: *time_step,
            base_pose: config.base_pose.unwrap_or_else(Isometry3::identity),
            joint_friction_force: 0.,
            robot_joint_indices,
            gripper_joint_indices,
            end_effector_link_index,
        }
    }
}

impl FrankaInterface for FrankaSim {
    fn control_joint_positions<F: FnMut(&RobotState, &Duration) -> JointPositions>(
        &mut self,
        mut control_callback: F,
    ) {
        let mut first_time = true;
        loop {
            let start_time = Instant::now();
            let joint_positions = match first_time {
                true => {
                    first_time = false;
                    control_callback(&self.get_state(), &Duration::from_secs(0))
                }
                false => control_callback(&self.get_state(), &self.time_step),
            };

            self.physics_client
                .borrow_mut()
                .set_joint_motor_control_array(
                    self.id,
                    &self.robot_joint_indices,
                    ControlCommandArray::Positions(joint_positions.joint_positions.as_slice()),
                    None,
                )
                .unwrap();
            self.physics_client.borrow_mut().step_simulation().unwrap();
            std::thread::sleep(self.time_step);
            self.wait_for_next_cycle(start_time);
            if joint_positions.is_finished {
                return;
            }
        }
    }

    fn control_torques<F: FnMut(&RobotState, &Duration) -> Torques>(
        &mut self,
        mut control_callback: F,
    ) {
        assert!(f64::abs(self.time_step.as_secs_f64() - 0.001) < 1e-5, "the simulation time step needs to be set to 1 Millisecond to perform torque control. See set_simulation_time_step");
        self.physics_client
            .borrow_mut()
            .set_joint_motor_control_array(
                self.id,
                &self.robot_joint_indices,
                ControlCommandArray::Velocities(&[0.; Self::PANDA_NUM_DOFS]),
                Some(&[self.joint_friction_force; Self::PANDA_NUM_DOFS]),
            )
            .unwrap();
        let mut first_time = true;
        loop {
            let start_time = Instant::now();
            let mut torques = match first_time {
                true => {
                    first_time = false;
                    control_callback(&self.get_state(), &Duration::from_secs(0))
                }
                false => control_callback(&self.get_state(), &self.time_step),
            };
            let state = self.get_state();

            let mut pos = [0.; FrankaSim::PANDA_NUM_TOTAL_DOFS];
            let mut vels = [0.; FrankaSim::PANDA_NUM_TOTAL_DOFS];
            for k in 0..Self::PANDA_NUM_DOFS {
                pos[k] = state.joint_positions_d[k];
                vels[k] = state.joint_velocities[k];
            }

            let gravity_torques = self
                .physics_client
                .borrow_mut()
                .calculate_inverse_dynamics(
                    self.id,
                    &pos,
                    &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
                    &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
                )
                .unwrap();
            let mut coriolis = self
                .physics_client
                .borrow_mut()
                .calculate_inverse_dynamics(
                    self.id,
                    &pos,
                    &vels,
                    &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
                )
                .unwrap();
            for i in 0..Self::PANDA_NUM_DOFS {
                coriolis[i] -= gravity_torques[i];
            }

            torques.torques += state.gravity;
            self.physics_client
                .borrow_mut()
                .set_joint_motor_control_array(
                    self.id,
                    &self.robot_joint_indices,
                    ControlCommandArray::Torques(torques.torques.as_slice()),
                    None,
                )
                .unwrap();
            self.physics_client.borrow_mut().step_simulation().unwrap();
            self.wait_for_next_cycle(start_time);
            if torques.is_finished {
                return;
            }
        }
    }

    fn control_cartesian_pose<F: FnMut(&RobotState, &Duration) -> CartesianPose>(
        &mut self,
        mut control_callback: F,
    ) {
        let mut first_time = true;
        loop {
            let start_time = Instant::now();
            let pose = match first_time {
                true => {
                    first_time = false;
                    control_callback(&self.get_state(), &Duration::from_secs(0))
                }
                false => control_callback(&self.get_state(), &self.time_step),
            };
            let desired_world_pose = self.base_pose * pose.pose;
            let inverse_kinematics_parameters = InverseKinematicsParametersBuilder::new(
                self.end_effector_link_index,
                &desired_world_pose,
            )
            .set_max_num_iterations(50)
            .build();
            let joint_poses = self
                .physics_client
                .borrow_mut()
                .calculate_inverse_kinematics(self.id, inverse_kinematics_parameters)
                .unwrap();
            self.physics_client
                .borrow_mut()
                .set_joint_motor_control_array(
                    self.id,
                    &self.robot_joint_indices,
                    ControlCommandArray::Positions(&joint_poses[0..Self::PANDA_NUM_DOFS]),
                    None,
                )
                .unwrap();
            self.physics_client.borrow_mut().step_simulation().unwrap();
            self.wait_for_next_cycle(start_time);
            if pose.is_finished {
                return;
            }
        }
    }

    fn open_gripper(&mut self, width: f64) {
        let steps = (1. / self.time_step.as_secs_f64()) as i32;
        let mut client = self.physics_client.borrow_mut();
        let width = f64::clamp(width, 0., FrankaSim::GRIPPER_OPEN_WIDTH);
        for _ in 0..steps {
            let start_time = Instant::now();
            client.set_joint_motor_control(
                self.id,
                self.gripper_joint_indices[1],
                ControlCommand::Position(width / 2.),
                Some(1.),
            );
            client.set_joint_motor_control(
                self.id,
                self.gripper_joint_indices[0],
                ControlCommand::Position(width / 2.),
                Some(1.),
            );
            client.step_simulation().unwrap();
            self.wait_for_next_cycle(start_time);
        }
    }

    fn close_gripper(&mut self) {
        let start_width = self.get_gripper_state().gripper_width / 2.;
        let mut client = self.physics_client.borrow_mut();
        let steps = (1. / self.time_step.as_secs_f64()) as i32;
        for i in 0..steps {
            let start_time = Instant::now();
            let val = start_width * (f64::cos((i) as f64 / steps as f64 * PI) / 2.);
            client.set_joint_motor_control(
                self.id,
                self.gripper_joint_indices[1],
                ControlCommand::Position(val),
                Some(10.),
            );
            client.set_joint_motor_control(
                self.id,
                self.gripper_joint_indices[0],
                ControlCommand::Position(val),
                Some(10.),
            );
            client.step_simulation().unwrap();
            self.wait_for_next_cycle(start_time);
        }
    }

    fn get_state(&mut self) -> RobotState {
        let indices = [
            self.robot_joint_indices[0],
            self.robot_joint_indices[1],
            self.robot_joint_indices[2],
            self.robot_joint_indices[3],
            self.robot_joint_indices[4],
            self.robot_joint_indices[5],
            self.robot_joint_indices[6],
            self.gripper_joint_indices[0],
            self.gripper_joint_indices[1],
        ]; // oh boy, we really need const generics in Rust.
        let joint_states = self
            .physics_client
            .borrow_mut()
            .get_joint_states(self.id, &indices)
            .unwrap();
        let mut joint_positions = [0.; FrankaSim::PANDA_NUM_TOTAL_DOFS];
        let mut joint_velocities = [0.; FrankaSim::PANDA_NUM_TOTAL_DOFS];
        for i in 0..FrankaSim::PANDA_NUM_TOTAL_DOFS {
            joint_positions[i] = joint_states[i].joint_position;
            joint_velocities[i] = joint_states[i].joint_velocity;
        }

        let end_effector_state = self
            .physics_client
            .borrow_mut()
            .get_link_state(self.id, self.end_effector_link_index, true, true)
            .unwrap();
        let end_effector_pose = self.base_pose.inverse() * end_effector_state.world_pose;

        let jacobian = self.get_jacobian();
        let mass_matrix = self
            .physics_client
            .borrow_mut()
            .calculate_mass_matrix(self.id, &joint_positions)
            .unwrap();
        let tmp = mass_matrix.fixed_slice::<U7, U7>(0, 0);
        let mass_matrix: Matrix7 = tmp.into();
        let coriolis_force = self
            .physics_client
            .borrow_mut()
            .calculate_inverse_dynamics(
                self.id,
                &joint_positions,
                &joint_velocities,
                &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
            )
            .unwrap();

        let gravity_torques = self
            .physics_client
            .borrow_mut()
            .calculate_inverse_dynamics(
                self.id,
                &joint_positions,
                &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
                &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
            )
            .unwrap();
        let gravity = Vector7::from_column_slice(&gravity_torques[0..Self::PANDA_NUM_DOFS]);
        let coriolis_force =
            Vector7::from_column_slice(&coriolis_force[0..Self::PANDA_NUM_DOFS]) - gravity;

        let mut joint_positions = Vector7::zeros();
        let mut joint_velocities = Vector7::zeros();
        let mut joint_torques = Vector7::zeros();
        for i in 0..Self::PANDA_NUM_DOFS {
            joint_positions[i] = joint_states[i].joint_position;
            joint_velocities[i] = joint_states[i].joint_velocity;
            joint_torques[i] = joint_states[i].joint_motor_torque;
        }
        RobotState {
            joint_positions_d: joint_positions,
            joint_positions,
            joint_velocities,
            joint_velocities_d: joint_velocities,
            joint_torques,
            end_effector_pose_c: end_effector_pose,
            end_effector_pose,
            end_effector_pose_d: end_effector_pose,
            end_effector_velocity_d: jacobian * joint_velocities,
            jacobian,
            mass_matrix,
            coriolis: coriolis_force,
            gravity,
            control_command_success_rate: 1.,
        }
    }

    fn get_gripper_state(&mut self) -> GripperState {
        let width = self
            .physics_client
            .borrow_mut()
            .get_joint_state(self.id, self.gripper_joint_indices[0])
            .unwrap()
            .joint_position
            * 2.;

        GripperState {
            gripper_width: width,
            closed: width < FrankaSim::GRIPPER_OPEN_WIDTH - 0.01,
        }
    }
    fn get_jacobian(&mut self) -> Matrix6x7 {
        let joint_states = self
            .physics_client
            .borrow_mut()
            .get_joint_states(self.id, &self.robot_joint_indices)
            .unwrap();
        let mut joint_positions = Vector7::zeros();
        for i in 0..Self::PANDA_NUM_DOFS {
            joint_positions[i] = joint_states[i].joint_position;
        }
        self.get_jacobian_at(joint_positions)
    }

    fn get_jacobian_at(&mut self, q: Vector7) -> Matrix6x7 {
        let mut pos = [0.; FrankaSim::PANDA_NUM_TOTAL_DOFS];
        let joint_positions = q;

        for i in 0..Self::PANDA_NUM_DOFS {
            pos[i] = joint_positions[i];
        }
        let jac = self
            .physics_client
            .borrow_mut()
            .calculate_jacobian(
                self.id,
                self.end_effector_link_index,
                Translation3::from(Vector3::zeros()),
                &pos,
                &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
                &[0.; FrankaSim::PANDA_NUM_TOTAL_DOFS],
            )
            .unwrap();
        Matrix6x7::from_column_slice(jac.jacobian.as_slice())
    }
}
