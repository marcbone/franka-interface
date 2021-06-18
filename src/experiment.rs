// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//! Contains the Experiment struct, which is needed to spawn and connect to robots

use crate::environment::{FrankaEnvironment, FrankaRealEnvironment, FrankaSimEnvironment};
use crate::{FrankaReal, FrankaSim, FrankaSimWithoutClient, Robot, RobotArguments};
use rubullet::image::RgbaImage;
use rubullet::PhysicsClient;
use std::cell::{RefCell, RefMut};
use std::path::PathBuf;
use std::rc::Rc;
use std::time::Duration;

/// choose whether to run your [`Experiment`](`Experiment`) in the simulation or with the real robot
pub enum Mode {
    /// Runs the experiment in the simulation.
    Simulation,
    /// Runs the experiment on the real robot.
    Real,
}
/// A trait which contains methods for handling the simulation. Apart from
/// [`set_franka_urdf_path`](`Self::set_franka_urdf_path`),
/// all methods have a default implementation,
/// so you do not have to implement them if you do not need them.
pub trait SimulationSetup {
    /// Sets how often the simulation should be updated inside the control loop. The default is
    /// 1/240s  = 240 Hz
    fn set_simulation_time_step(&self) -> Duration {
        Duration::from_secs_f64(1. / 240.)
    }
    /// Return the path to the URDF file of the Franka robot.
    fn set_franka_urdf_path(&self) -> PathBuf;
    /// this method runs directly after the simulation is spawned. Use this method for example
    /// to load additional object into the simulation.
    fn setup_simulation(&mut self, _client: Rc<RefCell<PhysicsClient>>) {}
    /// this method is intended to let you setup the camera. It is run directly after setup_simulation.
    fn setup_camera(&mut self, _client: Rc<RefCell<PhysicsClient>>) {}
    /// determine how you want to get the camera image with this method.
    fn get_camera_image(&mut self, _client: Rc<RefCell<PhysicsClient>>) -> RgbaImage {
        RgbaImage::default()
    }
}
/// A trait which contains methods for handling the real robot. All methods have a default implementation,
/// so you do not have to implement them if you do not need them.
pub trait RealSetup {
    /// This method runs directly on calling [`Experiment::new`](`Experiment::new`). You can use
    /// it to run specific code which is only needed when using the real hardware.
    fn setup_real(&mut self) {}
    /// Runs directly after [`setup_real`](`Self::setup_real`) and is intended for setting up the camera.
    fn setup_camera(&mut self) {}
    /// determine how you want to get the camera image with this method.
    fn get_camera_image(&mut self) -> RgbaImage {
        RgbaImage::default()
    }
}
/// Use it to create a new experiment which can either run on the real robot or in the simulation.
///
/// Use [`new`](`Self::new`) to create an experiment and then use [`new_robot`](`Self::new_robot`) to
/// create a new robot.
///
pub struct Experiment<Sim: ?Sized + SimulationSetup, Real: ?Sized + RealSetup> {
    pub environment: FrankaEnvironment,
    pub sim_setup: Box<Sim>,
    pub real_setup: Box<Real>,
}

impl Experiment<dyn SimulationSetup, dyn RealSetup> {
    /// Use it to create a new experiment which can either run on the real robot or in the simulation
    ///
    /// # Arguments
    /// * `mode` - specify whether to run the experiment in simulation or with a real robot
    /// * `sim_setup` - something that implements [`SimulationSetup`](`SimulationSetup`).
    /// * `real_setup` - something that implements [`RealSetup`](`RealSetup`).
    ///
    /// # Example
    /// ```no_run
    /// use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
    /// use franka_interface::types::Vector7;
    /// use franka_interface::RobotArguments;
    /// use std::f64::consts::PI;
    /// use std::path::PathBuf;
    /// struct MySimSetup {}
    /// impl SimulationSetup for MySimSetup {
    ///     fn set_franka_urdf_path(&self) -> PathBuf {
    ///         "path/to/panda.urdf".into()
    ///     }
    /// }
    /// struct MyRealSetup {}
    /// impl RealSetup for MyRealSetup {}
    /// let mut env = Experiment::new(Mode::Simulation, MySimSetup {}, MyRealSetup {});
    /// let mut robot = env.new_robot(RobotArguments {
    ///     hostname: "franka".to_string(),
    ///     base_pose: None,
    ///     initial_config: None,
    /// });
    /// robot.joint_motion(
    ///     0.1,
    ///     Vector7::from_column_slice(&[1., PI / 4., 0., -2. * PI / 4., 0., PI / 2., -PI / 4.]),
    /// );
    /// println!("{:?}", robot.get_state());
    /// ```
    pub fn new(
        mode: Mode,
        mut sim_setup: impl SimulationSetup + 'static,
        mut real_setup: impl RealSetup + 'static,
    ) -> Experiment<dyn SimulationSetup, dyn RealSetup> {
        match mode {
            Mode::Simulation => {
                let environment = Box::new(FrankaSimEnvironment::new(
                    sim_setup.set_simulation_time_step(),
                ));

                sim_setup.setup_simulation(environment.client.clone());
                sim_setup.setup_camera(environment.client.clone());
                let environment = FrankaEnvironment::Simulation(environment);
                Experiment {
                    environment,
                    sim_setup: Box::new(sim_setup),
                    real_setup: Box::new(real_setup),
                }
            }
            Mode::Real => {
                let environment = FrankaEnvironment::Real(Box::new(FrankaRealEnvironment::new()));
                real_setup.setup_real();
                real_setup.setup_camera();
                Experiment {
                    environment,
                    sim_setup: Box::new(sim_setup),
                    real_setup: Box::new(real_setup),
                }
            }
        }
    }
    /// returns the current image from the robot. Make sure you implemented `get_camera_image`
    /// in your [`RealSetup`](`RealSetup`) and [`SimulationSetup`](`SimulationSetup`).
    pub fn get_image(&mut self) -> RgbaImage {
        match &self.environment {
            FrankaEnvironment::Real(_) => self.real_setup.get_camera_image(),
            FrankaEnvironment::Simulation(environment) => {
                self.sim_setup.get_camera_image(environment.client.clone())
            }
        }
    }

    /// spawns/connects to a new robot using the [`RobotArguments`](`crate::RobotArguments`)
    pub fn new_robot(&mut self, config: RobotArguments) -> Robot {
        match &mut self.environment {
            FrankaEnvironment::Real(_environment) => {
                Robot::Real(FrankaReal::new(config.hostname.as_str()))
            }
            FrankaEnvironment::Simulation(environment) => {
                let args = FrankaSimWithoutClient {
                    urdf_path: self.sim_setup.set_franka_urdf_path(),
                    base_pose: config.base_pose,
                    initial_config: config.initial_config,
                };
                Robot::Sim(FrankaSim::new(
                    environment.client.clone(),
                    args,
                    &environment.time_step,
                ))
            }
        }
    }
    /// Query whether the current experiment is run in simulation
    pub fn is_simulation(&self) -> bool {
        match self.environment {
            FrankaEnvironment::Simulation(_) => true,
            FrankaEnvironment::Real(_) => false,
        }
    }
    /// allows accessing the PhysicsClient in Simulation by defining a closure that takes
    /// the PhysicsClient as input.
    /// Nothing will happen if the experiment is not run in the simulation
    /// # Return
    /// * In the simulation it will return Some(T) where T is the return type of the client_cb
    /// * Will return None when not executed with the simulation.
    pub fn use_physics_client<F: FnMut(RefMut<PhysicsClient>) -> T, T>(
        &mut self,
        mut client_cb: F,
    ) -> Option<T> {
        match &self.environment {
            FrankaEnvironment::Simulation(sim) => Some(client_cb(sim.client.borrow_mut())),
            FrankaEnvironment::Real(_) => None,
        }
    }
}
