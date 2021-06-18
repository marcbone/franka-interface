// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use nalgebra::Vector3;
use rubullet::Mode::Gui;
use rubullet::PhysicsClient;

/// Contains either a simulation environment or a real environment.
/// It can be convenient to access the simulation environment to get access to the physics client
/// to directly interact with the simulation.
pub enum FrankaEnvironment {
    Simulation(Box<FrankaSimEnvironment>),
    Real(Box<FrankaRealEnvironment>),
}

/// A simulation environment which holds the [`PhysicsClient`](`rubullet::PhysicsClient`) and the
/// time_step.
pub struct FrankaSimEnvironment {
    /// The PhysicsClient. Use it to directly interact with the simulation.
    pub client: Rc<RefCell<PhysicsClient>>,
    /// How often does the simulation get updated when you move the robot.
    pub time_step: Duration,
}
impl Default for FrankaSimEnvironment {
    fn default() -> Self {
        Self::new(Duration::from_secs_f64(0.001))
    }
}
impl FrankaSimEnvironment {
    pub(crate) fn new(time_step: Duration) -> FrankaSimEnvironment {
        let client = Rc::new(RefCell::new(PhysicsClient::connect(Gui).unwrap()));
        client
            .borrow_mut()
            .set_gravity(Vector3::new(0.0, 0.0, -9.81));

        client.borrow_mut().set_time_step(time_step);
        client
            .borrow_mut()
            .set_additional_search_path("../rubullet-github/bullet3/libbullet3/data")
            .unwrap();
        FrankaSimEnvironment { client, time_step }
    }
}

/// Environment for the real robot. Contains nothing.
pub struct FrankaRealEnvironment {}

impl FrankaRealEnvironment {
    pub(crate) fn new() -> FrankaRealEnvironment {
        FrankaRealEnvironment {}
    }
}
