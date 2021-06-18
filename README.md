[![crates.io](https://img.shields.io/crates/v/franka-interface.svg)](https://crates.io/crates/franka-interface)
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/marcbone/franka-interface/Rust)
[![crates.io](https://img.shields.io/crates/l/franka-interface.svg)](https://crates.io/crates/franka-interface)
[![crates.io](https://img.shields.io/crates/d/franka-interface.svg)](https://crates.io/crates/franka-interface)
[![docs.rs](https://docs.rs/franka-interface/badge.svg)](https://docs.rs/franka-interface)
# franka-interface

franka-interface allows developing software for controlling Franka Emika Panda robots in simulation and then run it on
the real hardware.

This is done by providing an abstraction over [libfranka-rs](https://github.com/marcbone/libfranka-rs) and [RuBullet](https://github.com/neachdainn/rubullet).
![](https://i.imgur.com/M6Mi8qr.png)
## Example
```rust
use franka_interface::experiment::{Experiment, Mode, RealSetup, SimulationSetup};
use franka_interface::RobotArguments;
use std::f64::consts::PI;
use std::path::PathBuf;

struct MySimSetup {}
impl SimulationSetup for MySimSetup {
    fn set_franka_urdf_path(&self) -> PathBuf {
        "path/to/panda.urdf".into()
    }
}
struct MyRealSetup {}
impl RealSetup for MyRealSetup {}
fn main() {
    let mode = Mode::Simulation;
    let mut experiment = Experiment::new(mode, MySimSetup {}, MyRealSetup {});
    let mut robot = experiment.new_robot(RobotArguments {
        name: "franka-ip".to_string(), // IP-Address or hostname of the real Panda (Not needed for the simulation)
        base_pose: None,
        initial_config: None,
    });
    robot.joint_motion(
        0.1,
        [1., PI / 4., 0., -2. * PI / 4., 0., PI / 2., -PI / 4.].into(),
    );
}
```

This example will spawn the panda robot in RuBullet and perform a join movement.
To run the program on the real robot it is necessary to set the mode to ```Mode::Real```.
Also, you have to build the program in `Release`-Mode or with `opt-level = 3` to run it on the real robot.

The URDF of the panda can be downloaded from the
[bullet3](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data/franka_panda) repository.
Make sure to also download the mesh files.



## Features
Robots in franka-interface do allow the following control modes:
* Joint position control
* Cartesian pose control
* Torque control
* Joint motions by specifying a goal configuration (see example above)

The robot has access to the following properties:
* Joint position
* Joint velocity
* Cartesian pose
* Jacobian
* Mass matrix
* Coriolis torques
* Gravity torques

Further, the library is very flexible as the user can define custom mechanisms for interacting with
the simulation or the real robot. For example, it is possible to create a camera and get images from
the simulation or adding visualization markers inside the simulation.
Take a look at the examples in the examples folder.

## Limitations
While it is possible to spawn/connect to multiple robots, it is not possible to control multiple
robots at the same time.

## Licence
This library is copyrighted © 2021 Marco Boneberger


Licensed under the EUPL, Version 1.2 or – as soon they will be approved by the European Commission - subsequent versions of the EUPL (the "Licence");

You may not use this work except in compliance with the Licence.
You may obtain a copy of the Licence at:

[https://joinup.ec.europa.eu/software/page/eupl](https://joinup.ec.europa.eu/software/page/eupl)

Unless required by applicable law or agreed to in writing, software distributed under the Licence is distributed on an "AS IS" basis
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the Licence for the specific language governing permissions and limitations under the Licence.
