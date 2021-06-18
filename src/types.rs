// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//! Contains the control types and types for the robot and gripper state
pub use franka::Matrix7;
pub use franka::Vector7;
use franka::{Finishable, Matrix6x7};
use nalgebra::{Isometry3, Vector6};

/// Describes the State of the Robot at a certain point in time.
#[derive(Debug)]
pub struct RobotState {
    /// desired join positions
    pub joint_positions_d: Vector7,
    /// measured joint positions
    pub joint_positions: Vector7,
    /// measured joint velocities
    pub joint_velocities: Vector7,
    /// desired joint velocities
    pub joint_velocities_d: Vector7,
    /// measured joint torques
    pub joint_torques: Vector7,
    /// command end-effector pose. Use it cartesian pose control
    pub end_effector_pose_c: Isometry3<f64>,
    /// measured end-effector pose.
    pub end_effector_pose: Isometry3<f64>,
    /// desired end-effector pose
    pub end_effector_pose_d: Isometry3<f64>,
    /// desired end-effector velocity (linear,angular)
    pub end_effector_velocity_d: Vector6<f64>,
    /// Jacobian at the current configuration
    pub jacobian: Matrix6x7,
    /// Mass matrix at the current configuration
    pub mass_matrix: Matrix7,
    /// Coriolis torques at the current configuration
    pub coriolis: Vector7,
    /// Gravity torques at the current configuration
    pub gravity: Vector7,
    /// Percentage of the last 100 robot commands that the robot successfully received
    pub control_command_success_rate: f64,
}
/// Describes the state of the Gripper at a certain point in time.
pub struct GripperState {
    /// current gripper width in meter
    pub gripper_width: f64,
    /// whether the gripper grasped an object.
    pub closed: bool,
}
/// Describes the desired Torques that will be send to the robot.
pub struct Torques {
    /// desired joint torques
    pub torques: Vector7,
    /// whether the robot should stop the control loop after this message.
    pub is_finished: bool,
}

impl From<franka::Torques> for Torques {
    fn from(t: franka::Torques) -> Self {
        let is_finished = t.is_finished();
        Torques {
            torques: Vector7::from_column_slice(&t.tau_J),
            is_finished,
        }
    }
}

impl From<Torques> for franka::Torques {
    fn from(t: Torques) -> Self {
        let is_finished = t.is_finished;
        let mut out = franka::Torques::new(t.torques.into());
        out.set_motion_finished(is_finished);
        out
    }
}

impl From<Vector7> for Torques {
    fn from(vec: Vector7) -> Self {
        Torques {
            torques: vec,
            is_finished: false,
        }
    }
}
/// Describes the desired joint positions that will be send to the robot.
pub struct JointPositions {
    /// desired joint positions
    pub joint_positions: Vector7,
    /// whether the robot should stop the control loop after this message.
    pub is_finished: bool,
}

impl From<franka::JointPositions> for JointPositions {
    fn from(t: franka::JointPositions) -> Self {
        let is_finished = t.is_finished();
        JointPositions {
            joint_positions: Vector7::from_column_slice(&t.q),
            is_finished,
        }
    }
}

impl From<JointPositions> for franka::JointPositions {
    fn from(t: JointPositions) -> Self {
        let is_finished = t.is_finished;
        let mut out = franka::JointPositions::new(t.joint_positions.into());
        out.set_motion_finished(is_finished);
        out
    }
}

impl From<Vector7> for JointPositions {
    fn from(vec: Vector7) -> Self {
        JointPositions {
            joint_positions: vec,
            is_finished: false,
        }
    }
}
/// Describes the desired cartesian pose that will be send to the robot.
pub struct CartesianPose {
    /// desired end-effector pose
    pub pose: Isometry3<f64>,
    /// whether the robot should stop the control loop after this message.
    pub is_finished: bool,
}

impl From<franka::CartesianPose> for CartesianPose {
    fn from(pose: franka::CartesianPose) -> Self {
        CartesianPose {
            pose: franka::utils::array_to_isometry(&pose.O_T_EE),
            is_finished: pose.is_finished(),
        }
    }
}

impl From<CartesianPose> for franka::CartesianPose {
    fn from(pose: CartesianPose) -> Self {
        let is_finished = pose.is_finished;
        let mut out: franka::CartesianPose = pose.pose.into();
        out.set_motion_finished(is_finished);
        out
    }
}

impl From<Isometry3<f64>> for CartesianPose {
    fn from(pose: Isometry3<f64>) -> Self {
        CartesianPose {
            pose,
            is_finished: false,
        }
    }
}

impl From<franka::RobotState> for RobotState {
    fn from(state: franka::RobotState) -> Self {
        RobotState {
            joint_positions_d: Vector7::from_column_slice(&state.q_d),
            joint_positions: Vector7::from_column_slice(&state.q),
            joint_velocities: Vector7::from_column_slice(&state.dq),
            joint_velocities_d: Vector7::from_column_slice(&state.dq),
            joint_torques: Vector7::from_column_slice(&state.tau_J),
            end_effector_pose_c: franka::utils::array_to_isometry(&state.O_T_EE_c),
            end_effector_pose_d: franka::utils::array_to_isometry(&state.O_T_EE_d),
            end_effector_pose: franka::utils::array_to_isometry(&state.O_T_EE),
            end_effector_velocity_d: state.O_dP_EE_d.into(),
            jacobian: Matrix6x7::zeros(),
            mass_matrix: Matrix7::zeros(),
            coriolis: Vector7::zeros(),
            gravity: Vector7::zeros(),
            control_command_success_rate: state.control_command_success_rate,
        }
    }
}

impl From<franka::GripperState> for GripperState {
    fn from(state: franka::GripperState) -> Self {
        GripperState {
            gripper_width: state.width,
            closed: state.is_grasped,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::types::{Torques, Vector7};

    #[test]
    pub fn convert_torques() {
        let vec = Vector7::from_column_slice(&[0., 1., 2., 3., 4., 5., 6.]);
        let torque = Torques::from(vec.clone());
        let franka_torque: franka::Torques = torque.into();
        let orig_torque: Torques = franka_torque.into();
        let orig_vec: Vector7 = orig_torque.torques;
        for i in 0..7 {
            assert_eq!(orig_vec[i], vec[i]);
        }
    }
}
