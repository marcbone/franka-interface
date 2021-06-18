// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//! Contains a struct for creating images inside the simulation

use rubullet::image::RgbaImage;
use rubullet::nalgebra::Matrix4;
use rubullet::{CameraImageOptions, PhysicsClient};
use std::cell::RefCell;
use std::rc::Rc;

/// Camera for the Simulation. This struct should make it easier to get camera images from the simulation.
pub struct SimCamera {
    view_matrix: Matrix4<f32>,
    projection_matrix: Matrix4<f32>,
    width: usize,
    height: usize,
}

impl SimCamera {
    /// creates a new SimCamera
    /// # Arguments
    /// * `width` - width of the image in pixel
    /// * `height` - height of the image in pixel
    /// * `view_matrix` - can be calculated using [`PhysicsClient::compute_view_matrix`](`rubullet::PhysicsClient::compute_view_matrix`)
    /// or [`PhysicsClient::compute_view_matrix_from_yaw_pitch_roll`](`rubullet::PhysicsClient::compute_view_matrix_from_yaw_pitch_roll`)
    /// * `projection_matrix` - can be calculated using [`PhysicsClient::compute_projection_matrix`](`rubullet::PhysicsClient::compute_projection_matrix`)
    /// or [`PhysicsClient::compute_projection_matrix_fov`](`rubullet::PhysicsClient::compute_projection_matrix_fov`)
    pub fn new(
        width: usize,
        height: usize,
        view_matrix: Matrix4<f32>,
        projection_matrix: Matrix4<f32>,
    ) -> SimCamera {
        SimCamera {
            view_matrix,
            projection_matrix,
            width,
            height,
        }
    }
    /// returns an image from the simulation. Note that taking pictures has a significant impact on
    /// the performance of the simulation.
    pub fn get_image(&self, client: Rc<RefCell<PhysicsClient>>) -> RgbaImage {
        client
            .borrow_mut()
            .get_camera_image(
                self.width,
                self.height,
                CameraImageOptions {
                    view_matrix: Some(self.view_matrix),
                    projection_matrix: Some(self.projection_matrix),
                    ..Default::default()
                },
            )
            .unwrap()
            .rgba
    }
}
