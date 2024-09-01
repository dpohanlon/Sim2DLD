use godot::classes::{Line2D, RayCast2D};
use godot::prelude::*;
use ndarray::Array2;

/// A struct to encapsulate the state variables for the Lidar.
pub struct LidarState {
    pub rays: Vec<Gd<RayCast2D>>,
    pub lines: Vec<Gd<Line2D>>,
    pub path: Vec<Vector2>,
    pub path_idx: usize,
    pub angle: f32,
    pub target_angle: f32,
    pub slewing: bool,
    pub slew_rate: f32,
    pub returns: Vec<Array2<f64>>,
}

impl LidarState {
    /// Creates a new `LidarState` with default values.
    pub fn new() -> Self {
        Self {
            rays: Vec::<Gd<RayCast2D>>::new(),
            lines: Vec::<Gd<Line2D>>::new(),
            path: Vec::<Vector2>::new(),
            path_idx: 0,
            angle: 0.0,
            target_angle: 0.0,
            slewing: false,
            slew_rate: 30.0, // degrees per second
            returns: Vec::<Array2<f64>>::new(),
        }
    }
}
