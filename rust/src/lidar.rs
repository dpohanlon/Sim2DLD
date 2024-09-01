// Run with .command file

mod argument_parser;
mod lidar_state;
mod random_geometry;
mod serializer;

use std::collections::HashMap;
use std::sync::Mutex;

use crate::lidar::random_geometry::RandomGeometryGenerator;
use crate::lidar::serializer::write_to_json;
use crate::lidar::serializer::SerializableArray2;
use godot::classes::{
    AStar2D, CollisionPolygon2D, Geometry2D, INode2D, Label, Line2D, Node2D, Polygon2D, RayCast2D,
    RenderingServer, StaticBody2D,
};
use godot::prelude::*;
use ndarray::Array2;
use std::env;

use std::{thread, time};

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
    _arena: Gd<Polygon2D>,
    parsed_args: HashMap<String, String>,
    out_dir: String,
    n_iterations: u32,
    state: lidar_state::LidarState, // Replace individual state variables
}

// Static variable declaration outside the struct and impl block
static LIDAR_COUNT: Mutex<u32> = Mutex::new(0); // Static mutable variable to track instances

#[godot_api]
impl INode2D for Lidar {
    fn init(base: Base<Node2D>) -> Self {
        let polygon = Self::create_arena_polygon(1024., 1024.);

        Self {
            base,
            _arena: polygon,
            parsed_args: HashMap::new(),
            out_dir: String::from("lidar_out"),
            n_iterations: 10,
            state: lidar_state::LidarState::new(), // Initialize LidarState
        }
    }

    fn ready(&mut self) {
        godot_print!("Count {}", LIDAR_COUNT.lock().unwrap());

        RenderingServer::singleton().set_default_clear_color(Color::from_rgba(
            255. / 255.,
            218. / 255.,
            118. / 255.,
            1.0,
        ));

        let args: Vec<String> = env::args().collect();

        self.parsed_args = argument_parser::parse_args(args);

        godot_print!("Command-line arguments: {:?}", self.parsed_args);

        if let Some(label) = self.parsed_args.get("label") {
            self.add_center_label(&label.clone(), 1024., 1024.);
        }

        let geom = self.generate_geometry();

        self.base_mut().add_child(geom.clone());
        let poly_len = geom.bind().polygons.len();
        godot_print!("I am LIDAR and I have {} polygons", poly_len);

        self.create_astar_grid();

        let path = self.calculate_path(&geom);
        self.state.path = path;
        godot_print!("Path length: {}", self.state.path.len());
        godot_print!("Path (0): {}", self.state.path[0]);

        // Copy path into array2 for serialization
        let path_array = Array2::from_shape_vec(
            (self.state.path.len(), 2),
            self.state
                .path
                .iter()
                .flat_map(|v| vec![v.x, v.y])
                .collect(),
        )
        .unwrap();

        // Serialize the path to a JSON file
        let serializable_path = serializer::SerializableArray2 { array: path_array };

        let count = LIDAR_COUNT.lock().unwrap(); // Lock the mutex before modifying
        let filename = format!("{}/lidar_path_{}.json", self.out_dir, count);
        let _ = serializer::write_to_json(&filename, &serializable_path);

        let points = self.state.path.clone();
        for point in points.iter() {
            self.draw_point(
                &point,
                Color::from_rgba(255. / 255., 78. / 255., 136. / 255., 1.0),
            );
        }

        let static_body = self.create_static_body(&geom);
        self.base_mut().add_child(static_body);

        // TODO: Align heading with the first segment of the path

        self.initialize_rays_and_lines();
    }

    fn process(&mut self, delta: f64) {
        if self.state.slewing {
            godot_print!(
                "Slewing, target {}, angle {}",
                self.state.target_angle,
                self.state.angle
            );
            // If slewing, calculate the rotation amount based on the slew rate and time delta
            let rotation_speed = self.state.slew_rate * delta as f32; // degrees per frame based on time delta
            let angle_diff = self.state.target_angle - self.state.angle;
            let rotation_step = angle_diff.signum() * rotation_speed.min(angle_diff.abs());

            // Update the Lidar's angle
            self.state.angle += rotation_step;

            // Check if we have reached the target angle
            if (self.state.target_angle - self.state.angle).abs() < 1E-4 {
                self.state.angle = self.state.target_angle; // Snap to target angle
                self.state.slewing = false; // Finished slewing
            }

            // Update rays' positions and orientations
            self.update_rays_rotation();
        } else {
            // If not slewing, handle the movement along the path
            if self.state.path.is_empty() || self.state.path_idx >= self.state.path.len() - 1 {
                if !self.state.path.is_empty() {
                    let serializable_arrays: Vec<serializer::SerializableArray2<f64>> = self
                        .state
                        .returns
                        .clone()
                        .into_iter()
                        .map(|array| SerializableArray2 { array })
                        .collect();

                    let mut count = LIDAR_COUNT.lock().unwrap(); // Lock the mutex before modifying

                    let filename = format!("{}/lidar_returns_{}.json", self.out_dir, count);

                    let _ = write_to_json(&filename, &serializable_arrays).unwrap();

                    *count += 1;

                    if *count >= self.n_iterations {
                        godot_print!("Finished {} iterations", self.n_iterations);
                        self.base_mut().get_tree().unwrap().quit();
                    }
                }

                self.base_mut().get_tree().unwrap().reload_current_scene();
                return;
            }

            let loc = self.state.path[self.state.path_idx];
            let prev_loc = if self.state.path_idx > 0 {
                self.state.path[self.state.path_idx - 1]
            } else {
                loc
            };

            let desired_angle = self.get_path_angle(prev_loc, loc);

            // Check if the Lidar needs to rotate to face the new direction
            if (self.state.angle - desired_angle).abs() > 1E-4 {
                // Start slewing to the desired angle
                self.state.slewing = true;
                self.state.target_angle = desired_angle;
            } else {
                // Move to the next point in the path
                self.update_rays_and_lines(loc, prev_loc);
                self.state.path_idx += 1;
            }

            // Optional: introduce a delay for testing
            // thread::sleep(time::Duration::from_secs(1));
        }
    }
}

// Additional methods for Lidar
impl Lidar {
    fn create_arena_polygon(size_x: f32, size_y: f32) -> Gd<Polygon2D> {
        let mut polygon = Polygon2D::new_alloc();
        let vertices = vec![
            Vector2::new(0., 0.),
            Vector2::new(size_x, 0.),
            Vector2::new(size_x, size_y),
            Vector2::new(0., size_y),
        ];
        polygon.set_polygon(vertices.into());
        polygon
    }

    fn add_center_label(&mut self, text: &str, arena_width: f32, arena_height: f32) {
        let mut label = Label::new_alloc();
        label.set_text(text.into());

        // Center the label within the arena
        label.set_anchor(Side::LEFT, 0.5); // Center horizontally
        label.set_anchor(Side::TOP, 0.5); // Center vertically

        // Set the label's position to the center of the arena
        let position = Vector2::new(arena_width / 2.0, arena_height / 2.0);
        label.set_position(position);

        // Add the label as a child to the current node
        self.base_mut().add_child(label);
    }

    fn create_astar_grid(&mut self) {
        let mut astar = AStar2D::new_gd();

        for i in 0..100 {
            for j in 0..100 {
                let x = i as f32 * (1024. / 100.);
                let y = j as f32 * (1024. / 100.);
                astar.add_point(i + 100 * j, Vector2::new(x, y));

                if i > 0 {
                    astar.connect_points(i + 100 * j, (i - 1) + 100 * j);
                }
                if j > 0 {
                    astar.connect_points(i + 100 * j, i + 100 * (j - 1));
                }
            }
        }
    }

    fn is_point_occluded(
        &self,
        x: f32,
        y: f32,
        geom: &Gd<RandomGeometryGenerator>,
        geometry2d: &mut Geometry2D,
    ) -> bool {
        for g in geom.bind().polygons.iter() {
            let poly = g.get_polygon();
            if geometry2d.is_point_in_polygon(Vector2::new(x, y), poly) {
                return true;
            }
        }
        false
    }

    fn calculate_path(&self, geom: &Gd<RandomGeometryGenerator>) -> Vec<Vector2> {
        let mut astar = AStar2D::new_gd();
        let mut geometry2d = Geometry2D::singleton();

        // Create a 100x100 grid of points
        for i in 0..100 {
            for j in 0..100 {
                let x = i as f32 * (1024. / 100.);
                let y = j as f32 * (1024. / 100.);
                astar.add_point(i + 100 * j, Vector2::new(x, y));
            }
        }

        // Connect points in the grid if they are not occluded by any geometry
        for i in 0..100 {
            for j in 0..100 {
                let index = i + 100 * j;
                let x = i as f32 * (1024. / 100.);
                let y = j as f32 * (1024. / 100.);

                if !self.is_point_occluded(x, y, geom, &mut geometry2d) {
                    // Connect to the left neighbor
                    if i > 0 {
                        let left_index = (i - 1) + 100 * j;
                        astar.connect_points(index, left_index);
                    }
                    // Connect to the top neighbor
                    if j > 0 {
                        let top_index = i + 100 * (j - 1);
                        astar.connect_points(index, top_index);
                    }
                }
            }
        }

        // Calculate and return the path from point 0 to point 6290 (end point)
        astar.get_point_path(702, 6290).to_vec()
    }

    fn draw_point(&mut self, point: &Vector2, color: Color) {
        let mut polygon = Polygon2D::new_alloc();
        let vertices = vec![
            *point,
            Vector2::new(point.x - 5.0, point.y),
            Vector2::new(point.x - 5.0, point.y + 5.0),
            Vector2::new(point.x, point.y + 5.0),
        ];
        polygon.set_polygon(vertices.into());
        polygon.set_color(color);
        self.base_mut().add_child(polygon);
    }

    fn create_static_body(&self, geom: &Gd<RandomGeometryGenerator>) -> Gd<StaticBody2D> {
        let mut static_body = StaticBody2D::new_alloc();
        godot_print!("Geoms: {}", geom.bind().polygons.len());
        for poly in geom.bind().polygons.iter() {
            godot_print!("Adding polygon to static body, {}", poly);
            let mut polygon = CollisionPolygon2D::new_alloc();
            polygon.set_polygon(poly.get_polygon());
            godot_print!("pol, {}", poly.get_polygon());
            static_body.add_child(polygon);
        }
        static_body
    }

    fn initialize_rays_and_lines(&mut self) {
        let n_rays = 360;
        let d_max = 100000.0;
        let angles = (0..n_rays).map(|i| i as f32 * 360.0 / n_rays as f32);

        let directions: Vec<Vector2> = angles
            .map(|angle| {
                Vector2::new(
                    d_max * angle.to_radians().cos(),
                    d_max * angle.to_radians().sin(),
                )
            })
            .collect();

        for direction in directions.iter() {
            let mut ray: Gd<RayCast2D> = RayCast2D::new_alloc();
            ray.set_position(Vector2::new(100.0, 100.0));
            ray.set_target_position(*direction);
            ray.set_collision_mask_value(1, true);
            ray.set_enabled(true);

            if !self.parsed_args.contains_key("suppress_lines") {
                let mut line = Line2D::new_alloc();
                line.set_width(3.0);
                line.add_point(ray.get_position());
                line.add_point(ray.get_position());
                self.base_mut().add_child(line.clone());
                self.state.lines.push(line.clone());
            }

            self.base_mut().add_child(ray.clone());
            self.state.rays.push(ray.clone());
        }
    }

    fn update_rays_and_lines(&mut self, loc: Vector2, prev_loc: Vector2) {
        // Calculate change in angle
        let angle = self.get_path_angle(prev_loc, loc);
        let d_angle = angle - self.state.angle;
        self.state.angle = angle; // Update Lidar heading angle

        // Ensure rays are sufficiently long and have correct target positions
        let mut ray_returns: Array2<f64> = Array2::zeros((360, 2));

        for (i, ray) in self.state.rays.clone().iter_mut().enumerate() {
            // Get the current position of the ray
            let ray_pos = ray.get_position();
            let target_pos = ray.get_target_position();

            // Compute vector from ray position to its target
            let offset = target_pos - ray_pos;

            // Apply rotation matrix to adjust for the new heading
            let rotated_offset = Vector2::new(
                offset.x * d_angle.cos() - offset.y * d_angle.sin(),
                offset.x * d_angle.sin() + offset.y * d_angle.cos(),
            );

            // Update the ray target position relative to its base
            let new_target_position = ray_pos + rotated_offset;
            ray.set_target_position(new_target_position);
            ray.set_position(loc); // Ensure ray position moves with the Lidar

            // Check for collision
            let collision_point = if ray.is_colliding() {
                ray.get_collision_point()
            } else {
                ray.get_target_position()
            };

            // Update ray return data with distance and angle
            let distance = (collision_point - ray.get_position()).length();
            let ray_angle = self.get_path_angle(ray.get_position(), collision_point);
            ray_returns[[i, 0]] = distance as f64;
            ray_returns[[i, 1]] = ray_angle as f64;

            if !self.parsed_args.contains_key("suppress_lines") {
                // Update visual line representation
                let mut line = self.state.lines[i].clone();
                line.clear_points();
                line.add_point(ray.get_position());
                line.add_point(collision_point);
                line.set_default_color(if ray.is_colliding() {
                    Color::from_rgba(255. / 255., 140. / 255., 158. / 255., 1.0)
                // Red for collision
                } else {
                    Color::from_rgba(0.0, 1.0, 0.0, 1.0) // Green otherwise
                });
            }
        }

        self.state.returns.push(ray_returns);
    }

    fn get_path_angle(&self, loc: Vector2, next_loc: Vector2) -> f32 {
        let diff = next_loc - loc;
        diff.angle()
    }

    fn update_rays_rotation(&mut self) {
        let loc = self.state.path[self.state.path_idx]; // Get Lidar's global position
        let rotation_radians = self.state.angle.to_radians();

        for ray in self.state.rays.iter_mut() {
            let ray_position = ray.get_position();
            let offset = ray.get_target_position() - ray_position;

            // Rotate each ray's target position
            let rotated_offset = Vector2::new(
                offset.x * rotation_radians.cos() - offset.y * rotation_radians.sin(),
                offset.x * rotation_radians.sin() + offset.y * rotation_radians.cos(),
            );

            ray.set_target_position(ray_position + rotated_offset);
            ray.set_position(loc);
        }
    }

    fn generate_geometry(&mut self) -> Gd<RandomGeometryGenerator> {
        // godot_print!("Generating geometry!");
        random_geometry::RandomGeometryGenerator::new()
    }
}
