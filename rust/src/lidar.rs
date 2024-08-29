mod random_geometry;

use std::sync::Mutex;

use crate::lidar::random_geometry::RandomGeometryGenerator;
use godot::classes::{
    AStar2D, CollisionPolygon2D, Geometry2D, INode2D, Line2D, Node2D, Polygon2D, RayCast2D,
    RenderingServer, StaticBody2D,
};
use godot::prelude::*;
use ndarray::Array2;
use serde::ser::{Serialize, Serializer};
use serde_json::to_writer;
use std::env;
use std::fs::File;
use std::io::BufWriter;

use std::{thread, time};

// Wrap Array2 in a new struct
struct SerializableArray2<T> {
    array: Array2<T>,
}

// Implement Serialize for the wrapper struct
impl<T: Serialize + std::clone::Clone> Serialize for SerializableArray2<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Convert Array2 into a serializable Vec<Vec<T>>
        let vec_of_vecs: Vec<Vec<T>> = self
            .array
            .rows()
            .into_iter()
            .map(|row| row.to_vec())
            .collect();

        // Serialize the Vec<Vec<T>>
        vec_of_vecs.serialize(serializer)
    }
}

// A function to write a serializable object to a JSON file
fn write_to_json<T: Serialize>(filename: &str, data: &T) -> std::io::Result<()> {
    let file = File::create(filename)?; // Open a file in write mode
    let writer = BufWriter::new(file); // Create a buffered writer for efficient writing
    to_writer(writer, data)?; // Serialize the data to JSON and write it to the file
    Ok(())
}

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
    _arena: Gd<Polygon2D>,
    rays: Vec<Gd<RayCast2D>>,
    lines: Vec<Gd<Line2D>>,
    path: Vec<Vector2>,
    path_idx: usize,
    angle: f32,
    returns: Vec<Array2<f64>>,
}

// Static variable declaration outside the struct and impl block
static LIDAR_COUNT: Mutex<u32> = Mutex::new(0); // Static mutable variable to track instances

#[godot_api]
impl INode2D for Lidar {
    fn init(base: Base<Node2D>) -> Self {
        // godot_print!("Hello, world!");

        let polygon = Self::create_arena_polygon(1024., 1024.);

        Self {
            base,
            _arena: polygon,
            rays: Vec::<Gd<RayCast2D>>::new(),
            lines: Vec::<Gd<Line2D>>::new(),
            path: Vec::<Vector2>::new(),
            path_idx: 0,
            angle: 0.0,
            returns: Vec::<Array2<f64>>::new(),
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
        godot_print!("Command-line arguments: {:?}", args);

        let geom = self.generate_geometry();

        self.base_mut().add_child(geom.clone());
        let poly_len = geom.bind().polygons.len();
        // godot_print!("I am LIDAR and I have {} polygons", poly_len);

        self.create_astar_grid();

        let path = self.calculate_path(&geom);
        self.path = path;
        // godot_print!("Path length: {}", self.path.len());

        let points = self.path.clone();
        for point in points.iter() {
            self.draw_point(&point, Color::from_rgba(0.0, 1.0, 0.0, 1.0));
        }

        let static_body = self.create_static_body(&geom);
        self.base_mut().add_child(static_body);

        // TODO: Align heading with the first segment of the path

        self.initialize_rays_and_lines();
    }

    fn process(&mut self, _delta: f64) {
        if self.path.is_empty() || self.path_idx >= self.path.len() - 1 {
            if !self.path.is_empty() {
                let serializable_arrays: Vec<SerializableArray2<f64>> = self
                    .returns
                    .clone()
                    .into_iter()
                    .map(|array| SerializableArray2 { array })
                    .collect();

                let mut count = LIDAR_COUNT.lock().unwrap(); // Lock the mutex before modifying

                let filename = format!("test_{}.json", count);

                let _ = write_to_json(&filename, &serializable_arrays).unwrap();

                *count += 1;
            }

            self.base_mut().get_tree().unwrap().reload_current_scene();
            return;
        }

        let loc = self.path[self.path_idx];
        let prev_loc = if self.path_idx > 0 {
            self.path[self.path_idx - 1]
        } else {
            loc
        };

        self.update_rays_and_lines(loc, prev_loc);
        self.path_idx += 1;

        // thread::sleep(time::Duration::from_secs(1));
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
        astar.get_point_path(0, 6290).to_vec()
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
        for poly in geom.bind().polygons.iter() {
            let mut polygon = CollisionPolygon2D::new_alloc();
            polygon.set_polygon(poly.get_polygon());
            static_body.add_child(polygon);
        }
        static_body
    }

    fn initialize_rays_and_lines(&mut self) {
        let n_rays = 360;
        let d_max = 10000.0;
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

            let mut line = Line2D::new_alloc();
            line.set_width(3.0);
            line.add_point(ray.get_position());
            line.add_point(ray.get_position());
            self.base_mut().add_child(line.clone());
            self.lines.push(line.clone());

            self.base_mut().add_child(ray.clone());
            self.rays.push(ray.clone());
        }
    }

    fn update_rays_and_lines(&mut self, loc: Vector2, prev_loc: Vector2) {
        // Angle relative to heading
        let angle = self.get_path_angle(prev_loc, loc);
        let d_angle = angle - self.angle;

        // Update heading
        self.angle = angle;

        // godot_print!(
        //     "Heading: {}, Angle: {}",
        //     self.angle.to_degrees(),
        //     angle.to_degrees()
        // );

        let mut ray_returns: Array2<f64> = Array2::zeros((360, 2));

        // One step behind when rotating?
        for (i, ray) in self.rays.clone().iter_mut().enumerate() {
            // Adjust for slew speed

            let ray_angle = self.get_path_angle(ray.get_position(), ray.get_target_position());
            let d_angle_ray = ray_angle - self.angle;

            // godot_print!(
            //     "Ray original: {}, dAngle: {}",
            //     ray_angle.to_degrees(),
            //     d_angle_ray.to_degrees()
            // );

            // Rotate the target position by relative difference in heading
            let mut target = ray.get_target_position();
            target = Vector2::new(
                target.x * d_angle.cos() - target.y * d_angle.sin(),
                target.x * d_angle.sin() + target.y * d_angle.cos(),
            );
            ray.set_target_position(target);

            ray.set_position(loc);

            let point = if ray.is_colliding() {
                ray.get_collision_point()
            } else {
                ray.get_target_position()
            };

            let dx = point.x - ray.get_position().x;
            let dy = point.y - ray.get_position().y;
            let distance = (dx * dx + dy * dy).sqrt();

            ray_returns[[i, 0]] = distance as f64;
            ray_returns[[i, 1]] = ray_angle as f64;

            // Slowwww
            // self.draw_point(&point, Color::from_rgba(0.0, 0.0, 1.0, 1.0));

            let mut line = self.lines[i].clone();
            line.clear_points();
            line.add_point(ray.get_position());
            line.add_point(point);

            if ray.is_colliding() {
                line.set_default_color(Color::from_rgba(1.0, 0.0, 0.0, 1.0));
            } else {
                line.set_default_color(Color::from_rgba(0.0, 0.0, 1.0, 1.0));
            }
        }

        self.returns.push(ray_returns);
    }

    fn get_path_angle(&self, loc: Vector2, next_loc: Vector2) -> f32 {
        let diff = next_loc - loc;
        diff.angle()
    }

    fn generate_geometry(&mut self) -> Gd<RandomGeometryGenerator> {
        // godot_print!("Generating geometry!");
        random_geometry::RandomGeometryGenerator::new()
    }
}
