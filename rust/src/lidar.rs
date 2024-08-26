mod random_geometry;

use crate::lidar::random_geometry::RandomGeometryGenerator;
use godot::classes::{
    AStar2D, CollisionPolygon2D, Geometry2D, INode2D, Line2D, Node2D, Polygon2D, RayCast2D,
    StaticBody2D,
};
use godot::prelude::*;
use ndarray::{arr2, Array2};
use serde::ser::{Serialize, Serializer};
use serde_json::to_writer;
use std::env;
use std::fs::File;
use std::io::BufWriter;

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

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
    _arena: Gd<Polygon2D>,
    rays: Vec<Gd<RayCast2D>>,
    lines: Vec<Gd<Line2D>>,
    path: Vec<Vector2>,
    path_idx: usize,
}

#[godot_api]
impl INode2D for Lidar {
    fn init(base: Base<Node2D>) -> Self {
        godot_print!("Hello, world!");

        let polygon = Self::create_arena_polygon(1024., 1024.);

        Self {
            base,
            _arena: polygon,
            rays: Vec::<Gd<RayCast2D>>::new(),
            lines: Vec::<Gd<Line2D>>::new(),
            path: Vec::<Vector2>::new(),
            path_idx: 0,
        }
    }

    fn ready(&mut self) {
        let args: Vec<String> = env::args().collect();
        godot_print!("Command-line arguments: {:?}", args);

        let geom = self.generate_geometry();

        self.base_mut().add_child(geom.clone());
        let poly_len = geom.bind().polygons.len();
        godot_print!("I am LIDAR and I have {} polygons", poly_len);

        self.create_astar_grid(&geom);

        let path = self.calculate_path(&geom);
        self.path = path;
        godot_print!("Path length: {}", self.path.len());

        let points = self.path.clone();
        for point in points.iter() {
            self.draw_point(&point, Color::from_rgba(0.0, 1.0, 0.0, 1.0));
        }

        let static_body = self.create_static_body(&geom);
        self.base_mut().add_child(static_body);

        self.initialize_rays_and_lines();

        let v = arr2(&[[1., 2.], [4., 5.]]);
        let serializable_array = SerializableArray2 { array: v.clone() };
        let _ = write_to_json("test.json", &serializable_array).unwrap();
    }

    fn process(&mut self, _delta: f64) {
        if self.path.is_empty() || self.path_idx >= self.path.len() - 1 {
            self.base_mut().get_tree().unwrap().reload_current_scene();
            return;
        }

        let loc = self.path[self.path_idx];
        self.update_rays_and_lines(loc);
        self.path_idx += 1;
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

    fn create_astar_grid(&mut self, geom: &Gd<RandomGeometryGenerator>) {
        let mut astar = AStar2D::new_gd();
        let mut geometry2d = Geometry2D::singleton();

        for i in 0..100 {
            for j in 0..100 {
                let x = i as f32 * (1024. / 100.);
                let y = j as f32 * (1024. / 100.);
                astar.add_point(i + 100 * j, Vector2::new(x, y));

                if self.is_point_occluded(x, y, geom, &mut geometry2d) {
                    self.draw_point(&Vector2::new(x, y), Color::from_rgba(1.0, 0.0, 0.0, 1.0));
                } else {
                    if i > 0 {
                        astar.connect_points(i + 100 * j, (i - 1) + 100 * j);
                    }
                    if j > 0 {
                        astar.connect_points(i + 100 * j, i + 100 * (j - 1));
                    }
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

    fn update_rays_and_lines(&mut self, loc: Vector2) {
        for (i, ray) in self.rays.clone().iter_mut().enumerate() {
            ray.set_position(loc);

            let point = if ray.is_colliding() {
                ray.get_collision_point()
            } else {
                ray.get_target_position()
            };

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
    }

    fn generate_geometry(&mut self) -> Gd<RandomGeometryGenerator> {
        godot_print!("Generating geometry!");
        random_geometry::RandomGeometryGenerator::new()
    }
}

// A function to write a serializable object to a JSON file
fn write_to_json<T: Serialize>(filename: &str, data: &T) -> std::io::Result<()> {
    let file = File::create(filename)?; // Open a file in write mode
    let writer = BufWriter::new(file); // Create a buffered writer for efficient writing
    to_writer(writer, data)?; // Serialize the data to JSON and write it to the file
    Ok(())
}
