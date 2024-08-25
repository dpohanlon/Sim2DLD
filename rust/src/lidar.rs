mod random_geometry;

use godot::prelude::*;

use godot::classes::{
    AStar2D, CollisionPolygon2D, Geometry2D, INode2D, Line2D, Node2D, Polygon2D, RayCast2D,
    StaticBody2D,
};

use ndarray::{arr2, Array2};

use serde::ser::{Serialize, Serializer};

use serde_json::to_writer;
use std::fs::File;
use std::io::BufWriter;

use std::env;

use crate::lidar::random_geometry::RandomGeometryGenerator;

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
    arena: Gd<Polygon2D>,
    rays: Vec<Gd<RayCast2D>>,
    lines: Vec<Gd<Line2D>>,
    path: Vec<Vector2>,
    path_idx: usize,
}

#[godot_api]
impl INode2D for Lidar {
    fn init(base: Base<Node2D>) -> Self {
        godot_print!("Hello, world!");

        let mut polygon = Polygon2D::new_alloc();

        let size_x = 1024.;
        let size_y = 1024.;

        // Define the vertices for the square
        // let vertices = vec![
        //     Vector2::new(0.0, 0.0),
        //     Vector2::new(size_x, 0.0),
        //     Vector2::new(size_x, size_y),
        //     Vector2::new(0.0, size_y),
        // ];
        let vertices = vec![
            Vector2::new(0., 0.),
            Vector2::new(size_x, 0.),
            Vector2::new(size_x, size_y),
            Vector2::new(0., size_y),
        ];
        polygon.set_polygon(vertices.into());

        Self {
            base,
            arena: polygon,
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

        // Register the geometry as a child of the Lidar node
        // Children are 'ready' before their parent
        self.base_mut().add_child(geom.clone());

        let poly_len = geom.bind().polygons.len();

        let mut geometry2d = Geometry2D::singleton();

        godot_print!("Arena polygon {}", self.arena.get_polygon());
        // godot_print!("Query polygon {}", geom.bind().polygons[0].get_polygon());

        // for poly in poly_diffs.iter_shared() {
        //     let mut polygon = Polygon2D::new_alloc();
        //     polygon.set_polygon(poly);
        //     godot_print!("Diff polygon {}", polygon.get_polygon());
        //     self.base_mut().add_child(polygon);
        // }

        godot_print!("I am LIDAR and I have {} polygons", poly_len);

        // set connectedness -> loop over x, set left and right to connected
        // loop over y, set top and bottom to connected

        let mut astar = AStar2D::new_gd();

        for i in 0..100 {
            for j in 0..100 {
                let x = i as f32 * (1024. / 100.);
                let y = j as f32 * (1024. / 100.);

                astar.add_point(i + 100 * j, Vector2::new(x, y));
            }
        }

        for i in 0..100 {
            let x = i as f32 * (1024. / 100.);

            for j in 0..100 {
                let y = j as f32 * (1024. / 100.);

                let mut polygon = Polygon2D::new_alloc();
                let vertices = vec![
                    Vector2::new(x, y),
                    Vector2::new(x - 5.0, y),
                    Vector2::new(x - 5.0, y + 5.0),
                    Vector2::new(x, y + 5.0),
                ];
                polygon.set_polygon(vertices.into());

                let mut occluded = false;
                for g in geom.bind().polygons.iter() {
                    let poly = g.get_polygon();
                    occluded = geometry2d.is_point_in_polygon(Vector2::new(x, y), poly);
                    if occluded {
                        break;
                    }
                }

                if occluded {
                    let color = Color::from_rgba(1.0, 0.0, 0.0, 1.0);
                    polygon.set_color(color);
                    self.base_mut().add_child(polygon);
                } else {
                    if i > 0 {
                        astar.connect_points(i + 100 * j, (i - 1) + 100 * j);
                    }
                    if j > 0 {
                        astar.connect_points(i + 100 * j, i + 100 * (j - 1));
                    }
                }

                // self.base_mut().add_child(polygon);
            }
        }

        let path = astar.get_point_path(0, 6290);
        self.path = path.to_vec();
        godot_print!("Path length: {}", path.to_vec().len());

        for point in path.to_vec().iter() {
            godot_print!("{}, {}", point.x, point.y);
            let mut polygon = Polygon2D::new_alloc();
            let vertices = vec![
                Vector2::new(point.x, point.y),
                Vector2::new(point.x - 5.0, point.y),
                Vector2::new(point.x - 5.0, point.y + 5.0),
                Vector2::new(point.x, point.y + 5.0),
            ];
            polygon.set_polygon(vertices.into());
            let color = Color::from_rgba(0.0, 1.0, 0.0, 1.0);
            polygon.set_color(color);
            self.base_mut().add_child(polygon);
        }

        let mut static_body = StaticBody2D::new_alloc();

        for poly in geom.bind().polygons.iter() {
            let mut polygon = CollisionPolygon2D::new_alloc();
            polygon.set_polygon(poly.get_polygon());
            static_body.add_child(polygon);
        }

        self.base_mut().add_child(static_body);

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
            // godot_print!("Direction: {}", direction);
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

        let v = arr2(&[[1., 2.], [4., 5.]]);

        let serializable_array = SerializableArray2 { array: v.clone() };

        let _ = write_to_json("test.json", &serializable_array).unwrap();
    }

    fn process(&mut self, _delta: f64) {
        // Keep an array of all rays (and Line2Ds for rendering) and update these every frame

        if (self.path.len() == 0) || (self.path_idx >= self.path.len() - 1) {
            self.base_mut().get_tree().unwrap().reload_current_scene();

            return;
        }

        let loc = self.path[self.path_idx];

        for (i, ray) in self.rays.clone().iter_mut().enumerate() {
            // Update the ray origin, but this might not take effect on the collisions until next frame?

            ray.set_position(loc);

            let point = if ray.is_colliding() {
                ray.get_collision_point()
            } else {
                ray.get_target_position()
            };

            // godot_print!("Ray collision: {}", point);

            // let mut polygon = Polygon2D::new_alloc();
            // let vertices = vec![
            //     Vector2::new(point.x, point.y),
            //     Vector2::new(point.x - 5.0, point.y),
            //     Vector2::new(point.x - 5.0, point.y + 5.0),
            //     Vector2::new(point.x, point.y + 5.0),
            // ];
            // polygon.set_polygon(vertices.into());
            // let color = Color::from_rgba(0.0, 0.0, 1.0, 1.0);
            // polygon.set_color(color);
            // // Slow
            // self.base_mut().add_child(polygon);

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

        self.path_idx += 1;
    }
}

// A function to write a serializable object to a JSON file
fn write_to_json<T: Serialize>(filename: &str, data: &T) -> std::io::Result<()> {
    let file = File::create(filename)?; // Open a file in write mode
    let writer = BufWriter::new(file); // Create a buffered writer for efficient writing
    to_writer(writer, data)?; // Serialize the data to JSON and write it to the file
    Ok(())
}

impl Lidar {
    fn generate_geometry(&mut self) -> Gd<RandomGeometryGenerator> {
        godot_print!("Generating geometry!");

        let geom = random_geometry::RandomGeometryGenerator::new();
        geom
    }
}
