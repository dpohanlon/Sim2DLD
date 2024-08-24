mod random_geometry;

use godot::prelude::*;

use godot::classes::{Geometry2D, INode2D, Node2D, Polygon2D};

use crate::lidar::random_geometry::RandomGeometryGenerator;

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
    arena: Gd<Polygon2D>,
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
        }
    }

    fn ready(&mut self) {
        let geom = self.generate_geometry();

        // Register the geometry as a child of the Lidar node
        // Children are 'ready' before their parent
        self.base_mut().add_child(geom.clone());

        let poly_len = geom.bind().polygons.len();

        let mut geometry2d = Geometry2D::singleton();

        let poly_diffs = geometry2d.exclude_polygons(
            self.arena.get_polygon(),
            geom.bind().polygons[0].get_polygon(),
        );

        godot_print!("Arena polygon {}", self.arena.get_polygon());
        godot_print!("Query polygon {}", geom.bind().polygons[0].get_polygon());

        // for poly in poly_diffs.iter_shared() {
        //     let mut polygon = Polygon2D::new_alloc();
        //     polygon.set_polygon(poly);
        //     godot_print!("Diff polygon {}", polygon.get_polygon());
        //     self.base_mut().add_child(polygon);
        // }

        let mut polygon = Polygon2D::new_alloc();
        polygon.set_polygon(poly_diffs.at(0).clone());
        godot_print!("Diff polygon {}", polygon.get_polygon());
        // self.base_mut().add_child(polygon);

        // let arena_b = self.arena.clone();
        // self.base_mut().add_child(arena_b);

        godot_print!("I am LIDAR and I have {} polygons", poly_len);

        for i in 0..100 {
            for j in 0..100 {
                let x = i as f32 * (1024. / 100.);
                let y = j as f32 * (1024. / 100.);
                let mut polygon = Polygon2D::new_alloc();
                let vertices = vec![
                    Vector2::new(x, y),
                    Vector2::new(x + 2.0, y),
                    Vector2::new(x + 2.0, y + 2.0),
                    Vector2::new(x, y + 2.0),
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
                }

                self.base_mut().add_child(polygon);
            }
        }
    }
}

impl Lidar {
    fn generate_geometry(&mut self) -> Gd<RandomGeometryGenerator> {
        godot_print!("Generating geometry!");

        let geom = random_geometry::RandomGeometryGenerator::new();
        geom
    }
}
