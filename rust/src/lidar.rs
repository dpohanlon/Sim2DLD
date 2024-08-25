mod random_geometry;

use godot::prelude::*;

use godot::classes::{
    AStar2D, CollisionPolygon2D, Geometry2D, INode2D, Node2D, Polygon2D, RayCast2D, StaticBody2D,
};

use crate::lidar::random_geometry::RandomGeometryGenerator;

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
    arena: Gd<Polygon2D>,
    ray: Gd<RayCast2D>,
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
            ray: RayCast2D::new_alloc(),
        }
    }

    fn ready(&mut self) {
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

                // astar.add_point(i + 10 * j, Vector2::new(x, y));

                // godot_print!("Point: {}", i + 10 * j);

                // godot_print!(
                //     "Points {} {} connected: {}",
                //     i + 10 * j,
                //     (i - 1) + 10 * j,
                //     astar.are_points_connected(i + 10 * j, (i - 1) + 10 * j)
                // );
                // godot_print!(
                //     "Points {} {} connected: {}",
                //     i + 10 * j,
                //     i + 10 * (j - 1),
                //     astar.are_points_connected(i + 10 * j, i + 10 * (j - 1))
                // );

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

        let mut ray: Gd<RayCast2D> = RayCast2D::new_alloc();
        ray.set_position(Vector2::new(0.0, 0.0));
        ray.set_target_position(Vector2::new(1024., 1024.));
        ray.set_collision_mask_value(1, true);
        ray.set_enabled(true);

        self.base_mut().add_child(ray.clone());

        self.ray = ray.clone();

        godot_print!("Ray collision: {}", ray.is_colliding());
    }

    fn process(&mut self, _delta: f64) {
        if self.ray.is_colliding() {
            godot_print!("Ray collision: {}", self.ray.get_collision_point());
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
