use godot::classes::{INode2D, Node2D, Polygon2D};
use godot::prelude::*;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct RandomGeometryGenerator {
    base: Base<Node2D>,
    pub polygons: Vec<Gd<Polygon2D>>,
}

#[godot_api]
impl INode2D for RandomGeometryGenerator {
    fn init(base: Base<Node2D>) -> Self {
        Self {
            base,
            polygons: Vec::new(),
        }
    }

    fn ready(&mut self) {}
}

impl RandomGeometryGenerator {
    pub fn new(seed: u64) -> Gd<Self> {
        let mut generator = Gd::from_init_fn(|base| Self {
            base,
            polygons: Vec::new(),
        });

        generator.bind_mut().generate(seed);

        generator
    }

    fn generate(&mut self, seed: u64) {
        const NUM_SHAPES: i32 = 100;

        let screen_width = 1024.0;
        let screen_height = 1024.0;
        let arena_width = 1024.0;
        let arena_height = 1024.0;
        let wall_thickness = 10.0;

        let mut rng = StdRng::seed_from_u64(seed);
        let mut polygons = Vec::new();

        for _ in 0..NUM_SHAPES {
            if rng.gen_range(0.0..1.0) < 0.5 {
                let square = self.generate_random_square(&mut rng, screen_width, screen_height);
                self.base_mut().add_child(square.clone());
                polygons.push(square);
            } else {
                let circle = self.generate_random_circle(&mut rng, screen_width, screen_height);
                self.base_mut().add_child(circle.clone());
                polygons.push(circle);
            }
        }

        let walls = self.create_arena_walls(arena_width, arena_height, wall_thickness);

        for wall in walls {
            self.base_mut().add_child(wall.clone());
            polygons.push(wall);
        }

        self.polygons = polygons;
    }

    fn generate_random_square(
        &mut self,
        rng: &mut StdRng,
        screen_width: f32,
        screen_height: f32,
    ) -> Gd<Polygon2D> {
        let mut polygon = Polygon2D::new_alloc();

        let size = rand_range(rng, 10.0, 100.0);

        let mut vertices = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(size, 0.0),
            Vector2::new(size, size),
            Vector2::new(0.0, size),
        ];

        let translation = Vector2::new(
            rand_range(rng, 0.0, screen_width - size),
            rand_range(rng, 0.0, screen_height - size),
        );

        for vertex in vertices.iter_mut() {
            *vertex += translation;
        }

        polygon.set_polygon(vertices.into());

        let color = Color::from_rgba(180. / 255., 214. / 255., 205. / 255., 1.0);
        polygon.set_color(color);

        polygon
    }

    fn generate_random_circle(
        &mut self,
        rng: &mut StdRng,
        screen_width: f32,
        screen_height: f32,
    ) -> Gd<Polygon2D> {
        let mut circle = Polygon2D::new_alloc();

        let radius = rand_range(rng, 10.0, 100.0);
        let mut polygon = self.create_circle_polygon(radius);

        let translation = Vector2::new(
            rand_range(rng, 0.0, screen_width - radius),
            rand_range(rng, 0.0, screen_height - radius),
        );

        for vertex in polygon.iter_mut() {
            *vertex += translation;
        }

        circle.set_polygon(polygon.into());

        let color = Color::from_rgba(180. / 255., 214. / 255., 205. / 255., 1.0);
        circle.set_color(color);

        circle
    }

    fn create_circle_polygon(&self, radius: f32) -> Vec<Vector2> {
        let num_points = 32;
        let mut points = Vec::new();

        for i in 0..num_points {
            let angle = std::f32::consts::PI * 2.0 * i as f32 / num_points as f32;
            points.push(Vector2::new(angle.cos(), angle.sin()) * radius);
        }

        points
    }

    fn create_arena_walls(
        &self,
        arena_width: f32,
        arena_height: f32,
        wall_thickness: f32,
    ) -> Vec<Gd<Polygon2D>> {
        let mut walls = Vec::new();

        walls.push(self.create_wall(arena_width, wall_thickness, Vector2::new(0.0, 0.0)));
        walls.push(self.create_wall(
            arena_width,
            wall_thickness,
            Vector2::new(0.0, arena_height - wall_thickness),
        ));
        walls.push(self.create_wall(wall_thickness, arena_height, Vector2::new(0.0, 0.0)));
        walls.push(self.create_wall(
            wall_thickness,
            arena_height,
            Vector2::new(arena_width - wall_thickness, 0.0),
        ));

        walls
    }

    fn create_wall(&self, width: f32, height: f32, position: Vector2) -> Gd<Polygon2D> {
        let mut wall = Polygon2D::new_alloc();

        let vertices = vec![
            Vector2::new(0.0, 0.0) + position,
            Vector2::new(width, 0.0) + position,
            Vector2::new(width, height) + position,
            Vector2::new(0.0, height) + position,
        ];

        wall.set_polygon(vertices.into());
        wall.set_color(Color::from_rgba(0.5, 0.5, 0.5, 1.0));

        wall
    }
}

fn rand_range(rng: &mut StdRng, min: f32, max: f32) -> f32 {
    rng.gen_range(min..max)
}
