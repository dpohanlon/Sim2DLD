use godot::classes::{INode2D, Node2D, Polygon2D};
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct RandomGeometryGenerator {
    base: Base<Node2D>,
    pub polygons: Vec<Gd<Polygon2D>>,
}

#[godot_api]
impl INode2D for RandomGeometryGenerator {
    fn init(base: Base<Node2D>) -> Self {
        // godot_print!("Hello, world! I am RandomGeometryGenerator");

        Self {
            base,
            polygons: Vec::new(),
        }
    }

    fn ready(&mut self) {
        const NUM_SHAPES: i32 = 100;
        let screen_width = 1024.0;
        let screen_height = 1024.0;

        let mut polygons = Vec::new();

        for _ in 0..NUM_SHAPES {
            if rand::random::<f32>() < 0.5 {
                // godot_print!("Generating square!");
                let square = self.generate_random_square(screen_width, screen_height);
                polygons.push(square.clone());
                self.base_mut().add_child(square);
            } else {
                // godot_print!("Generating circle!");
                let circle = self.generate_random_circle(screen_width, screen_height);
                polygons.push(circle.clone());
                self.base_mut().add_child(circle);
            }
        }

        // godot_print!(
        //     "I am RandomGeometry and I have {} polygons",
        //     self.polygons.len()
        // );
        //

        let arena_width = 1024.0; // Example arena size
        let arena_height = 1024.0; // Example arena size
        let wall_thickness = 10.0; // Example wall thickness

        // Generate walls
        let walls = self.create_arena_walls(arena_width, arena_height, wall_thickness);

        // Add walls to the scene
        for wall in walls {
            polygons.push(wall.clone());
            self.base_mut().add_child(wall);
        }

        self.polygons = polygons;
    }
}

impl RandomGeometryGenerator {
    pub fn new() -> Gd<Self> {
        Gd::from_init_fn(|base| Self {
            base,
            polygons: Vec::new(),
        })
    }

    fn generate_random_square(&mut self, screen_width: f32, screen_height: f32) -> Gd<Polygon2D> {
        let mut polygon = Polygon2D::new_alloc();

        // Define the size of the square
        let size = rand_range(10.0, 100.0);

        // Define the vertices for the square
        let mut vertices = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(size, 0.0),
            Vector2::new(size, size),
            Vector2::new(0.0, size),
        ];

        let translation = Vector2::new(
            rand_range(0.0, screen_width - size),
            rand_range(0.0, screen_height - size),
        );

        for vertex in vertices.iter_mut() {
            *vertex += translation;
        }

        polygon.set_polygon(vertices.into());

        // Set the color for the square
        let color = Color::from_rgba(180. / 255., 214. / 255., 205. / 255., 1.0);
        polygon.set_color(color);

        // Rather than setting position, set points directly to avoid having to transform from local -> global coords
        //
        // let position = Vector2::new(
        //     rand_range(0.0, screen_width - size),
        //     rand_range(0.0, screen_height - size),
        // );
        // polygon.set_position(position);

        polygon
    }

    fn generate_random_circle(&mut self, screen_width: f32, screen_height: f32) -> Gd<Polygon2D> {
        let mut circle = Polygon2D::new_alloc();

        let radius = rand_range(10.0, 100.0);
        let mut polygon = self.create_circle_polygon(radius);

        let translation = Vector2::new(
            rand_range(0.0, screen_width - radius),
            rand_range(0.0, screen_height - radius),
        );

        for vertex in polygon.iter_mut() {
            *vertex += translation;
        }

        circle.set_polygon(polygon.into());

        let color = Color::from_rgba(180. / 255., 214. / 255., 205. / 255., 1.0);
        circle.set_color(color);

        // let position = Vector2::new(
        //     rand_range(0.0, screen_width),
        //     rand_range(0.0, screen_height),
        // );
        // circle.set_position(position);

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

    // Function to generate four walls for an arena with specified size and wall thickness
    fn create_arena_walls(
        &self,
        arena_width: f32,
        arena_height: f32,
        wall_thickness: f32,
    ) -> Vec<Gd<Polygon2D>> {
        let mut walls = Vec::new();

        // Top wall
        walls.push(self.create_wall(arena_width, wall_thickness, Vector2::new(0.0, 0.0)));

        // Bottom wall
        walls.push(self.create_wall(
            arena_width,
            wall_thickness,
            Vector2::new(0.0, arena_height - wall_thickness),
        ));

        // Left wall
        walls.push(self.create_wall(wall_thickness, arena_height, Vector2::new(0.0, 0.0)));

        // Right wall
        walls.push(self.create_wall(
            wall_thickness,
            arena_height,
            Vector2::new(arena_width - wall_thickness, 0.0),
        ));

        walls
    }

    fn create_wall(&self, width: f32, height: f32, position: Vector2) -> Gd<Polygon2D> {
        let mut wall = Polygon2D::new_alloc();

        // Offset the vertices by the position
        let vertices = vec![
            Vector2::new(0.0, 0.0) + position,
            Vector2::new(width, 0.0) + position,
            Vector2::new(width, height) + position,
            Vector2::new(0.0, height) + position,
        ];

        wall.set_polygon(vertices.into());

        // Set wall color (optional)
        wall.set_color(Color::from_rgba(0.5, 0.5, 0.5, 1.0)); // Gray color

        wall
    }
}

// Helper function for generating random float range
fn rand_range(min: f32, max: f32) -> f32 {
    rand::random::<f32>() * (max - min) + min
}
