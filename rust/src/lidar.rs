mod random_geometry;

use godot::prelude::*;

use godot::classes::{INode2D, Node2D};

use crate::lidar::random_geometry::RandomGeometryGenerator;

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
}

#[godot_api]
impl INode2D for Lidar {
    fn init(base: Base<Node2D>) -> Self {
        godot_print!("Hello, world!");

        Self { base }
    }

    fn ready(&mut self) {
        let geom = self.generate_geometry();

        // Register the geometry as a child of the Lidar node
        // Children are 'ready' before their parent
        self.base_mut().add_child(geom.clone());

        let poly_len = geom.bind().polygons.len();

        godot_print!("I am LIDAR and I have {} polygons", poly_len);
    }
}

impl Lidar {
    fn generate_geometry(&mut self) -> Gd<RandomGeometryGenerator> {
        godot_print!("Generating geometry!");

        let geom = random_geometry::RandomGeometryGenerator::new();
        geom
    }
}
