mod argument_parser;
mod lidar_state;
mod random_geometry;
mod serializer;

use std::collections::HashMap;
use std::env;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use crate::lidar::random_geometry::RandomGeometryGenerator;
use crate::lidar::serializer::write_to_json;
use crate::lidar::serializer::SerializableArray2;
use godot::classes::{
    AStar2D, CollisionPolygon2D, Geometry2D, INode2D, Label, Line2D, Node2D, Polygon2D, RayCast2D,
    RenderingServer, StaticBody2D,
};
use godot::prelude::*;
use ndarray::Array2;

#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Lidar {
    base: Base<Node2D>,
    _arena: Gd<Polygon2D>,
    parsed_args: HashMap<String, String>,
    out_dir: String,
    n_iterations: u32,
    state: lidar_state::LidarState,
}

static LIDAR_COUNT: AtomicU32 = AtomicU32::new(0);
static RUN_SEED: AtomicU64 = AtomicU64::new(0);

const ARENA_SIZE: f32 = 1024.0;
const GRID_SIZE: i64 = 100;
const N_RAYS: usize = 360;
const RAY_LENGTH: f32 = 100000.0;
const MAX_GEOMETRY_ATTEMPTS_PER_ITERATION: u32 = 100;

#[godot_api]
impl INode2D for Lidar {
    fn init(base: Base<Node2D>) -> Self {
        let polygon = Self::create_arena_polygon(ARENA_SIZE, ARENA_SIZE);

        Self {
            base,
            _arena: polygon,
            parsed_args: HashMap::new(),
            out_dir: String::from("lidar_out"),
            n_iterations: 10,
            state: lidar_state::LidarState::new(),
        }
    }

    fn ready(&mut self) {
        let count = LIDAR_COUNT.load(Ordering::Relaxed);
        godot_print!("Count {}", count);

        RenderingServer::singleton().set_default_clear_color(Color::from_rgba(
            255. / 255.,
            218. / 255.,
            118. / 255.,
            1.0,
        ));

        let arena = self._arena.clone();
        self.base_mut().add_child(arena);

        let args: Vec<String> = env::args().collect();
        self.parsed_args = argument_parser::parse_args(args);

        godot_print!("Command-line arguments: {:?}", self.parsed_args);

        if !self.configure_output_dir() {
            self.base_mut().get_tree().unwrap().quit();
            return;
        }

        if let Some(label) = self.parsed_args.get("label") {
            self.add_center_label(&label.clone(), ARENA_SIZE, ARENA_SIZE);
        }

        if let Some(n) = self.parsed_args.get("n_iterations") {
            match n.parse() {
                Ok(n) => self.n_iterations = n,
                Err(err) => {
                    godot_print!("Invalid n_iterations '{}': {}", n, err);
                    self.base_mut().get_tree().unwrap().quit();
                    return;
                }
            }
        }

        let base_seed = self.run_seed();
        let iteration = LIDAR_COUNT.load(Ordering::Relaxed);

        godot_print!("Run seed: {}", base_seed);

        let mut selected_geom: Option<Gd<RandomGeometryGenerator>> = None;
        let mut selected_path: Vec<Vector2> = Vec::new();

        for attempt in 1..=MAX_GEOMETRY_ATTEMPTS_PER_ITERATION {
            let geometry_seed = Self::mix_seed(
                base_seed
                    ^ ((iteration as u64).wrapping_mul(0x9E3779B97F4A7C15))
                    ^ attempt as u64,
            );

            godot_print!("Geometry seed: {}", geometry_seed);

            let geom = self.generate_geometry(geometry_seed);

            let poly_len = geom.bind().polygons.len();
            godot_print!(
                "I am LIDAR and I have {} polygons, geometry attempt {}",
                poly_len,
                attempt
            );

            let path = self.calculate_path(&geom);

            godot_print!("Path length: {}", path.len());

            if path.is_empty() {
                godot_print!("No path found; skipping geometry seed");
                continue;
            }

            selected_geom = Some(geom);
            selected_path = path;
            break;
        }

        let Some(geom) = selected_geom else {
            godot_print!(
                "No valid geometry found after {} attempts; stopping",
                MAX_GEOMETRY_ATTEMPTS_PER_ITERATION
            );
            self.base_mut().get_tree().unwrap().quit();
            return;
        };

        self.state.path = selected_path;

        godot_print!("Path (0): {}", self.state.path[0]);

        self.base_mut().add_child(geom.clone());

        let path_array = Array2::from_shape_vec(
            (self.state.path.len(), 2),
            self.state
                .path
                .iter()
                .flat_map(|v| vec![v.x, v.y])
                .collect(),
        )
        .unwrap();

        let serializable_path = SerializableArray2 { array: path_array };

        let count = LIDAR_COUNT.load(Ordering::Relaxed);
        let filename = format!("{}/lidar_path_{}.json", self.out_dir, count);

        match serializer::write_to_json(&filename, &serializable_path) {
            Ok(()) => {
                godot_print!("Wrote path output '{}'", filename);
            }
            Err(err) => {
                godot_print!("Failed to write '{}': {}", filename, err);
                self.base_mut().get_tree().unwrap().quit();
                return;
            }
        }

        let points = self.state.path.clone();
        for point in points.iter() {
            self.draw_point(
                point,
                Color::from_rgba(255. / 255., 78. / 255., 136. / 255., 1.0),
            );
        }

        let static_body = self.create_static_body(&geom);
        self.base_mut().add_child(static_body);

        self.initialize_rays_and_lines();
    }

    fn process(&mut self, delta: f64) {
        if self.state.path.is_empty() || self.state.path_idx >= self.state.path.len() - 1 {
            if !self.state.path.is_empty() {
                let count = LIDAR_COUNT.fetch_add(1, Ordering::Relaxed);

                let returns = std::mem::take(&mut self.state.returns);
                let serializable_arrays: Vec<serializer::SerializableArray2<f64>> = returns
                    .into_iter()
                    .map(|array| SerializableArray2 { array })
                    .collect();

                let filename = format!("{}/lidar_returns_{}.json", self.out_dir, count);

                if let Err(err) = write_to_json(&filename, &serializable_arrays) {
                    godot_print!("Failed to write '{}': {}", filename, err);
                    self.base_mut().get_tree().unwrap().quit();
                    return;
                }

                godot_print!("Wrote returns output '{}'", filename);

                let completed = count + 1;

                if completed >= self.n_iterations {
                    godot_print!("Finished {} iterations", self.n_iterations);
                    self.base_mut().get_tree().unwrap().quit();
                    return;
                }
            }

            self.base_mut().get_tree().unwrap().reload_current_scene();
            return;
        }

        let loc = self.state.path[self.state.path_idx];

        if self.state.slewing {
            let rotation_speed = self.state.slew_rate.to_radians() * delta as f32;
            let angle_diff = Self::angle_diff(self.state.angle, self.state.target_angle);
            let rotation_step = angle_diff.signum() * rotation_speed.min(angle_diff.abs());

            self.state.angle += rotation_step;

            if Self::angle_diff(self.state.angle, self.state.target_angle).abs() < 1E-4 {
                self.state.angle = self.state.target_angle;
                self.state.slewing = false;
            }

            self.update_rays_and_lines(loc, self.state.angle);

            if !self.state.slewing {
                self.state.path_idx += 1;
            }

            return;
        }

        let next_idx = self.state.path_idx + 1;
        let next_loc = self.state.path[next_idx];
        let desired_angle = Self::path_angle(loc, next_loc);
        let angle_diff = Self::angle_diff(self.state.angle, desired_angle);

        if angle_diff.abs() > 1E-4 {
            self.state.slewing = true;
            self.state.target_angle = desired_angle;
            return;
        }

        self.update_rays_and_lines(loc, self.state.angle);
        self.state.path_idx += 1;
    }
}

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

        label.set_anchor(Side::LEFT, 0.5);
        label.set_anchor(Side::TOP, 0.5);

        let position = Vector2::new(arena_width / 2.0, arena_height / 2.0);
        label.set_position(position);

        self.base_mut().add_child(label);
    }

    fn is_point_occluded(
        &self,
        x: f32,
        y: f32,
        geom: &Gd<RandomGeometryGenerator>,
        geometry2d: &mut Geometry2D,
    ) -> bool {
        let point = Vector2::new(x, y);

        for g in geom.bind().polygons.iter() {
            let poly = g.get_polygon();

            if geometry2d.is_point_in_polygon(point, poly) {
                return true;
            }
        }

        false
    }

    fn nearest_free_grid_id(free: &[bool], target_id: i64) -> Option<i64> {
        let target_i = target_id % GRID_SIZE;
        let target_j = target_id / GRID_SIZE;

        let mut best_id = None;
        let mut best_dist = i64::MAX;

        for i in 0..GRID_SIZE {
            for j in 0..GRID_SIZE {
                let id = i + GRID_SIZE * j;

                if !free[id as usize] {
                    continue;
                }

                let di = i - target_i;
                let dj = j - target_j;
                let dist = di * di + dj * dj;

                if dist < best_dist {
                    best_dist = dist;
                    best_id = Some(id);
                }
            }
        }

        best_id
    }

    fn nearest_reachable_grid_id(free: &[bool], start_id: i64, target_id: i64) -> Option<i64> {
        if !free[start_id as usize] {
            return None;
        }

        let neighbours: [(i64, i64); 8] = [
            (-1, 0),
            (0, -1),
            (-1, -1),
            (1, -1),
            (1, 0),
            (0, 1),
            (1, 1),
            (-1, 1),
        ];

        let mut visited = vec![false; free.len()];
        let mut queue = std::collections::VecDeque::new();

        visited[start_id as usize] = true;
        queue.push_back(start_id);

        while let Some(id) = queue.pop_front() {
            let i = id % GRID_SIZE;
            let j = id / GRID_SIZE;

            for (di, dj) in neighbours {
                let ni = i + di;
                let nj = j + dj;

                if ni < 0 || ni >= GRID_SIZE || nj < 0 || nj >= GRID_SIZE {
                    continue;
                }

                let neighbour_id = ni + GRID_SIZE * nj;
                let neighbour_idx = neighbour_id as usize;

                if free[neighbour_idx] && !visited[neighbour_idx] {
                    visited[neighbour_idx] = true;
                    queue.push_back(neighbour_id);
                }
            }
        }

        let target_i = target_id % GRID_SIZE;
        let target_j = target_id / GRID_SIZE;

        let mut best_id = None;
        let mut best_dist = i64::MAX;

        for i in 0..GRID_SIZE {
            for j in 0..GRID_SIZE {
                let id = i + GRID_SIZE * j;

                if !visited[id as usize] {
                    continue;
                }

                let di = i - target_i;
                let dj = j - target_j;
                let dist = di * di + dj * dj;

                if dist < best_dist {
                    best_dist = dist;
                    best_id = Some(id);
                }
            }
        }

        best_id
    }

    fn calculate_path(&self, geom: &Gd<RandomGeometryGenerator>) -> Vec<Vector2> {
        let mut astar = AStar2D::new_gd();
        let mut geometry2d = Geometry2D::singleton();
        let grid_spacing = ARENA_SIZE / GRID_SIZE as f32;
        let n_grid_points = (GRID_SIZE * GRID_SIZE) as usize;

        let mut free = vec![false; n_grid_points];

        for i in 0..GRID_SIZE {
            for j in 0..GRID_SIZE {
                let id = i + GRID_SIZE * j;
                let x = i as f32 * grid_spacing;
                let y = j as f32 * grid_spacing;

                if !self.is_point_occluded(x, y, geom, &mut geometry2d) {
                    free[id as usize] = true;
                    astar.add_point(id, Vector2::new(x, y));
                }
            }
        }

        let free_count = free.iter().filter(|&&is_free| is_free).count();
        godot_print!("AStar free grid points: {}/{}", free_count, n_grid_points);

        if free_count == 0 {
            return Vec::new();
        }

        let neighbours: [(i64, i64); 4] = [(-1, 0), (0, -1), (-1, -1), (1, -1)];

        for i in 0..GRID_SIZE {
            for j in 0..GRID_SIZE {
                let index = i + GRID_SIZE * j;

                if !free[index as usize] {
                    continue;
                }

                for (di, dj) in neighbours {
                    let ni = i + di;
                    let nj = j + dj;

                    if ni < 0 || ni >= GRID_SIZE || nj < 0 || nj >= GRID_SIZE {
                        continue;
                    }

                    let neighbour_index = ni + GRID_SIZE * nj;

                    if free[neighbour_index as usize] {
                        astar.connect_points(index, neighbour_index);
                    }
                }
            }
        }

        let Some(start_id) = Self::nearest_free_grid_id(&free, 702) else {
            return Vec::new();
        };

        let Some(end_id) = Self::nearest_reachable_grid_id(&free, start_id, 6290) else {
            return Vec::new();
        };

        godot_print!("AStar start id: {}, end id: {}", start_id, end_id);

        let path = astar.get_point_path(start_id, end_id).to_vec();

        if path.len() < 2 {
            godot_print!(
                "No usable AStar path from {} to {} despite {} free grid points",
                start_id,
                end_id,
                free_count
            );
            return Vec::new();
        }

        path
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
            let mut polygon = CollisionPolygon2D::new_alloc();
            polygon.set_polygon(poly.get_polygon());
            static_body.add_child(polygon);
        }

        static_body
    }

    fn initialize_rays_and_lines(&mut self) {
        for i in 0..N_RAYS {
            let angle = i as f32 * std::f32::consts::TAU / N_RAYS as f32;
            let direction = Vector2::new(RAY_LENGTH * angle.cos(), RAY_LENGTH * angle.sin());

            let mut ray: Gd<RayCast2D> = RayCast2D::new_alloc();
            ray.set_position(Vector2::new(100.0, 100.0));
            ray.set_target_position(direction);
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

    fn angle_diff(from: f32, to: f32) -> f32 {
        let mut diff = to - from;

        while diff > std::f32::consts::PI {
            diff -= std::f32::consts::TAU;
        }

        while diff < -std::f32::consts::PI {
            diff += std::f32::consts::TAU;
        }

        diff
    }

    fn update_rays_and_lines(&mut self, loc: Vector2, angle: f32) {
        self.state.angle = angle;

        let draw_lines = !self.parsed_args.contains_key("suppress_lines");
        let mut ray_returns: Array2<f64> = Array2::zeros((self.state.rays.len(), 2));

        let rays = &mut self.state.rays;
        let lines = &self.state.lines;

        for i in 0..rays.len() {
            let ray = &mut rays[i];

            let ray_angle = angle + i as f32 * std::f32::consts::TAU / N_RAYS as f32;
            let target_position =
                Vector2::new(RAY_LENGTH * ray_angle.cos(), RAY_LENGTH * ray_angle.sin());

            ray.set_position(loc);
            ray.set_target_position(target_position);
            ray.force_raycast_update();

            let origin = ray.get_position();

            let collision_point = if ray.is_colliding() {
                ray.get_collision_point()
            } else {
                origin + target_position
            };

            let distance = (collision_point - origin).length();

            ray_returns[[i, 0]] = distance as f64;
            ray_returns[[i, 1]] = ray_angle as f64;

            if draw_lines {
                let mut line = lines[i].clone();

                line.clear_points();
                line.add_point(origin);
                line.add_point(collision_point);
                line.set_default_color(if ray.is_colliding() {
                    Color::from_rgba(255. / 255., 140. / 255., 158. / 255., 1.0)
                } else {
                    Color::from_rgba(0.0, 1.0, 0.0, 1.0)
                });
            }
        }

        self.state.returns.push(ray_returns);
    }

    fn path_angle(loc: Vector2, next_loc: Vector2) -> f32 {
        let diff = next_loc - loc;
        diff.angle()
    }

    fn generate_geometry(&mut self, seed: u64) -> Gd<RandomGeometryGenerator> {
        random_geometry::RandomGeometryGenerator::new(seed)
    }

    fn configure_output_dir(&mut self) -> bool {
        if let Some(out_dir) = self
            .parsed_args
            .get("out_dir")
            .or_else(|| self.parsed_args.get("output_dir"))
            .or_else(|| self.parsed_args.get("lidar_out"))
        {
            self.out_dir = out_dir.clone();
        }

        let out_dir_path = std::path::PathBuf::from(&self.out_dir);
        let out_dir_abs = if out_dir_path.is_absolute() {
            out_dir_path
        } else {
            match env::current_dir() {
                Ok(cwd) => cwd.join(out_dir_path),
                Err(err) => {
                    godot_print!("Failed to determine current directory: {}", err);
                    return false;
                }
            }
        };

        self.out_dir = out_dir_abs.to_string_lossy().to_string();

        if let Err(err) = std::fs::create_dir_all(&self.out_dir) {
            godot_print!(
                "Failed to create output directory '{}': {}",
                self.out_dir,
                err
            );
            return false;
        }

        godot_print!("Saving lidar output to '{}'", self.out_dir);

        true
    }

    fn run_seed(&self) -> u64 {
        if let Some(seed) = self.parsed_args.get("seed").and_then(|s| s.parse::<u64>().ok()) {
            return seed;
        }

        let existing = RUN_SEED.load(Ordering::Relaxed);

        if existing != 0 {
            return existing;
        }

        let pid = std::process::id() as u64;
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let seed = Self::mix_seed(now ^ pid.rotate_left(17));

        match RUN_SEED.compare_exchange(0, seed, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => seed,
            Err(existing) => existing,
        }
    }

    fn mix_seed(mut x: u64) -> u64 {
        x = x.wrapping_add(0x9E3779B97F4A7C15);
        x = (x ^ (x >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94D049BB133111EB);
        x ^ (x >> 31)
    }
}
