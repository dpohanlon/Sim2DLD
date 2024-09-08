<p align="center">
  <img width="400" height="400" src="https://github.com/dpohanlon/Sim2DLD/blob/main/assets/lidar.png">
  <br>
  Two dimensional LIDAR simulation in Godot.
</p>

Building
---

Check out the GitHub repository
```bash
git clone https://github.com/dpohanlon/Sim2DLD.git
```

Build the Rust library
```bash
cd Sim2DLD;
cargo build --verbose --manifest-path=rust/Cargo.toml;
```

If necessary, get the Godot export templates (and put them in the correct location for Linux)

```bash
wget https://github.com/godotengine/godot/releases/download/4.3-stable/Godot_v4.3-stable_export_templates.tpz;
mv Godot_v4.3-stable_export_templates.tpz Godot_v4.3-stable_export_templates.zip;
unzip Godot_v4.3-stable_export_templates.zip;
mkdir -p /usr/local/share/godot/export_templates/4.3.stable/;
mv templates/linux_debug.x86_64 /usr/local/share/godot/export_templates/4.3.stable/linux_debug.x86_64;
mv templates/linux_release.x86_64 /usr/local/share/godot/export_templates/4.3.stable/linux_release.x86_64;
```

Build the binary with Godot
```bash
godot --headless --export-debug "Linux" lidar.x86_64
```

Usage
---

Running the binary will run the simulator on random geometry, rendering the lidar path and returns. If the default path does not exist, a new configuration is generated.

At the moment there are very few configuration options, but the total number of iterations and the output directory can be set, along with an option to run in 'headless' mode where nothing is rendered (which speeds up simulation greatly).

```bash
lidar.x86_64 --headless --n_iterations 1 --out_dir lidar_out
```

For each iteration the lidar returns at every frame are saved, along with the true trajectory.
