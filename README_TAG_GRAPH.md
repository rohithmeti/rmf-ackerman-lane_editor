# AprilTag Graph Maker

Give it a **map**, get an **orthogonal AprilTag graph** (tags = nodes, lanes = edges) for a **differential-drive AGV**, with the coordinates
in **both** the map frame and the **simulator (Gazebo world) frame**, and a ready Gazebo world with the tags on the floor at exactly those coordinates.

```
your map (.pgm + .yaml)
   │  ① Graph Maker (browser)     auto layout + edit by hand  → lanes are only horizontal / vertical, parallel or perpendicular, never crossing
   │  ② frame: map → simulator    register_map_to_world.py (automatic)  or  reference points in the GUI  or  numbers
   ▼
<name>.taggraph.json  ──③ export_gazebo.py──►  world .sdf with the tags · tag map .yaml · robot start pose · textures · printable tag sheet (PDF)
```

## Start
```bash
cd fms_lane_maker
python3 -m http.server 8000          # then open  http://localhost:8000/tag_graph_maker.html   ("Load demo map" works)
```
(Opening `tag_graph_maker.html` directly from the file manager also works, only "Load demo map" needs the small server.)

## 1. The graph (GUI)
1. **Map**: choose the `.yaml` and its `.pgm`/`.png` together. Pixels at least as bright as *Free if pixel at least* (default 240) are free; walls **and grey "unknown"** pixels block.
2. **Robot**: length, width **including the wheels**, margin. A differential-drive robot stops at every tag and **turns on the spot**, so
   * every tag needs free space of its *spin circle radius* (half the diagonal of the body) + margin all around, and
   * every lane needs half the robot width + margin on both sides of its centre line.
   Spots or lanes that do not satisfy this are dropped automatically (red ✕, "Dropped spots" button).
3. **Auto layout**: puts a lane along the centre of **every aisle** in the map (e.g. between racks) and a regular lattice (*tag spacing*) on open floor. Tune *min. gap between parallel lanes*, *shortest aisle that gets a lane* and *longest lane*.
4. **By hand**: *Draw lane* (press, drag, release; snaps to horizontal / vertical and to existing nodes), *Erase area*, *Delete lane*, Undo / Redo. *Clearance shading* shows where there is room (green: tags fit, amber: lanes only).
5. **Tag 1** (*Set tag 1*, then click a tag): ids go row by row (south → north, west → east); the chosen tag gets id `first id + 1`: the robot's start.

**Why the graph is always clean.** The graph is stored as lane *lines* (x = const over a range, or y = const over a range). Tags are created at the ends of a line, at every crossing of two lines and at least every *longest lane* metres; a lane joins two consecutive tags of a line. So:
no diagonal lane can exist; parallel lanes are just parallel lines; two lanes only meet in a tag (a crossing is always a tag, so no cross-cross edges); overlapping collinear lines are merged (no overlapping lanes). The panel under *Lanes* re-checks all of this after every edit
(diagonal lanes, crossings, tags lying on a foreign lane, parallel lanes closer than the gap).

## 2. The simulator frame ("where is the 0 of the environment?")
A map made by SLAM or drawn from a floor plan has its own origin; the simulator world has another. The GUI keeps one transform,
**`world = R(yaw) · map + (dx, dy)`**, and shows the simulator's (0, 0) as a purple cross-hair on the map. Three ways to fill it:
* **Automatic (recommended).** Make an occupancy map of the *simulator* itself (it is in the world frame by construction) and register your map onto it:
  ```bash
  # with the simulator running and the robot in it (ros2_agv):
  python3 src/agv_tag_nav/scripts/gz_floorplan.py --out sim_world --world <gazebo world name> --model <robot model> --bounds X0 X1 Y0 Y1 --dir .
  python3 register_map_to_world.py --map my_map.yaml --world-map sim_world.yaml --out registration.json
  ```
  then *Load registration.json* in section 4 (or pass `--registration` to the CLI). It searches all rotations (FFT cross-correlation), refines with a chamfer fit and prints the fit quality.
* **Reference points.** Tool *Reference point*: click a landmark on the map (a pillar, a shelf corner), type its coordinates read in the simulator; one point gives the shift, two points also the rotation (and warn if the scale is off).
* **Numbers.** If you already know dx, dy, yaw: type them.

Conventions: ROS map origin = the **lower-left pixel**, the first row of a `.pgm` is the top; angles counter-clockwise in degrees; metres.
(If the map is rotated against the world, the lanes stay parallel to the map's aisles, which is what you want: they follow the aisles.)

## 3. Export
| Button | File | Use |
|---|---|---|
| Tag map · map frame | `<name>_tag_map_map_frame.yaml` | tag id → x, y in **your map's** frame |
| Tag map · simulator frame | `<name>_tag_map_world.yaml` | same, in the **simulator** frame: the table the AGV navigation stack loads |
| Coordinates | `<name>_coordinates.csv` | id, x/y in both frames, tag yaw: for placing tags by hand / surveying a real site |
| Gazebo tags | `<name>_tags_block.sdf` | `<model>` blocks to paste into any world before `</world>` |
| **For Gazebo exporter** | `<name>.taggraph.json` | everything `export_gazebo.py` needs |
| Save / load project | `<name>_project.json` | continue editing later (the map image is loaded separately) |

```bash
python3 export_gazebo.py my_layout.taggraph.json --world-in my_world.sdf --remove-model tugbot --out-dir out
# -> out/worlds/warehouse_my_layout.sdf   world with every tag at its coordinates
#    out/config/tag_map_warehouse_my_layout.yaml, out/worlds/....spawn.yaml (start pose on tag 1), out/models/agv_tags (textures), out/tags_sheet.pdf
python3 export_gazebo.py my_layout.taggraph.json --world-in my_world.sdf --ros2-agv ~/ros2_agv     # also installs into a ros2_agv workspace
```
`--full-size` makes the printable sheet real size (A3, one tag per page); the default contact sheet is scaled. **Tag size** is the black square (default 0.20 m; with the white quiet zone the printed tag is 0.25 m): the detector and the simulator textures use the same convention.
The AprilTag 36h11 PDF you supplied is attached: [`assets/AprilTag_0-20_family36h11.pdf`](assets/AprilTag_0-20_family36h11.pdf) (family 36h11 has ids 0 – 586; `export_gazebo.py` renders any id from OpenCV's dictionary).

## 4. Without the browser
```bash
node auto_layout_cli.js --map testmap/warehouse_slam.yaml --registration registration.json --name warehouse --out warehouse.taggraph.json
```
same logic as the GUI (robot size, spacing, `--bounds`, `--start-near-world X Y`, ... see the header of the file).

## Files
`tag_graph_maker.html` GUI · `tag_graph_core.js` the logic (also used by the CLI and the tests) · `register_map_to_world.py` · `export_gazebo.py` · `auto_layout_cli.js` · `lane_math.js` map parsers (shared with the Ackermann lane maker) · `testmap/warehouse_slam.*` demo map · `assets/` tag PDF · `tests/`.

## Tests
```bash
node tests/test_core.js              # distance transform, lanes: no diagonal / crossing / overlap, frames, exports, warehouse layout
python3 tests/test_register.py       # map → world registration on synthetic cases (known shift and rotation, partial + noisy)
```

## Limits (honest)
* Orthogonality is in the **map** frame. If the registration finds a small rotation (e.g. 1° for a SLAM map), the lanes are rotated by it in the simulator frame, parallel to the real aisles but not exactly parallel to the world's x / y axes.
* The tool only knows the map: obstacles that are not in the map (or are below the lidar plane that made it) are not avoided by the lane layout.
* Registration needs both maps to show the same structures; it reports the fit quality and warns below 50 % inliers.
* The graph holds two-way lanes only (no one-way lanes yet) and one level (no lifts).
