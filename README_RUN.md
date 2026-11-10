# grip_gz — how to run it on Ubuntu

A geometry-true multi-finger grasping hand in **MuJoCo + ROS 2**, with a Mechanics-of-Solids dashboard.
One script, `mujoco_15.sh`, writes the whole ROS 2 package, builds it with `colcon`, and launches the simulator.

> Want to know *why* it behaves the way it does? Read [`README_PHYSICS.md`](README_PHYSICS.md).

---

## 1. Quick start

```bash
# 1. system packages (Ubuntu 22.04 + ROS 2 Humble already installed — see §3 if not)
sudo apt update
sudo apt install -y python3-pip python3-numpy python3-matplotlib python3-tk python3-scipy \
                    python3-colcon-common-extensions libgl1

# 2. run
bash mujoco_15.sh
```

The first run installs the missing Python packages (`mujoco`, `fast-simplification`, `trimesh`), generates
`~/mos_ws/src/grip_gz`, builds it and opens **two windows**: the Tk dashboard and the MuJoCo 3-D viewer.
Press **▶ RUN EPISODE** in the dashboard.

---

## 2. Requirements

| Item | Needed | Notes |
|---|---|---|
| OS | **Ubuntu 22.04 LTS** | the script `source`s `/opt/ros/humble/setup.bash` — hard-coded |
| ROS 2 | **Humble** (`ros-humble-desktop` or at least `ros-humble-ros-base` + `launch`, `launch_ros`, `rclpy`) | `ros2`, `colcon` must work |
| Python | 3.10 (the Ubuntu 22.04 system Python) | |
| Python packages | `numpy`, `matplotlib`, `tkinter` (**required**), `mujoco` (auto-installed), `scipy` (strongly recommended), `fast-simplification`, `trimesh` (optional) | `scipy` makes CAD distance fields fast; `trimesh` adds PLY/OFF/3MF/GLB import |
| Display | A desktop session with OpenGL | The dashboard is a Tk GUI and the viewer is a GLFW window — it is **not headless** |
| Disk | ~ a few hundred MB for ROS + wheels | |

The script checks `numpy`, `matplotlib` and `tkinter` and stops with the exact `apt` command if one is missing.

---

## 3. One-time setup

### 3.1 Install ROS 2 Humble (skip if `ros2` already works)

Follow the official guide if anything below has changed: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

Check:

```bash
source /opt/ros/humble/setup.bash
ros2 --help | head -3
colcon --help | head -3
```

### 3.2 Install the Python dependencies

```bash
sudo apt install -y python3-pip python3-numpy python3-matplotlib python3-tk python3-scipy libgl1
pip3 install --user mujoco fast-simplification trimesh      # the script does this for you if missing
```

Check:

```bash
python3 -c "import numpy, matplotlib, tkinter, scipy, mujoco; print('mujoco', mujoco.__version__)"
```

> `~/.local/bin` should be on your `PATH` for `pip --user` installs (Ubuntu adds it automatically at next login).

---

## 4. Running the simulator

### 4.1 Built-in specimens

```bash
bash mujoco_15.sh                              # apple, small pad, 5 fingers
OBJECT=egg   bash mujoco_15.sh                 # pick the starting specimen
OBJECT=glass PAD=large bash mujoco_15.sh       # large pad (R = 100 mm)
FINGERS=3 OBJECT=foam bash mujoco_15.sh        # 3-finger hand
```

| Variable | Values | Default | Meaning |
|---|---|---|---|
| `OBJECT` | `steel`, `steel_2kg`, `apple`, `egg`, `glass`, `wet_glass`, `bottle_empty`, `bottle_full`, `foam` | `apple` | starting specimen (changeable in the GUI) |
| `PAD` | `small`, `large` | `small` | fingertip pad radius 25 mm / 100 mm |
| `FINGERS` | `3`, `4`, `5` | `5` | hand layout |
| `MOS_WS` | any path | `~/mos_ws` | ROS 2 workspace that is created/used |

`OBJECT` and `PAD` apply only when you do **not** pass a CAD file.

### 4.2 What the script does, step by step

1. Checks `numpy`, `matplotlib`, `tkinter`; installs `mujoco`, `scipy`, `fast-simplification`, `trimesh` if missing.
2. Writes the package `grip_gz` into `$MOS_WS/src/grip_gz` (all `.py` files, `setup.py`, `package.xml`, launch file).
3. Runs `rm -rf build/grip_gz install/grip_gz` then `colcon build --packages-select grip_gz`.
4. Sources `install/setup.bash`.
5. Without a CAD file: `ros2 launch grip_gz grip.launch.py object:=… pad:=… fingers:=…`.
   With a CAD file: first `ros2 run grip_gz grip_cad …` (prints the CAD report), then the launch with `cad:=…`.

You should see this line before the build:

```
grip_gz (MuJoCo + MoS + CAD, v3) written to: /home/<you>/mos_ws/src/grip_gz
```

> ⚠️ **Every run regenerates the source files** from the script (they are written with `cat > file`) and wipes the package's `build/` and `install/`. To change a parameter permanently, edit the matching block **inside `mujoco_15.sh`**, not the copy in `~/mos_ws/src/grip_gz`.

### 4.3 Using the dashboard

| Control | What it does |
|---|---|
| **OBJECT** | choose the specimen (built-ins plus any loaded CAD part) |
| **MODE** | `Adaptive (blind)` tactile controller, or `Fixed 25 N` baseline |
| **PAD** | `small (R=25mm)` / `large (R=100mm)` |
| **FINGERS** | 3 / 4 / 5 |
| **SPEED** | 0.5× / 1× (real time) / 2× / 4× |
| **▶ RUN EPISODE** | plan the grasp, approach, close, lift 100 mm, hold |
| **⏸ PAUSE / ⏹ STOP** | pause / abort the running episode |
| **▦ BENCHMARK ALL** | run every specimen headless, adaptive **and** fixed-25 N, and fill the RESULTS table |

* **Pipeline pills** show the controller state: `CALIB → WAIT_ARM → CLOSE → SQUEEZE → SETTLE → LIFT → HOLD`.
* **KPI cards:** normal force, safety factor, Von Mises stress, lift height. **Gauges:** Mindlin stick zone `c/a`, finger-joint torque load.
* **Banner** verdicts: `GRASP & LIFT SUCCESS`, `OBJECT SLIPPED OFF`, `FAILED TO LIFT`, `CRUSHED`, `COLLISION`, `ABORTED: NO SAFE GRIP`.
* **Tabs:** `TELEMETRY`, `FINGERS` (per-finger force, torques, dome gap), `RESULTS`, `CAD REPORT`, `ANALYSIS`.
* **ANALYSIS → OPEN ANALYSIS GRAPHS** opens six pages: *stress state, element equilibrium, wrist shaft, beams, finger deflection, failure limits*, with selectors for beam, finger section, failure theory and safety factor.
* **MuJoCo viewer:** standard mouse controls — left-drag rotates, right-drag pans, scroll zooms. Maximise the window for the sharpest image. Red dots and arrows on the part show each fingertip's contact and force.

If the 3-D viewer cannot start (e.g. no OpenGL), the script keeps running **without** the 3-D window and the dashboard still works.

---

## 5. Gripping your own CAD part

```bash
bash mujoco_15.sh PART [MATERIAL] [UNIT] [--flags …]
```

| Argument | Meaning | Default |
|---|---|---|
| `PART` | `.stl` or `.obj` (also `.ply .off .3mf .glb` if `trimesh` is installed) | – |
| `MATERIAL` | `steel stainless aluminium brass titanium abs pla nylon polycarbonate glass ceramic wood rubber foam fruit thin_shell` | `aluminium` |
| `UNIT` | `m cm mm in inch ft` | `mm` |

`PART`, `MATERIAL`, `UNIT` are **positional and in that order**; flags (starting with `--`) go after them.

```bash
bash mujoco_15.sh bracket.stl                           # aluminium, millimetres
bash mujoco_15.sh sponge.stl foam cm                    # foam, centimetres
bash mujoco_15.sh part.stl foam mm --mu 0.7 --E 5e6 --strength 2e5 --mass 0.12 --kind cellular
FINGERS=3 bash mujoco_15.sh small_part.stl brass mm     # small parts: use 3 or 4 fingers
```

### Optional flags (SI units)

| Flag | Unit | Meaning |
|---|---|---|
| `--mass` | kg | override mass (otherwise `density × volume`) |
| `--density` | kg/m³ | override density |
| `--mu` | – | pad/part friction coefficient |
| `--E`, `--nu` | Pa, – | Young's modulus, Poisson ratio |
| `--strength` | Pa | yield / fracture / crush strength |
| `--F-allow` | N | hard force limit (thin-walled parts) |
| `--R-contact` | m | contact radius (otherwise measured from the mesh) |
| `--kind` | `ductile` `cellular` `brittle` `shell` | failure mode |
| `--up` | `auto z y -y x -x -z` | which axis is up in the file (default auto-detect) |
| `--rotz` | degrees | extra rotation about Z (default: narrowest width across the fingers) |
| `--scale` | – | extra uniform scale on top of the unit |
| `--no-fit` | – | do not auto-scale parts that are too big/small for the hand |
| `--name` | text | name of the part in the GUI |
| `--spec` | JSON file | give all of the above in a JSON file |

You can also load a part from the GUI: **📂 BROWSE STL / OBJ… → pick unit and material → ▶ ANALYSE & GRIP** (editable fields: mass, density, μ, E, ν, strength, F_max, R, failure mode). Analysis can take up to a minute; the result appears in the **CAD REPORT** tab together with `GRIPPABLE` / `NO SAFE GRIP` for both pads.

### Tips and size limits

* Parts need roughly **≥ 21 mm height**; parts narrower than **~35 mm** usually need `FINGERS=3` or `4`.
* The open hand spans about 15 cm (2 × 74 mm), the part's width at grasp height must fit inside that, and the largest dimension is limited to 300 mm. Parts outside the range are **auto-scaled** and the CAD report starts with a `SCALE:` warning — check it; if wrong, pick the right unit.
* Y-up models (Blender / Sketchfab exports) are detected automatically; override with `--up y`.
* Meshes with more than ~120 000 triangles are simplified automatically; open (non-watertight) meshes are treated as solids via their cross-sections.
* A part that cannot be held safely is **refused** (`NO SAFE GRIP`) with a reason and a hint — it is not lifted anyway.

CAD analysis without the GUI (after the first build, see §6):

```bash
ros2 run grip_gz grip_cad part.stl --material foam --unit cm
```

---

## 6. Command-line tools (no dashboard)

After one successful run of `mujoco_15.sh`:

```bash
source /opt/ros/humble/setup.bash
source ~/mos_ws/install/setup.bash
```

| Command | What it prints / does |
|---|---|
| `ros2 run grip_gz grip_materials small` | grip window table (`N_slip`, `N_max`, ratio, OPEN / TOO NARROW) for every specimen; use `large` for the big pad |
| `ros2 run grip_gz grip_physics all 5` | MuJoCo rigid-contact check of the planned grasp for all specimens with 5 fingers (`grip_physics apple 3` for one) — lifted / slipped, rise, slip, peak penetration |
| `ros2 run grip_gz grip_hand 5` | headless benchmark, adaptive vs fixed-25 N, for a 5-finger hand (omit the number to run 4 and 5) |
| `ros2 run grip_gz grip_cad part.stl --material foam --unit cm` | CAD report only |
| `python3 -m grip_gz.nptel_priority apple small ./report` | writes the analysis pages as PNG files into `./report` and prints the text report (arguments: specimen, pad, output folder) |
| `ros2 launch grip_gz grip.launch.py object:=egg pad:=large fingers:=4` | start the GUI directly, without rebuilding |

Launch arguments: `object`, `pad`, `seed`, `cad`, `cad_material`, `cad_unit`, `cad_args`, `fingers`.

> `setup.py` also lists `grip_judge` and `grip_world`, but the script does not generate a `judge_node` module and `world.py` has no `main()`, so those two commands will not run. Use the commands above.

---

## 7. What gets created on your machine

```
~/mos_ws/
├── src/grip_gz/            generated source (overwritten on every run)
│   ├── grip_gz/            arm_ik, materials, mos, mech_extras, controller_core, hand, shapes,
│   │                       physics_check, hand_plant, cad, meshprep, gripper_models, world,
│   │                       nptel_priority, dashboard, controller_node
│   ├── launch/grip.launch.py
│   ├── package.xml  setup.py  setup.cfg  README.md
├── build/  install/  log/  colcon output
~/.grip_gz_cad/             normalised copies of loaded CAD parts + generated gripper STL meshes
```

Clean up completely:

```bash
rm -rf ~/mos_ws ~/.grip_gz_cad
```

---

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `source: /opt/ros/humble/setup.bash: No such file` | ROS 2 Humble is not installed (§3.1). On another distro edit the `source /opt/ros/<distro>/setup.bash` line in `mujoco_15.sh`. |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
| `tkinter missing: sudo apt install python3-tk` | do exactly that |
| `numpy missing` / `matplotlib missing` | `sudo apt install python3-numpy python3-matplotlib` |
| `pip3 … externally-managed-environment` (Ubuntu 24.04) | the script targets 22.04/Humble; on 24.04 you also need ROS 2 Jazzy and a path edit. A workaround for pip is `pip3 install --user --break-system-packages mujoco …` — untested here. |
| `A module that was compiled using NumPy 1.x cannot be run in NumPy 2…` | `pip3 install --user "numpy<2"` |
| `No module named mujoco` right after install | open a new terminal, or `export PATH="$PATH:$HOME/.local/bin"` |
| Dashboard works, but no MuJoCo window | no usable OpenGL/GLFW. Run in a local desktop session (not plain SSH), `sudo apt install libgl1 libglfw3`, or try `LIBGL_ALWAYS_SOFTWARE=1 bash mujoco_15.sh`. On Wayland try an *Ubuntu on Xorg* session. |
| `Part too wide` / `Part too thin` | wrong unit or scale — pick `m/cm/mm/in`, or use `--scale`; the hand opens ~15 cm and needs ≥ 4 mm width |
| `NO SAFE GRIP` | physics, not a bug: friction too low, force limit too small, COM unsupported, or fingers do not fit. Read the message; try `FINGERS=3`, a higher `--mu`, or the large pad. |
| CAD analysis is slow | install `scipy` (`sudo apt install python3-scipy`); large meshes need up to a minute |
| Build error after editing generated files | `rm -rf ~/mos_ws/build ~/mos_ws/install ~/mos_ws/log` and rerun |
| `ERROR: CAD file not found` | check the path; it is resolved with `realpath` relative to where you run the script |

---

## 9. Suggested repository layout for GitHub

```
.
├── mujoco_15.sh            the generator script
├── README.md               ← this file (README_RUN.md renamed)
├── README_PHYSICS.md       physics, derivations, sources
└── examples/               your own STL/OBJ test parts
```

The script also writes its own short `README.md` inside the generated package (`~/mos_ws/src/grip_gz/README.md`); that is separate from the repository README.

License: `package.xml` declares **MIT**; add a matching `LICENSE` file to the repository.
