# grip_gz — Adaptive Robotic Gripping Guided by Solid Mechanics

A simulation (ROS 2 + MuJoCo) of a robotic arm and multi-finger hand that picks up an object, from a steel block to an egg or your own CAD part, using **just enough force to hold it and never enough to break it.**

## The theme
Squeeze too little and the object slips. Squeeze too much and it gets crushed, cracked or dented. The safe grip lives in a window between the two:

- **Lower limit:** the force needed so friction carries the weight, `N_slip = m·g / (μ · fingers)`.
- **Upper limit:** the force at which the material itself would fail (or the gripper/shell limit is hit).

The whole project calculates, checks and displays that window using solid-mechanics ideas: stress and strain inside a body, Mohr's circle, strength theories, bending, twisting, energy methods and buckling.

## What the project calculates
**For the grip**
- Weight, friction and the minimum holding force, using the real shape and real centre of mass of the part.
- Where each fingertip should touch, how the hand should be turned, and whether the fingers would hit the part, table or each other.
- If no safe grip exists, it says so and refuses instead of lifting anyway.
- Every plan is re-tested in MuJoCo's physics: does the part really come up, with no palm or finger-link touching it?

**For the object (what the squeeze does to it)**
- Size of the contact patch, how far the pad sinks in, and the peak and average pressure.
- Stresses just under the surface, where damage starts: principal stresses, maximum shear, Von Mises, Tresca, Rankine.
- Safety factor against yielding, crushing or cracking, plus how much of the patch is sticking rather than sliding.
- A full stress-and-strain report at that point: Mohr circles, stress on any chosen plane, volume change versus shape change, stored strain energy, element equilibrium.
- The largest safe grip force according to five different strength theories, and a clear "safe window / no safe grip" verdict.

**For the robot itself**
- Wrist shaft under bending, twisting and shear together: stress, safety factor, twist angle, heaviest part it can carry.
- Arm links as beams: shear force and bending moment diagrams, stress, sag, and the payload that limits the arm (arm bending or grip friction).
- Finger bending worked out three ways (direct integration, Castigliano, unit load), compared with the joint-spring model.
- Buckling load of arm and fingers, plus extra worked cases (thin shells, pressurised tubes, curved and layered beams, thermal stress, fatigue, plasticity).

## How the hand behaves
The controller is **blind**: it has no camera and no knowledge of the object. It only feels force.
1. Calibrates its sensors, then closes until it touches.
2. Squeezes gently to feel how stiff the object is, and sets a safe force cap from that.
3. Starts lifting and checks for slip. If the object creeps, it pokes lightly, then raises the force smoothly and pauses the lift until the grip is firm.
4. Along the way it estimates the object's **mass, friction and stiffness**, and prints them next to the true values.

An optional fixed-25 N mode is included to show why a fixed force fails (crushes the egg, slips on wet glass).

## Objects and CAD
- Nine built-in specimens: steel (1 and 2 kg), apple, egg, glass, wet glass, empty and full bottle, foam.
- Load any STL/OBJ: it repairs messy files, finds the right way up, fixes unit mistakes, measures curvature, and gets mass and centre of mass from the volume. Sixteen material presets, every value can be overridden.

## What you see
A 3D lab scene with the arm and hand, red dots and arrows showing where and how hard each fingertip pushes, the part turning red when crushed, and a dashboard with live force and stress graphs, per-finger table, a benchmark of all objects and six analysis pages.

## Analysis graphs (dashboard → ANALYSIS → OPEN ANALYSIS GRAPHS)
Six pages, all drawn from the live grasp numbers. Sliders and drop-downs let you change the plane angle, beam, finger cross-section, strength theory and safety factor.
1. **Stress state:** the full stress at the critical point under the pad, its principal values, Mohr circles with every possible plane, the push and slide on a plane you choose, and the matching strains and stored energy.
2. **Element equilibrium:** a tiny element of the object with all the forces on its faces, a check that they balance, and a table of the same results for all nine specimens.
3. **Wrist shaft:** free-body picture, stress across the tube, Mohr circles at the worst point, Von Mises and Tresca around the section, and safety factor versus object mass.
4. **Beams:** shear force and bending moment diagrams, stress across the section, sag with and without shear, and a same-weight comparison of cross-section shapes (arm link or finger).
5. **Finger deflection:** bending moment curves, strain energy versus force, and the tip movement found three ways and compared with the joint-spring model.
6. **Failure limits:** five strength theories side by side, the largest safe grip force from each, utilisation versus force, and the final safe-window or no-safe-grip verdict.

## Run
```bash
bash run_mujoco_simulation.sh                      # built-in objects
bash run_mujoco_simulation.sh part.stl             # your part (aluminium, mm)
bash run_mujoco_simulation.sh part.stl foam cm --mu 0.7 --mass 0.12
FINGERS=3 bash run_mujoco_simulation.sh ...        # 3, 4 or 5 fingers
python -m grip_gz.physics_check all 5              # physics test only
python3 -m grip_gz.nptel_priority apple small out  # offline analysis report
```
Needs ROS 2 Humble, numpy, matplotlib, tkinter and mujoco. The script builds everything into `~/mos_ws` and launches it.

## Good to know
Parts must be roughly 21 mm tall or more; narrow parts (< 35 mm) work better with 3–4 fingers. Arm-tube, shaft and shell dimensions are assumed values, listed in `mech_extras.py` and `nptel_priority.py`, and can be edited.

## Reference
Theory follows the NPTEL *Mechanics of Solids* course by Prof. Ajeet Kumar, IIT Delhi.
