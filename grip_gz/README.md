# grip_gz v3 - geometry-true adaptive gripper in MuJoCo (ROS 2)

What changed from v2 (finger parts penetrating asymmetric parts):
  * The planner now works on the REAL part geometry (signed-distance field of the CAD mesh), not a cylinder of
    "half the width".  Contacts, clearance and the live monitor all use the same geometry.
  * New hand (after Barrett Hand / Robotiq 3F / Yale OpenHand): opposed finger groups, a SPREAD joint per finger,
    a wrist YAW joint, 3-phalanx tendon-synergy fingers and compliant DOME fingertips (single-point contact).
  * Grasp search over aim point x wrist yaw x finger subset; friction-cone check, exact force balance, minimum
    holding force with the true centre of mass, swept clearance (finger-part, finger-table, finger-finger, palm).
  * A part that cannot be held safely is reported as NO SAFE GRIP instead of being lifted anyway.
  * physics_check.py re-tests every plan in MuJoCo rigid-contact physics (real colliding bodies, friction) and keeps
    only a plan that really lifts the part with zero palm/phalanx contact:   python -m grip_gz.physics_check all 5

CAD input:  bash mujoco_12.sh part.stl [material] [unit] [--mu 0.6 --E 5e6 --strength 2e5 --mass 0.2 ...]
            or press BROWSE STL / OBJ in the dashboard's CAD INPUT bar.
Limits:     physics_check approximates a CAD part by its mid-height outline extruded to full height (exact for
            prismatic parts).  Torsional (soft-finger) contact friction is modelled; no other compliance.

Graphics: 8192 px shadow map, 16x offscreen MSAA, 96-segment round parts, textured bench and sky box
          (maximise the MuJoCo window for the sharpest image; a 3840x2160 offscreen buffer is configured).
Small parts: STL welding is now relative to part size (no false "not watertight" on tiny parts), inverted triangles
          are repaired, unit mistakes are flagged.  Parts need >= ~21 mm height; below ~35 mm width use FINGERS=3 or 4.

