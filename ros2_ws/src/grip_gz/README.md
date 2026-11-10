# grip_gz - Mechanics of Solids Adaptive Grip Simulation in MuJoCo (ROS 2)

A 2-link planar robotic arm (shoulder, elbow, wrist) with an n-finger (3, 4, or 5) 3-phalanx
tendon-synergy hand approaches, probes, squeezes, and lifts an unknown test object in MuJoCo.

CAD input:  bash mujoco_2.sh part.stl [material] [unit] [--mu 0.6 --E 5e6 --strength 2e5 --mass 0.2 ...]
            or press BROWSE STL / OBJ in the dashboard's CAD INPUT bar (material + every property editable).
Motion:     fingertips close at <= 25 mm/s and at 8 mm/s for the last 6 mm before contact, the arm moves
            at <= 120 mm/s (50 mm/s on the final descent), the lift runs at 10 mm/s and stops while the
            part slips, and the grip force is slew-limited so delicate parts are never jabbed.
