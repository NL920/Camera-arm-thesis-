# Camera Arm Thesis

Simulation of a 2-DOF camera arm in [MuJoCo](https://mujoco.org/). The arm has a base, a lower link (~31 cm) and an upper link (~23 cm), joined by two hinge joints that rotate around the Y axis, so the arm moves in the X–Z plane.

## Repository layout

```
dof2/                    MuJoCo model of the 2-DOF arm (current work)
  first_model.xml        Model: bodies, joints, limits, motors
  meshes/                STL parts (in millimetres) + Fusion 360 source
  range_test.py          Loops through the full range of motion + an IK tracking pose
  show_model.py          Inverse kinematics to a target point, then drives the arm there
  slider.py              Slider controlling joint1
  test_diagnose.py       Holds joint1 at a fixed angle and prints target/actual/torque
  test_mujoco.py         Basic model check (prints joint1 limits, drives joint1)
  movement_test.py       Older PyBullet test (needs robot_arm.urdf next to it)
  requirements.txt       Python dependencies
robot_arm.urdf           Earlier multi-joint URDF model (PyBullet)
symulate.py              PyBullet simulation of robot_arm.urdf
```

## Setup

Requires Python 3.10+ (3.12/3.13 tested).

**Windows**
```bat
cd dof2
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

**macOS / Linux**
```bash
cd dof2
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The virtual environment is not committed (see `.gitignore`); each person creates their own.

## Running

Run scripts from inside `dof2/`, since they load `first_model.xml` by relative path.

```bash
python range_test.py            # Windows / Linux
mjpython range_test.py          # macOS: the MuJoCo viewer requires mjpython
python range_test.py --no-viewer  # one loop, prints a table, no window
```

The same applies to `show_model.py`, `slider.py`, `test_diagnose.py` and `test_mujoco.py`.

## Model summary (`dof2/first_model.xml`)

| Item | Value |
|---|---|
| Mesh scale | `0.001` (STLs are in mm) |
| Material | Aluminium, density 2700 kg/m³ (masses computed from solid meshes) |
| Masses | base 400 g, link1 496 g, midjoint2 23 g, link2 380 g |
| joint1 (base) range | −1.0 … 1.2 rad, keeps the arm clear of the base |
| joint2 (elbow) range | −1.5 … 0 rad, folds one way only (no hyperextension) |
| Motors | position actuators, `kp=100`, `kv=4` |
| Motor torque limit | ±5 Nm **placeholder**: set to the real motors' stall torque |
| Integrator | `implicitfast` |

Peak holding torque in the range test is about 2 Nm on joint1.

## Known gaps

- **Camera:** not modelled; its mass at the `end_effector` site will raise the joint torques.
- **Masses:** assume solid aluminium; hollow or profiled links will be lighter.
- **Collisions:** turned off on all parts (`contype="0"`). The joint limits are what keep the arm away from the base.
