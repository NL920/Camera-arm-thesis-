"""
Test zakresu ruchu: ramie w petli przechodzi przez punkty pokazujace
caly zakres obu przegubow.

Uruchomienie (macOS):   mjpython range_test.py
Bez okna (jedna petla, tabela wynikow):   python range_test.py --no-viewer
"""

import os
import sys
import time
import numpy as np
import mujoco

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "first_model.xml")
SEGMENT_TIME = 2.5   # czas przejscia miedzy punktami [s]
HOLD_TIME = 0.5      # postoj w kazdym punkcie [s]

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")

# Zakresy czytane z modelu, wiec test dopasuje sie do zmian w XML
q1_min, q1_max = model.jnt_range[0]
q2_min, q2_max = model.jnt_range[1]   # q2_max = 0 (wyprost), q2_min = maks. zgiecie

# Punkty testowe (q1, q2) w radianach
WAYPOINTS = [
    ("home - pionowo, wyprost",  (0.0,    q2_max)),
    ("joint1 min",               (q1_min, q2_max)),
    ("joint1 min + zgiecie",     (q1_min, q2_min)),
    ("joint2 maks. zgiecie",     (0.0,    q2_min)),
    ("joint1 max + zgiecie",     (q1_max, q2_min)),
    ("joint1 max",               (q1_max, q2_max)),
]


def solve_ik(target_xyz, iters=1000, damping=0.01):
    """Odwrotna kinematyka (DLS) - zwraca (q1, q2) dla punktu XYZ koncowki."""
    tmp = mujoco.MjData(model)
    jacp = np.zeros((3, model.nv))
    for _ in range(iters):
        mujoco.mj_forward(model, tmp)
        err = target_xyz - tmp.site_xpos[site_id]
        if np.linalg.norm(err) < 1e-4:
            break
        mujoco.mj_jacSite(model, tmp, jacp, None, site_id)
        J = jacp[:, :2]
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), err)
        tmp.qpos[:2] = np.clip(tmp.qpos[:2] + 0.5 * dq, model.jnt_range[:2, 0], model.jnt_range[:2, 1])
    return tuple(tmp.qpos[:2])


# Sledzenie punktu XYZ: oba przeguby pracuja jednoczesnie (jak w show_model.py)
mujoco.mj_forward(model, data)
HOME_TIP = data.site_xpos[site_id].copy()
TRACK_TARGET = HOME_TIP + np.array([0.115, 0.0, -0.066])
WAYPOINTS += [
    ("home",                        (0.0, q2_max)),
    ("sledzenie punktu XYZ (IK)",   solve_ik(TRACK_TARGET)),
]


def target_at(t):
    """Zwraca (nazwa, cel q1, q2) w chwili t - plynne przejscia (cosinus)."""
    period = SEGMENT_TIME + HOLD_TIME
    n = len(WAYPOINTS)
    k = int(t // period) % n
    tau = t % period
    name, a = WAYPOINTS[k]
    _, b = WAYPOINTS[(k + 1) % n]
    if tau < HOLD_TIME:
        return name, np.array(a)
    s = 0.5 - 0.5 * np.cos(np.pi * (tau - HOLD_TIME) / SEGMENT_TIME)
    return name, (1 - s) * np.array(a) + s * np.array(b)


def report(name, q_target):
    tip = data.site_xpos[site_id]
    print(f"{name:36s} | cel q=({q_target[0]:+.3f}, {q_target[1]:+.3f}) "
          f"| jest q=({data.qpos[0]:+.3f}, {data.qpos[1]:+.3f}) "
          f"| koncowka XYZ=({tip[0]:+.3f}, {tip[1]:+.3f}, {tip[2]:+.3f}) "
          f"| moment=({data.actuator_force[0]:+.2f}, {data.actuator_force[1]:+.2f}) Nm")


def step(t):
    name, q = target_at(t)
    data.ctrl[:] = q
    mujoco.mj_step(model, data)
    return name, q


period = SEGMENT_TIME + HOLD_TIME
dt = model.opt.timestep

if "--no-viewer" in sys.argv:
    # Jedna pelna petla bez okna; raport na koncu postoju w kazdym punkcie
    last = None
    for i in range(int(len(WAYPOINTS) * period / dt)):
        t = i * dt
        name, q = step(t)
        k = int(t // period)
        if t % period >= HOLD_TIME - dt and k != last:
            report(name, q)
            last = k
else:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last = None
        while viewer.is_running():
            start = time.time()
            name, q = step(data.time)
            k = int(data.time // period)
            if data.time % period >= HOLD_TIME - dt and k != last:
                report(name, q)
                last = k
            viewer.sync()
            time.sleep(max(0.0, dt - (time.time() - start)))
