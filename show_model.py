
import mujoco
import mujoco.viewer
import numpy as np
import time

# Wczytanie modelu
model = mujoco.MjModel.from_xml_path("first_model.xml")
data = mujoco.MjData(model)

# Identyfikatory
joint1_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "joint1"
)
joint2_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "joint2"
)

site_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
)

motor1_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor1"
)
motor2_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor2"
)

# Indeksy zmiennych przegubow
q1 = model.jnt_qposadr[joint1_id]
q2 = model.jnt_qposadr[joint2_id]

d1 = model.jnt_dofadr[joint1_id]
d2 = model.jnt_dofadr[joint2_id]

# Poczatkowa konfiguracja
mujoco.mj_forward(model, data)
initial_qpos = data.qpos.copy()

# Punkt docelowy XYZ
start = data.site_xpos[site_id].copy()

target = start + np.array([0.05, 0.0, 0.05])

print("Punkt poczatkowy:", start)
print("Punkt docelowy:", target)

# IK - odwrotna kinematyka
jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))

for i in range(1000):

    mujoco.mj_forward(model, data)

    current = data.site_xpos[site_id].copy()
    error = target - current

    if np.linalg.norm(error) < 0.002:
        break

    mujoco.mj_jacSite(
        model, data, jacp, jacr, site_id
    )

    # Jacobian dla dwoch przegubow
    J = jacp[:, [d1, d2]]

    # Damped Least Squares
    damping = 0.01

    dq = J.T @ np.linalg.solve(
        J @ J.T + damping**2 * np.eye(3),
        error
    )

    data.qpos[q1] += dq[0] * 0.5
    data.qpos[q2] += dq[1] * 0.5

    # Ograniczenia przegubow
    for joint_id, qadr in [
        (joint1_id, q1),
        (joint2_id, q2)
    ]:
        if model.jnt_limited[joint_id]:
            low, high = model.jnt_range[joint_id]
            data.qpos[qadr] = np.clip(
                data.qpos[qadr], low, high
            )

# Docelowe katy
desired_q1 = data.qpos[q1]
desired_q2 = data.qpos[q2]

mujoco.mj_forward(model, data)

print("Osiagnieta pozycja:", data.site_xpos[site_id])
print("Blad XYZ:", np.linalg.norm(
    target - data.site_xpos[site_id]
))
print("Kat joint1:", desired_q1)
print("Kat joint2:", desired_q2)

# Przywrocenie pozycji poczatkowej
data.qpos[:] = initial_qpos
mujoco.mj_forward(model, data)

# Symulacja
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        data.ctrl[motor1_id] = desired_q1
        data.ctrl[motor2_id] = desired_q2

        mujoco.mj_step(model, data)

        viewer.sync()
        time.sleep(model.opt.timestep)
