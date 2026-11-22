import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("first_model.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        data.ctrl[0] = 0.2  

        mujoco.mj_step(model, data)

        viewer.sync()

        print(
            f"Zadany: {data.ctrl[0]:.3f} | "
            f"Aktualny: {data.qpos[0]:.3f} | "
            f"Prędkość: {data.qvel[0]:.3f} | "
            f"Siła silnika: {data.actuator_force[0]:.3f}"
        )

        time.sleep(0.05)