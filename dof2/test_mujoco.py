import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("first_model.xml")
data = mujoco.MjData(model)

print("Czy przegub ma ograniczenia:",
      model.jnt_limited[0])

print("Zakres przegubu:",
      model.jnt_range[0])

print("Zakres sterowania:",
      model.actuator_ctrlrange[0])

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        data.ctrl[0] = 1.0

        mujoco.mj_step(model, data)

        #print("Zadany kąt:", data.ctrl[0])
        #print("Aktualny kąt:", data.qpos[0])

        viewer.sync()
        time.sleep(0.01)