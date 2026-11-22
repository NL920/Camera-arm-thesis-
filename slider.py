
import time
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

model = mujoco.MjModel.from_xml_path("first_model.xml")
data = mujoco.MjData(model)

# Początkowy kąt
angle = 0.0

# Okno z suwakiem
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)

slider_ax = plt.axes([0.2, 0.1, 0.65, 0.04])

slider = Slider(
    slider_ax,
    'Kąt [stopnie]',
    -90,
    90,
    valinit=0
)

def update(val):
    global angle
    angle = slider.val

slider.on_changed(update)

# Uruchomienie MuJoCo
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # Zamiana stopni na radiany
        data.ctrl[0] = angle * 3.14159 / 180

        mujoco.mj_step(model, data)

        viewer.sync()

        plt.pause(0.001)
