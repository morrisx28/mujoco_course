import mujoco
import mujoco.viewer

# Load a sample model 
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)

# Run a simple simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

