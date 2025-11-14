import mujoco
import mujoco.viewer
import time
import numpy as np

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

NUM_MOTOR = 6
# Load a sample model 
model = mujoco.MjModel.from_xml_path('/home/csl/test/mujoco_course/biped_wheel_edu/xml/scene.xml')
data = mujoco.MjData(model)
target_dof_pos = np.array([1.27, 1.27, -2.127, -2.127, 0, 0])

simulation_dt = 0.005
kps = np.array([10, 10 ,10, 10, 10, 10])
kds = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# Run a simple simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
    
        tau = pd_control(target_dof_pos, data.sensordata[:NUM_MOTOR], kps, np.zeros(6), data.sensordata[NUM_MOTOR:NUM_MOTOR + NUM_MOTOR], kds)
        # data.ctrl[:] = tau
        model.opt.timestep = simulation_dt
        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

