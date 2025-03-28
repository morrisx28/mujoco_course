import mujoco


def printSceneInformation(mj_model):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(mj_model.nbody):
            name = mujoco.mj_id2name(mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(mj_model.njnt):
            name = mujoco.mj_id2name(mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(mj_model.nu):
            name = mujoco.mj_id2name(
                mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(mj_model.nsensor):
            name = mujoco.mj_id2name(
                mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    mj_model.sensor_dim[i],
                )
            index = index + mj_model.sensor_dim[i]
        print(" ")

mj_model = mujoco.MjModel.from_xml_path('biped_wheel.xml')
printSceneInformation(mj_model)
