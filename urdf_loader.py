import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np

class loadRobotModel():
    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.pos = pin.neutral(self.model)

    def calculateCom(self, plot=False):

        # Compute forward kinematics
        pin.forwardKinematics(self.model, self.data, self.pos)

        pin.updateFramePlacements(self.model, self.data)

        com = pin.centerOfMass(self.model, self.data)

        com_lenth_vector = com - self.data.oMi[-1].translation
        com_lenth = (com_lenth_vector[0]**2+com_lenth_vector[2]**2)**(1/2)

        
        return com, com_lenth

    
    def calculateMass(self):
        body_mass = 0
        for name, inertia, oMi in zip(self.model.names, self.model.inertias, self.data.oMi):
            body_mass += inertia.mass
        return body_mass
    
    

if __name__=="__main__":
    urdf_path = "/home/csl/test/mujoco_course/crazydog_urdf/urdf/crazydog_urdf.urdf"
    robot = loadRobotModel(urdf_path=urdf_path)
    com, com_lenth = robot.calculateCom(plot=True)
    print('com', com, com_lenth)
    body_mass = robot.calculateMass()
    print('mass', body_mass)