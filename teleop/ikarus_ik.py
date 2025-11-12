import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
import time
import os 
import sys

"""Two IK controllers: NLP based and Differential QP based (clik)"""

class Ikarus_IK:
    def __init__(self):

        self.robot = pin.RobotWrapper.BuildFromURDF("icarus_urdf_stack/icarus_orca_description/urdf/" \
        "icarus_orca_flat.urdf")

        self.locked_joints_names = ["head1","Head2",
                                    "right_thumb_mcp","right_thumb_abd","right_thumb_pip","right_thumb_dip",
                                    "right_index_abd","right_index_mcp","right_index_pip",
                                    "right_middle_abd","right_middle_mcp","right_middle_pip",
                                    "right_ring_abd","right_ring_mcp","right_ring_pip",
                                    "right_pinky_abd","right_pinky_mcp","right_pinky_pip"]
        
        #locked joints will go to neutral position
        self.reduced_robot = self.robot.buildReducedRobot(list_of_joints_to_lock=self.locked_joints_names)

        self.reduced_robot.model.addFrame(
            pin.Frame("R_ee",
                      self.reduced_robot.model.getJointId("right_wrist"),
                      pin.SE3(np.eye(3), np.array([-0.002,0.00144,0.03872]).T ),
                      pin.FrameType.OP_FRAME,
                      )
        )



        self.cmodel = cpin.Model(self.reduced_robot.model) #symbolic casadi model
        self.cdata = self.cmodel.createData() #allocates runtime buffers, where jacobians, etc are stored

        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq,1) #symbolic joint position
        self.cTf_r = casadi.SX.sym("ee_T", 4, 4) #transformation matrix of right end-effector
        
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.R_hand_id = self.cmodel.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_r],
            

        )


