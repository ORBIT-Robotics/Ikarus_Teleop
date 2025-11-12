import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
import time
import os 
import sys
import logging_mp
logger_mp = logging_mp.get_logger(__name__)


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
        self.cTf_r = casadi.SX.sym("ee_T", 4, 4) #transformation matrix of right end-effecto: from world to ee frame
        
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.R_hand_id = self.cmodel.getFrameId("R_ee") 
        
        #.translation gives the position vector of the actual end-effector frame
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_r],
            [
                casadi.vertcat(self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]) 
            ],

        )


        #.rotation gives the rotation matrix of the actual end-effector frame
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_r],
            [
                casadi.vertcat(cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T))
            ]
        )


        #def of optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq) #joints position
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq) #joint position at last time step
        self.param_tf_r = self.opti.parameter(4,4) #target end-effector pose in world frame
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_r))
        self.rotationnal_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        #optimization constraints = joint bound constraints: q_lower <= q <= q_upper
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit
        ))

        #optimization goal: minimize the sum of costs
        #note:weights mirror unitree style (strong on translation), to test
        self.opti.minimize(50*self.translational_cost + 1.0*self.rotationnal_cost + 0.02*self.regularization_cost + 0.1*self.smooth_cost)

        #solver options (ipopt)
        opts={
            'ipopt':{
            'print_level':0,     #quiet solver logs
            'max_iter':50,       #iteration cap per call
            'tol':1e-6           #convergence tolerance
            },
            'print_time':False,      #no walltime print
            'calc_lam_p':False       #stability tweak to avoid NaNs in some cases
        }
        self.opti.solver("ipopt",opts)

        #initialization buffers
        self.init_data=np.zeros(self.reduced_robot.model.nq)                           #last accepted q (initial guess)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4,0.3,0.2,0.1]),14)        #TODO: implement function in utils
        self.vis=None


    def solve_ik(self,right_palm,current_arm_q=None,current_arm_dq=None):
        #right_palm:4x4 target pose (world->palm) from meta quest after world alignment
        #current_arm_q:optional seed for q (size nq), if provided we start from it
        #current_arm_dq:optional measured joint velocities (size nv), only used for rnea feedforward zeroing here

        #seed the solver with last q or provided seed
        if current_arm_q is not None:
            self.init_data=current_arm_q
        self.opti.set_initial(self.var_q,self.init_data)

        #set runtime parameters: target pose and smoothness reference
        self.opti.set_value(self.param_tf_r,right_palm)        #target pose goes into the Opti parameter
        self.opti.set_value(self.var_q_last,self.init_data)    #smoothness reference is last accepted q

        try:
            #solve the nlp
            sol=self.opti.solve()
            sol_q=self.opti.value(self.var_q)                  #extract optimal q

            #apply temporal smoothing on q to reduce jitter
            self.smooth_filter.add_data(sol_q)
            sol_q=self.smooth_filter.filtered_data


            #update last accepted q
            self.init_data=sol_q

            #feedforward torques from rnea (optional output for torque control)
            #sol_tauff=pin.rnea(self.reduced_robot.model,
             #           self.reduced_robot.data,
              #          sol_q,
               #         v,
                #        np.zeros(self.reduced_robot.model.nv))


            return sol_q

        except Exception as e:
            #on failure: log, pull debug q, smooth, and return fallback like unitree code
            logger_mp.error(f"ERROR in convergence, plotting debug info.{e}")

            sol_q=self.opti.debug.value(self.var_q)            #debug q from failed solve
            self.smooth_filter.add_data(sol_q)
            sol_q=self.smooth_filter.filtered_data

            if current_arm_dq is not None:
                v=current_arm_dq*0.0
            else:
                v=(sol_q-self.init_data)*0.0

            self.init_data=sol_q

            sol_tauff=pin.rnea(self.reduced_robot.model,
                                self.reduced_robot.data,
                                sol_q,
                                v,
                                np.zeros(self.reduced_robot.model.nv))

            logger_mp.error(f"sol_q:{sol_q} \nmotorstate:\n{current_arm_q} \nright_pose:\n{right_palm}")
            

        #fallback: return prior q and zero torques if available, matching unitree pattern
        return current_arm_q, np.zeros(self.reduced_robot.model.nv)


class Ikarus_CLIK:
    #TODO: to be implemented
    pass