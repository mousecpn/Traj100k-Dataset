import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import os
import glob
import random
from scipy.spatial.transform import Rotation
from franka_sim_test import setCameraOnRobotWrist

import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import qpsolvers as qp
import pickle

from spatialmath import SE3, base
import math
from typing import Union

pandaNumDofs = 7
maxV = 0.6

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
init_joint_pose1=[0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4, 0.0, 0.0]
rp = init_joint_pose1
total_data_num = 20000

def angle_axis(T, Td):
    e = np.empty(6)
    e[:3] = Td[:3, -1] - T[:3, -1]
    R = Td[:3, :3] @ T[:3, :3].T
    li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if base.iszerovec(li):
        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = base.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    e[3:] = a

    return e

def p_servo(
    wTe, wTep, gain, threshold=0.1, method="rpy"
):
    

    if isinstance(wTe, SE3):
        wTe = wTe.A

    if isinstance(wTep, SE3):
        wTep = wTep.A

    if method == "rpy":
        # Pose difference
        eTep = np.linalg.inv(wTe) @ wTep
        e = np.empty(6)

        # Translational error
        e[:3] = eTep[:3, -1]

        # Angular error
        e[3:] = base.tr2rpy(eTep, unit="rad", order="zyx", check=False)
    else:
        e = angle_axis(wTe, wTep)

    if base.isscalar(gain):
        k = gain * np.eye(6)
    else:
        k = np.diag(gain)

    v = k @ e
    arrived = True if np.sum(np.abs(e)) < threshold else False

    return v, arrived

def calculate_velocity(panda, cur_joint, tar_position, tar_orientation):
    # The pose of the Panda's end-effector
    n = 7
    panda.q = cur_joint
    Te = panda.fkine(cur_joint)

    R_Mat = np.array(p.getMatrixFromQuaternion(tar_orientation)).reshape(3,3)
    
    Tep = np.c_[R_Mat, tar_position.reshape(3,1)]
    Tep = np.r_[Tep, np.array([0,0,0,1]).reshape(1,4)]
    Tep = sm.SE3(Tep)


    # Transform from the end-effector to desired pose
    eTep = Te.inv() * Tep

    # Spatial error
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

    # Calulate the required end-effector spatial velocity for the robot
    # to approach the goal. Gain is set to 1.0
    v, arrived = p_servo(Te, Tep, 1, 0.01)

    # Gain term (lambda) for control minimisation
    Y = 0.1

    v += rand(v.shape[0]) * v * 0.5 # * np.array([1,1,1,0,0,0])

    # Quadratic component of objective function
    Q = np.eye(n + 6)

    # Joint velocity component of Q
    Q[:n, :n] *= Y

    # Slack component of Q
    Q[n:, n:] = (1 / e) * np.eye(6)

    # The equality contraints
    Aeq = np.c_[panda.jacobe(panda.q), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((n + 6, n + 6))
    bin = np.zeros(n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.05

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[:n, :n], bin[:n] = panda.joint_velocity_damper(ps, pi, n)

    # Linear component of objective function: the manipulability Jacobian
    c = np.r_[-panda.jacobm(panda.q).reshape((n,)), np.zeros(6)]

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[panda.qdlim[:n], 10 * np.ones(6)]
    ub = np.r_[panda.qdlim[:n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='daqp')

    # Apply the joint velocities to the Panda
    joint_velocity = qd[:n]

    return joint_velocity, arrived

class PandaSim(object):
    def __init__(self, offset):
        self.offset = np.array(offset)
        self.LINK_EE_OFFSET = 0.05
        self.initial_offset = 0.05
        self.workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
        self._numObjects = 5
        self._urdfRoot = pd.getDataPath()
        self._blockRandom = 0.3


        # self.object_list = self.object_setup()
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

        orn=[0.0, 0.0, 0, 1.0]
        self.init_joint_pose=[0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4, 0.0, 0.0]

        p.loadURDF(os.path.join(self._urdfRoot,"plane.urdf"),[0,0,-1])

        p.loadURDF(os.path.join(self._urdfRoot,"table/table.urdf"), 0.5000000,0.00000,-.620000,0.000000,0.000000,0.0,1.0)

        self.panda = p.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
        self.reset()
        

        # frame_start_postition, frame_posture = p.getLinkState(self.panda,11)[4:6]
        # R_Mat = np.array(p.getMatrixFromQuaternion(frame_posture)).reshape(3,3)
        # x_axis = R_Mat[:,0]
        # x_end_p = (np.array(frame_start_postition) + np.array(x_axis*5)).tolist()
        # x_line_id = p.addUserDebugLine(frame_start_postition,x_end_p,[1,0,0])# y 轴
        # y_axis = R_Mat[:,1]
        # y_end_p = (np.array(frame_start_postition) + np.array(y_axis*5)).tolist()
        # y_line_id = p.addUserDebugLine(frame_start_postition,y_end_p,[0,1,0])# z轴
        # z_axis = R_Mat[:,2]
        # z_end_p = (np.array(frame_start_postition) + np.array(z_axis*5)).tolist()
        # z_line_id = p.addUserDebugLine(frame_start_postition,z_end_p,[0,0,1])

        return
    

    def random_select_object(self):
        tar_id = random.choice(list(self.objectUids))
        return tar_id
    
    def random_approaching_orn(self, tar_id):
        obj_pose, obj_orn = p.getBasePositionAndOrientation(tar_id)
        ee_pose, ee_orn = p.getLinkState(self.panda,11)[4:6]
        R_Mat = np.array(p.getMatrixFromQuaternion(ee_orn)).reshape(3,3)

        z_direct = (np.array(obj_pose)-np.array(ee_pose))
        z_direct += (2*np.random.rand(3)-1)*0.1
        z_direct = z_direct/np.linalg.norm(z_direct)
        w = np.random.rand()
        z_direct = w * z_direct + (1-w) * np.array([0,0,-1])

        aux_axis = R_Mat[:,0]
        y_direct = np.cross(z_direct, aux_axis)
        y_direct += (2*np.random.rand(3)-1)*0.01
        y_direct = y_direct/np.linalg.norm(y_direct)

        x_direct = np.cross(y_direct, z_direct)
        x_direct = x_direct/np.linalg.norm(x_direct)

        matrix = np.array(
            [x_direct, y_direct, z_direct]).T
        
        grasp_orn = Rotation.from_matrix(matrix).as_quat()
        return matrix, grasp_orn
    
    def reset(self, reset_obejects=True):
        index = 0
        for i in range(pandaNumDofs):
            p.setJointMotorControl2(self.panda, i, p.VELOCITY_CONTROL, targetVelocity=0, force=5 * 240.)
        if reset_obejects==True:
            while(1):
                try:
                    tar_id = self.random_select_object()
                    self.remove(tar_id)
                except:
                    break
        for j in range(p.getNumJoints(self.panda)):
            p.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.panda, j)
            init_joint_pose = np.array(self.init_joint_pose)+\
                np.array([rand(), rand()*0.5, rand()*0.3, rand()*0.3, rand(), rand()*0.2, 0, 0 , 0])

            jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC):
                p.resetJointState(self.panda, j, init_joint_pose[index]) 
                index=index+1
            if (jointType == p.JOINT_REVOLUTE):
                p.resetJointState(self.panda, j, init_joint_pose[index]) 
                index=index+1
        self.init_pose, self.init_orn = p.getLinkState(self.panda,11)[4:6]
        if reset_obejects==True:
            urdfList = self.get_random_object(self._numObjects, False)
            self.objectUids = set(self.randomly_place_objects(urdfList))
        self.panda_control = rtb.models.Panda()
    
    def gripper_homing(self):
        p.setJointMotorControl2(self.panda, 9, p.POSITION_CONTROL, 0.04, force=5 * 240., maxVelocity=maxV)
        p.setJointMotorControl2(self.panda, 10, p.POSITION_CONTROL, 0.04, force=5 * 240., maxVelocity=maxV)
        # print(p.getJointState(self.panda,0)[1])
        if abs(p.getJointState(self.panda,9)[0] - 0.04) < 1e-5:
            return True
        return False
    
    def move(self, tar_id, orn=None):
        pos,_ = p.getBasePositionAndOrientation(tar_id)
        pos = np.array(pos)
        # orn = p.getQuaternionFromEuler([0.,0,0])  #math.pi/2.
        if orn is None:
            orn=[1.0, 0.0, 0.0, 0.0]
            pos[-1] += self.LINK_EE_OFFSET
        else:
            R_Mat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
            pos -= self.LINK_EE_OFFSET * R_Mat[:,2]

        jointPoses = p.calculateInverseKinematics(self.panda,11, pos, orn, ll, ul,
            jr, rp, maxNumIterations=10)
        success = True
        for i in range(pandaNumDofs):
            p.setJointMotorControl2(self.panda, i, p.POSITION_CONTROL, jointPoses[i],force=5 * 240., maxVelocity=maxV)
            # print(p.getJointState(self.panda,i)[1])
        if np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(pos)).sum() > 1e-3:
            # print(np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(pos)).sum())
            success = success & False
        return success
    
    def move2(self, tar_id, orn=None):
        pos,_ = p.getBasePositionAndOrientation(tar_id)
        pos = np.array(pos)
        # orn = p.getQuaternionFromEuler([0.,0,0])  #math.pi/2.
        if orn is None:
            orn=[1.0, 0.0, 0.0, 0.0]
            pos[-1] += self.LINK_EE_OFFSET
        else:
            R_Mat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
            pos -= self.LINK_EE_OFFSET * R_Mat[:,2]
        joint_pose = []
        joint_vel = []
        for i in range(7):
            pos_i, vel_i, _, _ = p.getJointState(self.panda,i)
            joint_pose.append(pos_i)
            joint_vel.append(vel_i)

        target_vel,arrived = calculate_velocity(self.panda_control, np.array(joint_pose), pos, orn)
        success = True
        for i in range(pandaNumDofs):
            p.setJointMotorControl2(self.panda, i, p.VELOCITY_CONTROL, targetVelocity=target_vel[i], force=5 * 240.) #+target_vel[i]*rand()
            # print(p.getJointState(self.panda,i)[1])
        if np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(pos)).sum() > 5e-2:
            # print(np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(pos)).sum())
            success = success & False
        return success
    
    def approaching(self, tar_id, orn=None):
        pos,_ = p.getBasePositionAndOrientation(tar_id)
        pos = np.array(pos)
        # orn = p.getQuaternionFromEuler([0.,0,0])  #math.pi/2.

        # approching
        jointPoses = p.calculateInverseKinematics(self.panda,11, pos, orn, ll, ul,
            jr, rp, maxNumIterations=10)
        success = True
        for i in range(pandaNumDofs):
            p.setJointMotorControl2(self.panda, i, p.POSITION_CONTROL, jointPoses[i], force=5 * 240., maxVelocity=maxV)
            # print(p.getJointState(self.panda,i)[1])
        if np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(pos)).sum() > 1e-3:
            # print(np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(pos)).sum())
            success = success & False
        return success

    
    def ready_pose(self):
        # jointPoses = self.init_joint_pose
        # jointPoses = np.array(self.init_joint_pose)+np.random.rand(9)*0.1
        jointPoses = np.array(self.init_joint_pose)+\
            np.array([rand(), (rand())*0.5, 2*(np.random.rand()-1)*0.1, 2*(np.random.rand()-1)*0.1, 2*(np.random.rand()-1)*0.1, 2*(np.random.rand()-1)*0.1, 0, 0 , 0])

        success = True
        for i in range(pandaNumDofs):
            p.setJointMotorControl2(self.panda, i, p.POSITION_CONTROL, jointPoses[i],force=5 * 240., maxVelocity=maxV)
        if np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(self.init_pose)).sum() > 1e-3:
            success = success & False
        return success
    

    def ready_pose2(self):
        joint_pose = []
        joint_vel = []
        ready_pose = self.init_pose+(2*np.random.rand(3)-1)*0.02
        for i in range(7):
            pos_i, vel_i, _, _ = p.getJointState(self.panda,i)
            joint_pose.append(pos_i)
            joint_vel.append(vel_i)
        target_vel, arrived = calculate_velocity(self.panda_control, np.array(joint_pose), np.array(ready_pose), np.array(self.init_orn))
        success = True
        for i in range(pandaNumDofs):
            p.setJointMotorControl2(self.panda, i, p.VELOCITY_CONTROL, targetVelocity=target_vel[i],force=5 * 240.)
        if np.abs(np.array(p.getLinkState(self.panda,11)[4]) - np.array(self.init_pose)).sum() > 1e-2:
            success = success & False
        return success


    def grasp(self, tar_id):
        p.setJointMotorControl2(self.panda, 9, p.POSITION_CONTROL, 0.0, force=5 * 240., maxVelocity=maxV)
        p.setJointMotorControl2(self.panda, 10, p.POSITION_CONTROL, 0.0, force=5 * 240., maxVelocity=maxV)
        if bool(p.getContactPoints(bodyA=self.panda,bodyB=tar_id)):
            # p.setJointMotorControl2(self.panda, 9, p.VELOCITY_CONTROL, targetVelocity=0.0, force=5 * 240., maxVelocity=maxV)
            # p.setJointMotorControl2(self.panda, 10, p.VELOCITY_CONTROL, targetVelocity=0.0, force=5 * 240., maxVelocity=maxV)
            return True
        return False

    # def object_setup():

    #     return
    def remove(self,tar_id):
        p.removeBody(tar_id)
        self.objectUids.remove(tar_id)
        return

    def get_random_object(self, num_objects, test):
        """Randomly choose an object urdf from the random_urdfs directory.

        Args:
        num_objects:
            Number of graspable objects.

        Returns:
        A list of urdf filenames.
        """
        if test:
            urdf_pattern = os.path.join(self._urdfRoot, 'random_urdfs/*0/*.urdf')
        else:
            urdf_pattern = os.path.join(self._urdfRoot, 'random_urdfs/*[^0]/*.urdf')
        found_object_directories = glob.glob(urdf_pattern)
        total_num_objects = len(found_object_directories)
        selected_objects = np.random.choice(np.arange(total_num_objects),
                                            num_objects)
        selected_objects_filenames = []
        for object_index in selected_objects:
            selected_objects_filenames += [found_object_directories[object_index]]
        return selected_objects_filenames
    
    def randomly_place_objects(self, urdfList):
        """Randomly places the objects in the bin.

        Args:
        urdfList: The list of urdf files to place in the bin.

        Returns:
        The list of object unique ID's.
        """

        # Randomize positions of each object urdf.
        objectUids = []
        for urdf_name in urdfList:
            # xpos = 0.4 +self._blockRandom*random.random()
            # ypos = self._blockRandom*(random.random()-.5)
            xpos = random.random()*0.8 + 0.1
            ypos = random.random()*0.8 - 0.4
            angle = np.pi/2 + self._blockRandom * np.pi * random.random()
            orn = p.getQuaternionFromEuler([0, 0, angle])
            # urdf_path = os.path.join(self._urdfRoot, urdf_name)
            # urdf_path = os.path.join(self._urdfRoot, "sphere_small.urdf")
            urdf_path = os.path.join(self._urdfRoot, "cube_small.urdf")

            uid = p.loadURDF(urdf_path, [xpos, ypos, .15],
                [orn[0], orn[1], orn[2], orn[3]], globalScaling=1.2)
            objectUids.append(uid)
            # Let each object fall to the tray individual, to prevent object
            # intersection.
            for _ in range(500):
                p.stepSimulation()
        return objectUids

def visualize(pos,orn):
    R_Mat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
    x_axis = R_Mat[:,0]
    x_end_p = (np.array(pos) + np.array(x_axis*5)).tolist()
    x_line_id = p.addUserDebugLine(pos,x_end_p,[1,0,0])# y 轴
    y_axis = R_Mat[:,1]
    y_end_p = (np.array(pos) + np.array(y_axis*5)).tolist()
    y_line_id = p.addUserDebugLine(pos,y_end_p,[0,1,0])# z轴
    z_axis = R_Mat[:,2]
    z_end_p = (np.array(pos) + np.array(z_axis*5)).tolist()
    z_line_id = p.addUserDebugLine(pos,z_end_p,[0,0,1])

def stepSimulation(iter):
    for k in range(iter):
        p.stepSimulation()
    return

class Watchdog():
    def __init__(self,limit=200):
        self.count = 0
        self.limit=limit
        return
    
    def error(self):
        self.count+=1
        if self.count>self.limit:
            return True
        return False
    
    def reset(self):
        self.count=0
        return

def rand(size=None):
    if size==None:
        return 2*np.random.rand()-1
    else:
        return 2*np.random.rand(size)-1

if __name__=="__main__":
    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    # p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
    p.setAdditionalSearchPath(pd.getDataPath())

    p.resetDebugVisualizerCamera(cameraDistance=1,cameraYaw=0,\
                                cameraPitch=-40,cameraTargetPosition=[-0.5,-0.9,1.5])
    timeStep=1./100.
    steps = 5
    p.setTimeStep(timeStep)
    p.setRealTimeSimulation(1)

    p.setGravity(0,0,-9.8)
    ee_log = []
    joint_pos_log = []
    joint_vel_log = []
    
    panda = PandaSim([0,0,0])
    # tar_id = panda.random_select_object()
    # matrix, grasp_orn = panda.random_approaching_orn(tar_id)
    # pos,orn = p.getBasePositionAndOrientation(tar_id)
    # orn=[1.0, 0.0, 0.0, 0.0]
    # visualize(pos, grasp_orn)
    watchdog = Watchdog()
    index = 0

    while(1):
        # g = p.getKeyboardEvents()
        # print(g)
        # continue
        ee_trajectory = []
        joint_pose = []
        joint_vel = []
        try:
            tar_id = panda.random_select_object()
        except:
            panda.reset()

        matrix, grasp_orn = panda.random_approaching_orn(tar_id)
        pos,orn = p.getBasePositionAndOrientation(tar_id)
        while(1):
            setCameraOnRobotWrist(p, panda.panda, 11)
            success = panda.gripper_homing()
            stepSimulation(steps)
            if watchdog.error():
                break     
            if success == True:
                break
        if watchdog.error():
            panda.reset()
            watchdog.reset()
            continue
        else:
            watchdog.reset()

        while(1):
            setCameraOnRobotWrist(p, panda.panda, 11)
            success = panda.move2(tar_id, grasp_orn)
            cur_pos, cur_orn = p.getLinkState(panda.panda,11)[4:6]
            ee_data = np.concatenate((np.array(cur_pos), np.array(cur_orn)),axis=0)
            ee_trajectory.append(ee_data)

            j_pos = []
            j_vel = []
            for i in range(7):
                pos_i, vel_i, _, _ = p.getJointState(panda.panda,i)
                j_pos.append(pos_i)
                j_vel.append(vel_i)
            joint_pose.append(np.array(j_pos))
            joint_vel.append(np.array(j_vel))
            stepSimulation(steps)
            if watchdog.error():
                break   
            if success == True:
                break
        if watchdog.error():
            panda.reset()
            watchdog.reset()
            continue
        else:
            watchdog.reset()
        joint_pos_log.append(np.array(joint_pose))
        joint_vel_log.append(np.array(joint_vel))
        ee_log.append(np.array(ee_trajectory))
        # print(ee_trajectory)
        index += 1
        print("sample:",index)
        panda.reset(reset_obejects=False)
        

        # while(1):
        #     setCameraOnRobotWrist(p, panda.panda, 11)
        #     success = panda.approaching(tar_id, grasp_orn)
        #     stepSimulation(steps)
        #     if watchdog.error():
        #         break
        #     if success == True:
        #         break
        # if watchdog.error():
        #     panda.reset()
        #     watchdog.reset()
        #     continue
        # else:
        #     watchdog.reset()
        
        # while(1):
        #     setCameraOnRobotWrist(p, panda.panda, 11)
        #     success = panda.grasp(tar_id)
        #     stepSimulation(steps)
        #     if watchdog.error():
        #         break
        #     if success == True:
        #         break
        # if watchdog.error():
        #     panda.reset()
        #     watchdog.reset()
        #     continue
        # else:
        #     watchdog.reset()
        
        # while(1):
        #     setCameraOnRobotWrist(p, panda.panda, 11)
        #     success = panda.ready_pose2()
        #     stepSimulation(steps)
        #     if watchdog.error():
        #         break
        #     if success == True:
        #         break
        # if watchdog.error():
        #     panda.reset()
        #     watchdog.reset()
        #     continue
        # else:
        #     watchdog.reset()
        panda.remove(tar_id)

        if len(ee_log)>total_data_num:
            data_dict = {}
            data_dict['frequency'] = int(1/timeStep)//5
            data_dict['joint_pos_log']=joint_pos_log
            data_dict['joint_vel_log']=joint_vel_log
            data_dict['ee_log']=ee_log
            with open('data.json', 'wb') as fp:
                pickle.dump(data_dict, fp)
            break