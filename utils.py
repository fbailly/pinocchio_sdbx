import os.path

import pinocchio as pin
import hppfcl as fcl
from pinocchio.utils import np, rotate
import matplotlib.pyplot as plt
import warnings
from os.path import dirname, exists, join, abspath
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer


pin.switchToNumpyMatrix()
DENSITY = 1


def _depr_msg(deprecated, key):
    return "`%s` is deprecated. Please use `load('%s')`" % (deprecated, key)


class RobotLoader(object):
    path = ''
    urdf_filename = ''
    urdf_subpath = ''
    free_flyer = False
    verbose = True

    def __init__(self):
        df_path = join(self.path, self.urdf_subpath, self.urdf_filename)
        builder = RobotWrapper.BuildFromURDF
        self.model_path = abspath(".")
        self.df_path = df_path
        self.robot = builder(self.df_path, self.model_path,
                             pin.JointModelFreeFlyer() if self.free_flyer else None, verbose=True)
        self.srdf_path = None
        self.robot.q0 = pin.neutral(self.robot.model)

def create_N_pendulum(N, seq=None):
    model = pin.Model()
    geom_model = pin.GeometryModel()
    parent_id = 0
    base_radius = 0.2
    shape_base = fcl.Sphere(base_radius)
    geom_base = pin.GeometryObject("base", 0, shape_base, pin.SE3.Identity())
    geom_base.meshColor = np.array([1., 0.1, 0.1, 1.])
    geom_model.addGeometryObject(geom_base)

    joint_placement = pin.SE3.Identity()
    body_mass = 1.
    body_radius = 0.1

    for k in range(N):
        joint_name = "joint_" + str(k+1)
        if seq is None:
            joint_type = pin.JointModelRX()
        elif seq[k] == 'x':
            joint_type = pin.JointModelRX()
        elif seq[k] == 'y':
            joint_type = pin.JointModelRY()
        elif seq[k] == 'z':
            joint_type = pin.JointModelRZ()
        joint_id = model.addJoint(parent_id, joint_type, joint_placement, joint_name)
        # body_inertia = pin.Inertia.FromSphere(body_mass, body_radius)
        body_inertia = pin.Inertia.Identity()
        body_inertia.inertia = np.diag([0.025, 0.025, 0.2125])
        if k == 0:
            body_inertia.lever[2] = 0.5
        else:
            body_inertia.lever[2] = -0.5
        body_placement = joint_placement.copy()
        geom_placement = joint_placement.copy()
        if k == 0:
            geom_placement.translation[2] = 1
        else:
            body_placement.translation[2] = 1
            geom_placement.translation[2] = 1
        model.appendBodyToJoint(joint_id, body_inertia, body_placement)

        geom1_name = "ball_" + str(k+1)
        shape1 = fcl.Sphere(body_radius)
        geom1_obj = pin.GeometryObject(geom1_name, joint_id, shape1, geom_placement)
        geom1_obj.meshColor = np.ones((4))
        geom_model.addGeometryObject(geom1_obj)

        geom2_name = "bar_" + str(k+1)
        shape2 = fcl.Cylinder(body_radius/4., 1)
        shape2_placement = geom_placement.copy()
        shape2_placement.translation[2] /= 2.

        geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2, shape2_placement)
        geom2_obj.meshColor = np.array([0., 0., 0., 1.])
        geom_model.addGeometryObject(geom2_obj)

        parent_id = joint_id
        joint_placement = geom_placement.copy()
    return model, geom_model

def placement(x=0, y=0, z=0, rx=0, ry=0, rz=0):
    m = pin.SE3.Identity()
    m.translation = np.matrix([[float(i)] for i in [x, y, z]])
    m.rotation *= rotate('x', rx)
    m.rotation *= rotate('y', ry)
    m.rotation *= rotate('z', rz)
    return m


class DoublePendulumLoader(RobotLoader):
    path = abspath(".")
    urdf_filename = "pendulum.urdf"
    urdf_subpath = "urdf"


def loadDoublePendulum():
    return DoublePendulumLoader().robot


